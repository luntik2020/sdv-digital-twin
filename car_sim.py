import time
import json
import random
import math
import urllib.request
import ssl
import paho.mqtt.client as mqtt

BROKER_ADDRESS = "broker.emqx.io"
PORT = 1883
TELEMETRY_TOPIC_PREFIX = "sdv/telemetry/"
COMMANDS_TOPIC = "sdv/commands/+"

# Košice Waypoints [lng, lat]
WAYPOINTS = {
    # car_001: Moyzesova -> Štúrova -> Hlavná
    "car_001": [
        [21.2531, 48.7245], [21.2526, 48.7215], [21.2520, 48.7182], 
        [21.2545, 48.7170], [21.2580, 48.7162], [21.2575, 48.7200], 
        [21.2570, 48.7240], [21.2558, 48.7280], [21.2538, 48.7270]
    ],
    # car_002: TUKE Campus / Watsonova / Festivalové námestie
    "car_002": [
        [21.2415, 48.7282], [21.2452, 48.7325], [21.2498, 48.7362], 
        [21.2545, 48.7358], [21.2550, 48.7320], [21.2490, 48.7305]
    ],
    # car_003: Južná trieda / Rastislavova
    "car_003": [
        [21.2580, 48.7160], [21.2595, 48.7095], [21.2615, 48.7020], 
        [21.2630, 48.6960], [21.2615, 48.7020], [21.2595, 48.7095]
    ]
}

def fetch_osrm_road_geometry(coords):
    """ Fetches actual road network geometry for Košice via OSRM """
    try:
        coord_str = ";".join([f"{c[0]},{c[1]}" for c in coords])
        url = f"https://router.project-osrm.org/route/v1/driving/{coord_str}?overview=full&geometries=geojson"
        
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE

        req = urllib.request.Request(
            url, 
            headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        )
        with urllib.request.urlopen(req, context=ctx, timeout=5) as response:
            data = json.loads(response.read().decode())
            geometry = data['routes'][0]['geometry']['coordinates']
            pts = [[p[1], p[0]] for p in geometry] # lat, lng
            print(f"[OSRM SUCCESS] Loaded {len(pts)} road geometry nodes.")
            return pts + pts[::-1][1:-1]
    except Exception as e:
        print(f"[OSRM WARN] OSRM API unreachable ({e}). Using dense fallback grid.")
        dense_pts = []
        for i in range(len(coords)):
            p1 = coords[i]
            p2 = coords[(i + 1) % len(coords)]
            for step in range(12):
                t = step / 12.0
                lat = p1[1] + (p2[1] - p1[1]) * t
                lng = p1[0] + (p2[0] - p1[0]) * t
                dense_pts.append([lat, lng])
        return dense_pts

def calculate_turn_angle(p1, p2, p3):
    """ Calculates turn angle in degrees between three consecutive points """
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    len1 = math.hypot(*v1)
    len2 = math.hypot(*v2)
    if len1 == 0 or len2 == 0:
        return 0.0
    dot = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))

print("[INIT] Loading Košice road network for Autonomous AI Agents...")

fleet = {
    "car_001": {
        "speed": 0.0, "target_speed": 50.0, "cruise_speed": 55.0,
        "battery": 95.0, "temp": 25.0, "odo": 15420.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_001"]),
        "curr_idx": 0, "segment_progress": 0.0,
        "state": "CRUISING", "stop_timer": 0
    },
    "car_002": {
        "speed": 40.0, "target_speed": 50.0, "cruise_speed": 60.0,
        "battery": 78.0, "temp": 29.0, "odo": 8410.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_002"]),
        "curr_idx": 0, "segment_progress": 0.0,
        "state": "CRUISING", "stop_timer": 0
    },
    "car_003": {
        "speed": 60.0, "target_speed": 80.0, "cruise_speed": 85.0,
        "battery": 34.0, "temp": 33.0, "odo": 41200.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_003"]),
        "curr_idx": 0, "segment_progress": 0.0,
        "state": "CRUISING", "stop_timer": 0
    }
}

def update_ai_driver_behavior(car_id):
    """ Autonomous AI Driving Logic Engine """
    car = fleet[car_id]
    route = car["route_points"]
    if not route or len(route) < 3:
        return

    # Check if vehicle is stopped at red light / intersection
    if car["stop_timer"] > 0:
        car["stop_timer"] -= 1
        car["target_speed"] = 0.0
        car["state"] = "RED LIGHT STOP"
        return

    idx = car["curr_idx"]
    p1 = route[idx]
    p2 = route[(idx + 1) % len(route)]
    p3 = route[(idx + 2) % len(route)]

    angle = calculate_turn_angle(p1, p2, p3)

    # 1. Detect sharp turn ahead
    if angle > 35.0:
        car["target_speed"] = max(15.0, car["cruise_speed"] * 0.35)
        car["state"] = "CORNERING"
    else:
        # 2. Random chance to encounter Red Light / Traffic Junction when switching segments
        if car["segment_progress"] < 0.1 and random.random() < 0.08:
            car["stop_timer"] = random.randint(3, 6) # Stop for 3-6 seconds
            car["target_speed"] = 0.0
            car["state"] = "RED LIGHT STOP"
            print(f"[TRAFFIC AI] {car_id} stopped at Traffic Light / Intersection.")
            return
        else:
            car["target_speed"] = car["cruise_speed"] + random.uniform(-3, 3)
            car["state"] = "CRUISING"

def update_car_position(car_id):
    """ Moves car along route using smooth physics & AI speed controller """
    car = fleet[car_id]
    route = car["route_points"]
    if not route or len(route) < 2:
        return 48.7220, 21.2575

    # Smooth Acceleration / Deceleration Controller (Max 6 km/h change per sec)
    speed_diff = car["target_speed"] - car["speed"]
    accel_limit = 6.0
    if abs(speed_diff) > 0.1:
        car["speed"] += max(-accel_limit, min(accel_limit, speed_diff))

    idx = car["curr_idx"]
    next_idx = (idx + 1) % len(route)
    p1 = route[idx]
    p2 = route[next_idx]

    d_lat = p2[0] - p1[0]
    d_lng = p2[1] - p1[1]
    seg_len = math.sqrt(d_lat**2 + d_lng**2)

    if seg_len < 0.0000001:
        car["curr_idx"] = next_idx
        car["segment_progress"] = 0.0
        return p2[0], p2[1]

    speed_deg_sec = (car["speed"] / 3600.0) / 111.0
    car["segment_progress"] += speed_deg_sec / seg_len

    while car["segment_progress"] >= 1.0:
        car["segment_progress"] -= 1.0
        car["curr_idx"] = (car["curr_idx"] + 1) % len(route)
        update_ai_driver_behavior(car_id)

        idx = car["curr_idx"]
        next_idx = (idx + 1) % len(route)
        p1 = route[idx]
        p2 = route[next_idx]
        d_lat = p2[0] - p1[0]
        d_lng = p2[1] - p1[1]
        seg_len = math.sqrt(d_lat**2 + d_lng**2)
        if seg_len < 0.0000001:
            car["segment_progress"] = 1.0
            break

    prog = car["segment_progress"]
    lat = p1[0] + (p2[0] - p1[0]) * prog
    lng = p1[1] + (p2[1] - p1[1]) * prog

    return lat, lng

def on_connect(client, userdata, flags, rc, properties=None):
    print("[MQTT] Simulator connected successfully. Listening for commands...")
    client.subscribe(COMMANDS_TOPIC)

def on_message(client, userdata, msg):
    try:
        topic_parts = msg.topic.split("/")
        car_id = topic_parts[-1]
        payload = json.loads(msg.payload.decode())
        command = payload.get("command")

        if car_id in fleet:
            print(f"[FAULT INJECTION] {car_id} -> {command}")
            if command == "puncture_fl":
                fleet[car_id]["faults"]["puncture_fl"] = True
            elif command == "overheat_battery":
                fleet[car_id]["faults"]["overheat_battery"] = True
            elif command == "open_door":
                fleet[car_id]["faults"]["open_door"] = True
            elif command == "reset_faults":
                fleet[car_id]["faults"] = {}
    except Exception as e:
        print("[ERROR] Error processing incoming command:", e)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "SDV_Fleet_Simulator")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, PORT)
client.loop_start()

print("[READY] Autonomous Vehicle Simulator (Traffic AI & ADAS) running...")

try:
    while True:
        for car_id, data in fleet.items():
            is_moving = data["speed"] > 0.5

            if is_moving:
                data["battery"] = max(0.0, data["battery"] - random.uniform(0.01, 0.03))
                data["odo"] += (data["speed"] / 3600)

            lat, lng = update_car_position(car_id)

            current_temp = 25.0 + (data["speed"] / 100) * 5.0 + random.uniform(0.1, 0.3)
            tire_fl = 1.2 if data["faults"].get("puncture_fl") else round(random.uniform(2.1, 2.3), 2)
            if data["faults"].get("overheat_battery"):
                current_temp = 68.5 + random.uniform(0.1, 0.8)

            doors_closed = False if data["faults"].get("open_door") else True

            telemetry_payload = {
                "vehicle_id": car_id,
                "speed_kmh": round(data["speed"], 1),
                "battery_pct": round(data["battery"], 2),
                "battery_temp_c": round(current_temp, 1),
                "range_km": round(data["battery"] * 4.2, 1),
                "odometer_km": round(data["odo"], 2),
                "driver_state": data["state"], # ADAS AI State
                "location": {
                    "lat": round(lat, 6),
                    "lng": round(lng, 6)
                },
                "tires_bar": {
                    "fl": tire_fl,
                    "fr": round(random.uniform(2.1, 2.3), 2),
                    "rl": round(random.uniform(2.2, 2.4), 2),
                    "rr": round(random.uniform(2.2, 2.4), 2)
                },
                "status": {
                    "doors_closed": doors_closed,
                    "seatbelts_fastened": True,
                    "ac_on": True,
                    "gear": "D" if is_moving else "P"
                }
            }

            topic = f"{TELEMETRY_TOPIC_PREFIX}{car_id}"
            client.publish(topic, json.dumps(telemetry_payload))

        time.sleep(1)

except KeyboardInterrupt:
    print("Simulator stopped.")
    client.loop_stop()
    client.disconnect()
