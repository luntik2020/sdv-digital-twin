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
V2X_HAZARD_TOPIC = "sdv/v2x/hazard"

GEOFENCE_CENTER = [48.7220, 21.2575]  # lat, lng
GEOFENCE_RADIUS_METERS = 400.0

WAYPOINTS = {
    "car_001": [
        [21.2531, 48.7245], [21.2520, 48.7182], [21.2615, 48.7160],
        [21.2625, 48.7240], [21.2558, 48.7285], [21.2531, 48.7245]
    ],
    "car_002": [
        [21.2420, 48.7300], [21.2510, 48.7370], [21.2545, 48.7350],
        [21.2480, 48.7280], [21.2420, 48.7300]
    ],
    "car_003": [
        [21.2580, 48.7140], [21.2610, 48.7000], [21.2540, 48.7000],
        [21.2520, 48.7150], [21.2580, 48.7140]
    ]
}

def fetch_osrm_road_geometry(coords):
    try:
        coord_str = ";".join([f"{c[0]},{c[1]}" for c in coords])
        url = f"https://router.project-osrm.org/route/v1/driving/{coord_str}?overview=full&geometries=geojson"
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req, context=ctx, timeout=5) as response:
            data = json.loads(response.read().decode())
            geometry = data['routes'][0]['geometry']['coordinates']
            pts = [[p[1], p[0]] for p in geometry]
            print(f"[OSRM SUCCESS] Построено кольцо из {len(pts)} точек.")
            return pts
    except Exception as e:
        print(f"[OSRM WARN] OSRM недоступен ({e}). Использована локальная сетка.")
        dense_pts = []
        for i in range(len(coords) - 1):
            p1, p2 = coords[i], coords[i + 1]
            for step in range(20):
                t = step / 20.0
                lat = p1[1] + (p2[1] - p1[1]) * t
                lng = p1[0] + (p2[0] - p1[0]) * t
                dense_pts.append([lat, lng])
        return dense_pts

def haversine_distance(p1, p2):
    R = 6371000.0
    phi1, phi2 = math.radians(p1[0]), math.radians(p2[0])
    dphi = math.radians(p2[0] - p1[0])
    dlambda = math.radians(p2[1] - p1[1])
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def prepare_fleet_routes(fleet_data):
    for car_id, car in fleet_data.items():
        pts = car["route_points"]
        cum_dist = [0.0]
        for i in range(len(pts) - 1):
            d = haversine_distance(pts[i], pts[i+1])
            cum_dist.append(cum_dist[-1] + d)
        car["cum_dist"] = cum_dist
        car["total_dist"] = cum_dist[-1] if cum_dist[-1] > 0 else 1.0
        car["dist_along_route"] = 0.0

def get_position_at_distance(pts, cum_dist, total_dist, target_d):
    target_d = target_d % total_dist
    for i in range(len(cum_dist) - 1):
        if cum_dist[i] <= target_d <= cum_dist[i+1]:
            seg_len = cum_dist[i+1] - cum_dist[i]
            if seg_len < 0.0001:
                return pts[i][0], pts[i][1]
            t = (target_d - cum_dist[i]) / seg_len
            lat = pts[i][0] + (pts[i+1][0] - pts[i][0]) * t
            lng = pts[i][1] + (pts[i+1][1] - pts[i][1]) * t
            return lat, lng
    return pts[0][0], pts[0][1]

print("[INIT] Загрузка кольцевых автодорог Кошице...")

fleet = {
    "car_001": {
        "speed": 0.0, "target_speed": 50.0, "cruise_speed": 50.0,
        "battery": 95.0, "temp": 25.0, "odo": 15420.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_001"]),
        "state": "CRUISING", "stop_timer": 0,
        "sw_version": "v1.0.0-STD", "ota_status": "IDLE", "ota_progress": 0, "pending_ota": None,
        "dtc_codes": [], "v2x_warning": None, "v2x_timestamp": 0.0,
        "recirc_on": False, "smoke_hazard": False,
        "security": {
            "immobilized": False, "deadlock": False, "panic_alarm": False,
            "stealth_tracking": False, "rf_jamming": False, "geofence_lockdown": False
        },
        "tire_pressures": {"fl": 2.20, "fr": 2.20, "rl": 2.20, "rr": 2.20}
    },
    "car_002": {
        "speed": 40.0, "target_speed": 50.0, "cruise_speed": 55.0,
        "battery": 78.0, "temp": 29.0, "odo": 8410.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_002"]),
        "state": "CRUISING", "stop_timer": 0,
        "sw_version": "v1.0.0-STD", "ota_status": "IDLE", "ota_progress": 0, "pending_ota": None,
        "dtc_codes": [], "v2x_warning": None, "v2x_timestamp": 0.0,
        "recirc_on": False, "smoke_hazard": False,
        "security": {
            "immobilized": False, "deadlock": False, "panic_alarm": False,
            "stealth_tracking": False, "rf_jamming": False, "geofence_lockdown": False
        },
        "tire_pressures": {"fl": 2.25, "fr": 2.25, "rl": 2.25, "rr": 2.25}
    },
    "car_003": {
        "speed": 60.0, "target_speed": 70.0, "cruise_speed": 70.0,
        "battery": 34.0, "temp": 33.0, "odo": 41200.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_003"]),
        "state": "CRUISING", "stop_timer": 0,
        "sw_version": "v1.0.0-STD", "ota_status": "IDLE", "ota_progress": 0, "pending_ota": None,
        "dtc_codes": [], "v2x_warning": None, "v2x_timestamp": 0.0,
        "recirc_on": False, "smoke_hazard": False,
        "security": {
            "immobilized": False, "deadlock": False, "panic_alarm": False,
            "stealth_tracking": False, "rf_jamming": False, "geofence_lockdown": False
        },
        "tire_pressures": {"fl": 2.30, "fr": 2.30, "rl": 2.30, "rr": 2.30}
    }
}

prepare_fleet_routes(fleet)

def update_dtc_diagnostics(car_id):
    car = fleet[car_id]
    sec = car["security"]
    codes = []
    
    # Стандартные неисправности
    if car["faults"].get("overheat_battery"):
        codes.append({"code": "P0A7F", "desc": "Hybrid Battery Pack Over Temperature Critical", "severity": "CRITICAL"})
    if car["faults"].get("puncture_fl"):
        codes.append({"code": "C0031", "desc": "Left Front Wheel Pressure Signal Abnormal", "severity": "WARNING"})
    if car["faults"].get("open_door"):
        codes.append({"code": "B1203", "desc": "Passenger Door Emergency Unlock Override", "severity": "INFO"})
    if car["smoke_hazard"]:
        codes.append({"code": "B14A2", "desc": "HVAC Smoke Isolation - Recirculation Active", "severity": "WARNING"})
    if car["faults"].get("hv_fuse"):
        codes.append({"code": "P0A0A", "desc": "High Voltage Interlock Cutoff / Pyro-Fuse Fired", "severity": "CRITICAL"})
    if car["faults"].get("thermal_purge"):
        codes.append({"code": "P0A80", "desc": "Hybrid Battery Thermal Runaway Purge Active", "severity": "CRITICAL"})
    if car["faults"].get("safe_pullover"):
        codes.append({"code": "U0100", "desc": "Telematics Safe Pull-Over Override Initiated", "severity": "WARNING"})
    if car["faults"].get("brake_backup"):
        codes.append({"code": "C1203", "desc": "Primary Brake Failure - EPB Redundancy Engaged", "severity": "WARNING"})
    if car["faults"].get("crash_prep"):
        codes.append({"code": "B0001", "desc": "Crash Event - Window Micro-Drop & Strobes Active", "severity": "CRITICAL"})

    # Противоугонные DTC (ISO 21434 / ISO 14229 UDS)
    if sec["immobilized"]:
        codes.append({"code": "P1609", "desc": "Powertrain Anti-Theft Remote Immobilizer Active", "severity": "CRITICAL"})
    if sec["geofence_lockdown"]:
        codes.append({"code": "U1201", "desc": "Geofence Security Perimeter Breach Lockdown", "severity": "WARNING"})
    if sec["deadlock"]:
        codes.append({"code": "B1108", "desc": "Interior Door Handle Security Bypass / Deadlock Engaged", "severity": "WARNING"})
    if sec["panic_alarm"]:
        codes.append({"code": "B10AD", "desc": "Remote Anti-Theft Panic Alarm & Strobes Active", "severity": "CRITICAL"})
    if sec["stealth_tracking"]:
        codes.append({"code": "U0140", "desc": "Stealth Tracking Mode Engaged - Display Subsystem Muted", "severity": "INFO"})
    if sec["rf_jamming"]:
        codes.append({"code": "U1115", "desc": "RF Signal Jamming Interference Detected - Autonomous Safeguard", "severity": "CRITICAL"})

    car["dtc_codes"] = codes

def process_ota_updates(car_id):
    car = fleet[car_id]
    if car["pending_ota"] and car["ota_status"] == "UPDATING":
        car["ota_progress"] += 20
        if car["ota_progress"] >= 100:
            ota_type = car["pending_ota"]
            if ota_type == "ota_sport":
                car["sw_version"] = "v1.1.0-SPORT"
                car["cruise_speed"] = 75.0
            elif ota_type == "ota_eco":
                car["sw_version"] = "v1.2.0-ECO"
                car["cruise_speed"] = 40.0
            elif ota_type == "ota_std":
                car["sw_version"] = "v1.0.0-STD"
                car["cruise_speed"] = 50.0

            car["ota_status"] = "SUCCESS"
            car["ota_progress"] = 100
            car["pending_ota"] = None

def update_ai_driver_behavior(car_id, client, current_lat, current_lng):
    car = fleet[car_id]
    sec = car["security"]

    # Автоматический контроль выезда за геозону (Geofence Perimeter Lockdown)
    dist_to_center = haversine_distance([current_lat, current_lng], GEOFENCE_CENTER)
    if dist_to_center > GEOFENCE_RADIUS_METERS:
        sec["geofence_lockdown"] = True
    else:
        sec["geofence_lockdown"] = False

    if car.get("v2x_warning") and car.get("v2x_timestamp"):
        if time.time() - car["v2x_timestamp"] > 10.0:
            car["v2x_warning"] = None

    if car["ota_status"] == "UPDATING":
        car["target_speed"] = max(15.0, car["speed"] * 0.8)
        car["state"] = "OTA FLASHING"
        return

    # Приоритет 1: Имобилайзер и критические блокировки
    if sec["immobilized"]:
        car["target_speed"] = 0.0
        car["state"] = "IMMOBILIZED (KILL-SWITCH)"
        return

    if car["faults"].get("hv_fuse"):
        car["target_speed"] = 0.0
        car["state"] = "HV PYRO CUTOFF"
        return

    if sec["panic_alarm"]:
        car["state"] = "PANIC ALARM STROBES"

    if sec["rf_jamming"]:
        car["target_speed"] = 0.0
        car["state"] = "RF JAMMED SAFEGUARD"
        return

    if car["faults"].get("crash_prep"):
        car["target_speed"] = 0.0
        car["state"] = "CRASH STROBES ACTIVE"
        return

    # Приоритет 2: Ограничения по геозоне и безопасному прижатию
    if sec["geofence_lockdown"]:
        car["target_speed"] = 15.0
        car["state"] = "GEOFENCE LOCKDOWN"
        return

    if car["faults"].get("safe_pullover"):
        car["target_speed"] = 0.0
        car["state"] = "SAFE PULLOVER"
        return

    if car["faults"].get("overheat_battery") or car["faults"].get("puncture_fl"):
        car["target_speed"] = 0.0
        car["state"] = "EMERGENCY STOP"
        
        v2x_payload = {
            "sender": car_id, "event": "HAZARD_STOP",
            "lat": current_lat, "lng": current_lng,
            "timestamp": time.time()
        }
        client.publish(V2X_HAZARD_TOPIC, json.dumps(v2x_payload))
        return

    if car["faults"].get("thermal_purge"):
        car["target_speed"] = 25.0
        car["state"] = "THERMAL PURGE"
        return

    if car["faults"].get("brake_backup"):
        car["target_speed"] = 30.0
        car["state"] = "EPB BRAKE BACKUP"
        return

    if car["smoke_hazard"]:
        car["state"] = "CABIN ISOLATED"

    if sec["stealth_tracking"]:
        car["state"] = "STEALTH TRACKING"

    if car["stop_timer"] > 0:
        car["stop_timer"] -= 1
        car["target_speed"] = 0.0
        car["state"] = "RED LIGHT STOP"
        return

    if car["v2x_warning"]:
        car["target_speed"] = 30.0
        car["state"] = "V2X SLOWDOWN"
        return

    curr_d = car["dist_along_route"]
    pts = car["route_points"]
    cum_d, tot_d = car["cum_dist"], car["total_dist"]

    lat1, lng1 = get_position_at_distance(pts, cum_d, tot_d, curr_d)
    lat2, lng2 = get_position_at_distance(pts, cum_d, tot_d, curr_d + 15)
    lat3, lng3 = get_position_at_distance(pts, cum_d, tot_d, curr_d + 30)

    v1 = (lat2 - lat1, lng2 - lng1)
    v2 = (lat3 - lat2, lng3 - lng2)
    len1, len2 = math.hypot(*v1), math.hypot(*v2)

    angle = 0.0
    if len1 > 0.000001 and len2 > 0.000001:
        dot = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)))
        angle = math.degrees(math.acos(dot))

    if angle > 25.0:
        car["target_speed"] = 20.0
        car["state"] = "CORNERING"
    else:
        if random.random() < 0.008:
            car["stop_timer"] = random.randint(4, 7)
            car["target_speed"] = 0.0
            car["state"] = "RED LIGHT STOP"
        else:
            car["target_speed"] = car["cruise_speed"] + random.uniform(-2, 2)
            if not car["smoke_hazard"] and not sec["stealth_tracking"]:
                car["state"] = "CRUISING"

def update_car_physics_and_position(car_id):
    car = fleet[car_id]
    speed_diff = car["target_speed"] - car["speed"]

    if speed_diff < 0:
        decel_rate = 12.0 if car["security"]["immobilized"] or car["faults"].get("hv_fuse") else 4.0
        car["speed"] = max(0.0, car["speed"] - decel_rate)
    elif speed_diff > 0:
        car["speed"] = min(car["target_speed"], car["speed"] + 4.0)

    if car["speed"] < 0.5 and car["target_speed"] == 0.0:
        car["speed"] = 0.0

    dist_moved = (car["speed"] / 3.6)
    car["dist_along_route"] = (car["dist_along_route"] + dist_moved) % car["total_dist"]

    lat, lng = get_position_at_distance(
        car["route_points"], car["cum_dist"], car["total_dist"], car["dist_along_route"]
    )
    return lat, lng

def update_tire_thermodynamics(car_id):
    car = fleet[car_id]
    speed = car["speed"]
    tp = car["tire_pressures"]
    target_warm_pressure = 2.15 + (speed / 100.0) * 0.30

    for key in ["fl", "fr", "rl", "rr"]:
        if key == "fl" and car["faults"].get("puncture_fl"):
            if tp[key] > 1.15:
                tp[key] -= 0.08
            else:
                tp[key] = 1.15 + random.uniform(-0.01, 0.01)
            continue

        diff = target_warm_pressure - tp[key]
        tp[key] += diff * 0.03 + random.uniform(-0.005, 0.005)

def on_connect(client, userdata, flags, rc, properties=None):
    print("[MQTT] Симулятор подключен к брокеру.")
    client.subscribe(COMMANDS_TOPIC)
    client.subscribe(V2X_HAZARD_TOPIC)

def on_message(client, userdata, msg):
    try:
        if msg.topic == V2X_HAZARD_TOPIC:
            payload = json.loads(msg.payload.decode())
            sender = payload.get("sender")
            event = payload.get("event")

            if event == "HAZARD_STOP":
                for car_id in fleet:
                    if car_id != sender:
                        fleet[car_id]["v2x_warning"] = f"HAZARD: Vehicle {sender} stopped!"
                        fleet[car_id]["v2x_timestamp"] = time.time()
            elif event == "HAZARD_CLEARED":
                for car_id in fleet:
                    if car_id != sender:
                        if fleet[car_id]["v2x_warning"] and sender in fleet[car_id]["v2x_warning"]:
                            fleet[car_id]["v2x_warning"] = None
            return

        topic_parts = msg.topic.split("/")
        car_id = topic_parts[-1]
        payload = json.loads(msg.payload.decode())
        command = payload.get("command")

        if car_id in fleet:
            print(f"[COMMAND RECEIVED] {car_id} -> {command}")
            sec = fleet[car_id]["security"]

            if command in ["ota_sport", "ota_eco", "ota_std"]:
                fleet[car_id]["pending_ota"] = command
                fleet[car_id]["ota_status"] = "UPDATING"
                fleet[car_id]["ota_progress"] = 0
            elif command in ["puncture_fl", "overheat_battery", "hv_fuse", "thermal_purge", "safe_pullover", "brake_backup", "crash_prep"]:
                fleet[car_id]["faults"][command] = True
            elif command == "toggle_doors":
                currently_closed = not fleet[car_id]["faults"].get("open_door", False)
                fleet[car_id]["faults"]["open_door"] = currently_closed
            elif command == "smoke_hazard":
                fleet[car_id]["smoke_hazard"] = True
                fleet[car_id]["recirc_on"] = True
            
            # Противоугонные команды
            elif command == "toggle_immobilizer":
                sec["immobilized"] = not sec["immobilized"]
            elif command == "toggle_deadlock":
                sec["deadlock"] = not sec["deadlock"]
                fleet[car_id]["faults"]["open_door"] = False  # Принудительное заклинивание дверей
            elif command == "toggle_panic":
                sec["panic_alarm"] = not sec["panic_alarm"]
            elif command == "toggle_stealth":
                sec["stealth_tracking"] = not sec["stealth_tracking"]
            elif command == "trigger_jamming":
                sec["rf_jamming"] = True

            elif command == "reset_faults":
                fleet[car_id]["faults"] = {}
                fleet[car_id]["security"] = {
                    "immobilized": False, "deadlock": False, "panic_alarm": False,
                    "stealth_tracking": False, "rf_jamming": False, "geofence_lockdown": False
                }
                fleet[car_id]["smoke_hazard"] = False
                fleet[car_id]["recirc_on"] = False
                fleet[car_id]["v2x_warning"] = None
                
                clear_payload = {"sender": car_id, "event": "HAZARD_CLEARED", "timestamp": time.time()}
                client.publish(V2X_HAZARD_TOPIC, json.dumps(clear_payload))
    except Exception as e:
        print("[ERROR] Ошибка команды:", e)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "SDV_Fleet_Simulator")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, PORT)
client.loop_start()

print("[READY] Симулятор автономных автомобилей запущен...")

try:
    while True:
        for car_id, data in fleet.items():
            lat, lng = update_car_physics_and_position(car_id)
            update_dtc_diagnostics(car_id)
            process_ota_updates(car_id)
            update_ai_driver_behavior(car_id, client, lat, lng)
            update_tire_thermodynamics(car_id)

            is_moving = data["speed"] > 0.5
            if is_moving:
                data["battery"] = max(0.0, data["battery"] - random.uniform(0.01, 0.03))
                data["odo"] += (data["speed"] / 3600)

            current_temp = 25.0 + (data["speed"] / 100) * 5.0 + random.uniform(0.1, 0.3)
            if data["faults"].get("overheat_battery"):
                current_temp = 68.5 + random.uniform(0.1, 0.8)

            doors_closed = False if data["faults"].get("open_door") else True
            gear_state = "P" if (data["speed"] == 0.0 and (len(data["faults"]) > 0 or data["security"]["immobilized"])) else "D"

            telemetry_payload = {
                "vehicle_id": car_id,
                "speed_kmh": round(data["speed"], 1),
                "battery_pct": round(data["battery"], 2),
                "battery_temp_c": round(current_temp, 1),
                "range_km": round(data["battery"] * 4.2, 1),
                "odometer_km": round(data["odo"], 2),
                "driver_state": data["state"],
                "sw_version": data["sw_version"],
                "ota_status": data["ota_status"],
                "ota_progress": data["ota_progress"],
                "dtc_codes": data["dtc_codes"],
                "v2x_warning": data["v2x_warning"],
                "recirc_on": data["recirc_on"],
                "smoke_hazard": data["smoke_hazard"],
                "security": data["security"],
                "location": {"lat": round(lat, 6), "lng": round(lng, 6)},
                "tires_bar": {
                    "fl": round(data["tire_pressures"]["fl"], 2),
                    "fr": round(data["tire_pressures"]["fr"], 2),
                    "rl": round(data["tire_pressures"]["rl"], 2),
                    "rr": round(data["tire_pressures"]["rr"], 2)
                },
                "status": {
                    "doors_closed": doors_closed,
                    "seatbelts_fastened": True,
                    "ac_on": not data["smoke_hazard"],
                    "gear": gear_state
                }
            }
            topic = f"{TELEMETRY_TOPIC_PREFIX}{car_id}"
            client.publish(topic, json.dumps(telemetry_payload))

        time.sleep(1)

except KeyboardInterrupt:
    print("Симулятор остановлен.")
    client.loop_stop()
    client.disconnect()
