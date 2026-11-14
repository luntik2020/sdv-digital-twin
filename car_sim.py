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

# Замкнутые кольцевые маршруты по улицам Кошице [lng, lat]
WAYPOINTS = {
    # car_001: Кольцо вокруг Старого Города (Moyzesova -> Štúrova -> Protifašistických bojovníkov -> Hviezdoslavova)
    "car_001": [
        [21.2531, 48.7245],
        [21.2520, 48.7182],
        [21.2615, 48.7160],
        [21.2625, 48.7240],
        [21.2558, 48.7285],
        [21.2531, 48.7245]  # Замыкание кольца
    ],
    # car_002: Район TUKE (Watsonova -> Komenského -> Letná -> Watsonova)
    "car_002": [
        [21.2420, 48.7300],
        [21.2510, 48.7370],
        [21.2545, 48.7350],
        [21.2480, 48.7280],
        [21.2420, 48.7300]  # Замыкание кольца
    ],
    # car_003: Южная магистраль Кошице (Južná trieda -> Rastislavova)
    "car_003": [
        [21.2580, 48.7140],
        [21.2610, 48.7000],
        [21.2540, 48.7000],
        [21.2520, 48.7150],
        [21.2580, 48.7140]  # Замыкание кольца
    ]
}

def fetch_osrm_road_geometry(coords):
    """ Запрашивает геометрию дорог OSRM для замкнутого кольца """
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
            pts = [[p[1], p[0]] for p in geometry]  # lat, lng
            print(f"[OSRM SUCCESS] Построено кольцо из {len(pts)} точек.")
            return pts
    except Exception as e:
        print(f"[OSRM WARN] OSRM недоступен ({e}). Использована локальная сетка.")
        dense_pts = []
        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i + 1]
            for step in range(20):
                t = step / 20.0
                lat = p1[1] + (p2[1] - p1[1]) * t
                lng = p1[0] + (p2[0] - p1[0]) * t
                dense_pts.append([lat, lng])
        return dense_pts

def haversine_distance(p1, p2):
    """ Расчет расстояния между GPS точками в метрах """
    R = 6371000.0
    phi1, phi2 = math.radians(p1[0]), math.radians(p2[0])
    dphi = math.radians(p2[0] - p1[0])
    dlambda = math.radians(p2[1] - p1[1])
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def prepare_fleet_routes(fleet_data):
    """ Предрасчет накопительной дистанции маршрута """
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
    """ Вычисляет точные координаты машины на дистанции target_d (в метрах) """
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
        "state": "CRUISING", "stop_timer": 0
    },
    "car_002": {
        "speed": 40.0, "target_speed": 50.0, "cruise_speed": 55.0,
        "battery": 78.0, "temp": 29.0, "odo": 8410.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_002"]),
        "state": "CRUISING", "stop_timer": 0
    },
    "car_003": {
        "speed": 60.0, "target_speed": 70.0, "cruise_speed": 70.0,
        "battery": 34.0, "temp": 33.0, "odo": 41200.0, "faults": {},
        "route_points": fetch_osrm_road_geometry(WAYPOINTS["car_003"]),
        "state": "CRUISING", "stop_timer": 0
    }
}

prepare_fleet_routes(fleet)

def update_ai_driver_behavior(car_id):
    """ ИИ-контроллер вождения и реакция на аварийные события """
    car = fleet[car_id]
    
    # 1. Приоритетный контроль критических неисправностей (Аварийная остановка ADAS)
    if car["faults"].get("overheat_battery") or car["faults"].get("puncture_fl"):
        car["target_speed"] = 0.0
        car["state"] = "EMERGENCY STOP"
        return

    # 2. Остановка на красный свет светофора
    if car["stop_timer"] > 0:
        car["stop_timer"] -= 1
        car["target_speed"] = 0.0
        car["state"] = "RED LIGHT STOP"
        return

    curr_d = car["dist_along_route"]
    pts = car["route_points"]
    cum_d = car["cum_dist"]
    tot_d = car["total_dist"]

    lat1, lng1 = get_position_at_distance(pts, cum_d, tot_d, curr_d)
    lat2, lng2 = get_position_at_distance(pts, cum_d, tot_d, curr_d + 15)
    lat3, lng3 = get_position_at_distance(pts, cum_d, tot_d, curr_d + 30)

    v1 = (lat2 - lat1, lng2 - lng1)
    v2 = (lat3 - lat2, lng3 - lng2)
    len1 = math.hypot(*v1)
    len2 = math.hypot(*v2)

    angle = 0.0
    if len1 > 0.000001 and len2 > 0.000001:
        dot = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
        dot = max(-1.0, min(1.0, dot))
        angle = math.degrees(math.acos(dot))

    # 3. Притормаживание на повороте
    if angle > 25.0:
        car["target_speed"] = 20.0
        car["state"] = "CORNERING"
    else:
        # Случайная остановка на светофоре (0.8% шанс в сек)
        if random.random() < 0.008:
            car["stop_timer"] = random.randint(4, 7)
            car["target_speed"] = 0.0
            car["state"] = "RED LIGHT STOP"
            print(f"[TRAFFIC AI] {car_id} остановился на красный свет.")
        else:
            car["target_speed"] = car["cruise_speed"] + random.uniform(-2, 2)
            car["state"] = "CRUISING"

def update_car_physics_and_position(car_id):
    """ Постепенная физика торможения/разгона и расчет позиции """
    car = fleet[car_id]

    speed_diff = car["target_speed"] - car["speed"]
    if speed_diff < 0:
        # Постепенное аварийное торможение со скоростью 4.0 км/ч в секунду
        decel_rate = 4.0 if car["state"] == "EMERGENCY STOP" else 8.0
        car["speed"] = max(0.0, car["speed"] - decel_rate)
    elif speed_diff > 0:
        car["speed"] = min(car["target_speed"], car["speed"] + 4.0)

    if car["speed"] < 0.5 and car["target_speed"] == 0.0:
        car["speed"] = 0.0

    # Продвижение вперед в метрах за 1 секунду
    dist_moved = (car["speed"] / 3.6)
    car["dist_along_route"] = (car["dist_along_route"] + dist_moved) % car["total_dist"]

    lat, lng = get_position_at_distance(
        car["route_points"], car["cum_dist"], car["total_dist"], car["dist_along_route"]
    )

    return lat, lng

def on_connect(client, userdata, flags, rc, properties=None):
    print("[MQTT] Симулятор подключен к брокеру.")
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
            update_ai_driver_behavior(car_id)
            lat, lng = update_car_physics_and_position(car_id)

            is_moving = data["speed"] > 0.5

            if is_moving:
                data["battery"] = max(0.0, data["battery"] - random.uniform(0.01, 0.03))
                data["odo"] += (data["speed"] / 3600)

            current_temp = 25.0 + (data["speed"] / 100) * 5.0 + random.uniform(0.1, 0.3)
            tire_fl = 1.2 if data["faults"].get("puncture_fl") else round(random.uniform(2.1, 2.3), 2)
            if data["faults"].get("overheat_battery"):
                current_temp = 68.5 + random.uniform(0.1, 0.8)

            doors_closed = False if data["faults"].get("open_door") else True

            # Режим передачи КПП: P включается ТОЛЬКО когда машина полностью остановилась (speed == 0.0)
            if data["speed"] == 0.0 and (data["state"] == "EMERGENCY STOP" or len(data["faults"]) > 0):
                gear_state = "P"
            else:
                gear_state = "D"

            telemetry_payload = {
                "vehicle_id": car_id,
                "speed_kmh": round(data["speed"], 1),
                "battery_pct": round(data["battery"], 2),
                "battery_temp_c": round(current_temp, 1),
                "range_km": round(data["battery"] * 4.2, 1),
                "odometer_km": round(data["odo"], 2),
                "driver_state": data["state"],
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
