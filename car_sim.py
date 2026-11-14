import time
import json
import random
import math
import paho.mqtt.client as mqtt

BROKER_ADDRESS = "broker.emqx.io"
PORT = 1883
TELEMETRY_TOPIC_PREFIX = "sdv/telemetry/"
COMMANDS_TOPIC = "sdv/commands/+"

# База данных автопарка с стартовыми GPS-координатами
fleet = {
    "car_001": {"speed": 0.0, "battery": 95.0, "temp": 25.0, "odo": 15420.0, "faults": {}, "lat": 50.4501, "lng": 30.5234, "heading": 45.0},
    "car_002": {"speed": 60.0, "battery": 78.0, "temp": 29.0, "odo": 8410.0,  "faults": {}, "lat": 50.4580, "lng": 30.5350, "heading": 120.0},
    "car_003": {"speed": 110.0, "battery": 34.0, "temp": 33.0, "odo": 41200.0, "faults": {}, "lat": 50.4420, "lng": 30.5120, "heading": 270.0}
}

def on_connect(client, userdata, flags, rc, properties=None):
    print("Мульти-симулятор подключен к MQTT-брокеру. Подписка на канал команд...")
    client.subscribe(COMMANDS_TOPIC)

def on_message(client, userdata, msg):
    """Прием управляющих команд Fault Injection из веб-дашборда"""
    try:
        topic_parts = msg.topic.split("/")
        car_id = topic_parts[-1]
        payload = json.loads(msg.payload.decode())
        command = payload.get("command")

        if car_id in fleet:
            print(f"[FAULT INJECTION COMMAND] {car_id} -> {command}")
            if command == "puncture_fl":
                fleet[car_id]["faults"]["puncture_fl"] = True
            elif command == "overheat_battery":
                fleet[car_id]["faults"]["overheat_battery"] = True
            elif command == "open_door":
                fleet[car_id]["faults"]["open_door"] = True
            elif command == "reset_faults":
                fleet[car_id]["faults"] = {}
    except Exception as e:
        print("Ошибка обработки MQTT-команды:", e)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "SDV_Fleet_Simulator")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, PORT)
client.loop_start()

print("Симулятор SDV с GPS-трекингом и мульти-автопарком запущен...")

try:
    while True:
        for car_id, data in fleet.items():
            # Моделирование динамики скорости
            data["speed"] = max(0.0, min(140.0, data["speed"] + random.uniform(-3.0, 4.0)))
            is_moving = data["speed"] > 1.0
            
            if is_moving:
                data["battery"] = max(0.0, data["battery"] - random.uniform(0.01, 0.03))
                data["odo"] += (data["speed"] / 3600)
                
                # Расчет физики движения по GPS-координатам
                dist_km = (data["speed"] / 3600)
                data["heading"] = (data["heading"] + random.uniform(-4.0, 4.0)) % 360
                rad = math.radians(data["heading"])
                
                # Смещение широты и долготы (1 град lat ~ 111 км)
                data["lat"] += (dist_km / 111.0) * math.cos(rad)
                data["lng"] += (dist_km / (111.0 * math.cos(math.radians(data["lat"])))) * math.sin(rad)

            # Базовая температура и обработка сбоев
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
                "location": {
                    "lat": round(data["lat"], 6),
                    "lng": round(data["lng"], 6),
                    "heading": round(data["heading"], 1)
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
    print("Симулятор остановлен.")
    client.loop_stop()
    client.disconnect()