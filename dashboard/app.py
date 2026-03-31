"""
Cow GPS Tracker Dashboard
Backend server that receives data from ESP32 (via serial or HTTP)
and serves a real-time web dashboard.
"""

import json
import time
import threading
import random
import math
import sys
import os
from datetime import datetime, timezone

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO

app = Flask(__name__)
app.config["SECRET_KEY"] = "cow-tracker-2026"
socketio = SocketIO(app, cors_allowed_origins="*")

# ── In-memory data store ──
tracker_data = {}  # cow_id -> latest data
tracker_history = {}  # cow_id -> list of recent positions
HISTORY_MAX = 200  # keep last N positions per cow

# ── Geofence (editable from dashboard) ──
geofence = {
    "center_lat": 33.7199,
    "center_lon": -112.1065,
    "radius_m": 500,
    "enabled": True,
}

alerts = []  # list of alert dicts
ALERTS_MAX = 50


def add_alert(cow_id, alert_type, message):
    alert = {
        "cow_id": cow_id,
        "type": alert_type,
        "message": message,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    alerts.insert(0, alert)
    if len(alerts) > ALERTS_MAX:
        alerts.pop()
    socketio.emit("new_alert", alert)


def check_geofence(cow_id, lat, lon):
    if not geofence["enabled"] or lat == 0 or lon == 0:
        return
    dlat = lat - geofence["center_lat"]
    dlon = (lon - geofence["center_lon"]) * math.cos(math.radians(lat))
    dist_m = math.sqrt(dlat**2 + dlon**2) * 111320
    if dist_m > geofence["radius_m"]:
        add_alert(cow_id, "geofence", f"Cow {cow_id} left geofence! Distance: {dist_m:.0f}m")


def process_tracker_update(data):
    cow_id = data.get("cow_id", 1)
    data["received_at"] = datetime.now(timezone.utc).isoformat()

    tracker_data[cow_id] = data

    if cow_id not in tracker_history:
        tracker_history[cow_id] = []

    if data.get("lat", 0) != 0:
        tracker_history[cow_id].append({
            "lat": data["lat"],
            "lon": data["lon"],
            "behavior": data.get("behavior", "UNKNOWN"),
            "timestamp": data["received_at"],
        })
        if len(tracker_history[cow_id]) > HISTORY_MAX:
            tracker_history[cow_id] = tracker_history[cow_id][-HISTORY_MAX:]

        check_geofence(cow_id, data["lat"], data["lon"])

    # Check for abnormal behavior
    behavior = data.get("behavior", "")
    if behavior == "RUNNING":
        add_alert(cow_id, "behavior", f"Cow {cow_id} is RUNNING — possible distress")

    socketio.emit("tracker_update", data)


# ── Serial reader (reads JSON from ESP32 USB) ──
def serial_reader(port, baud=115200):
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[Serial] Connected to {port} at {baud} baud")
        while True:
            line = ser.readline()
            if line:
                try:
                    text = line.decode("utf-8", errors="replace").strip()
                    if text.startswith("{"):
                        data = json.loads(text)
                        process_tracker_update(data)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass
    except Exception as e:
        print(f"[Serial] Error: {e}")
        print("[Serial] Running without serial — use simulation or HTTP API")


# ── Data simulator (generates fake cow data for testing) ──
def simulate_cows(num_cows=5):
    """Simulate multiple cows moving around a center point."""
    center_lat = 33.7199
    center_lon = -112.1065
    cow_states = {}

    for i in range(1, num_cows + 1):
        angle = (2 * math.pi * i) / num_cows
        cow_states[i] = {
            "lat": center_lat + 0.002 * math.sin(angle),
            "lon": center_lon + 0.002 * math.cos(angle),
            "heading": random.uniform(0, 360),
            "behavior": "GRAZING",
        }

    behaviors = ["RESTING", "GRAZING", "GRAZING", "GRAZING", "WALKING", "WALKING"]

    print(f"[Simulator] Simulating {num_cows} cows")
    while True:
        for cow_id, state in cow_states.items():
            # Occasionally change behavior
            if random.random() < 0.05:
                state["behavior"] = random.choice(behaviors)
                if random.random() < 0.02:
                    state["behavior"] = "RUNNING"

            # Move based on behavior
            speed = {
                "RESTING": 0.0,
                "GRAZING": 0.00001,
                "WALKING": 0.00005,
                "RUNNING": 0.0002,
            }.get(state["behavior"], 0)

            state["heading"] += random.gauss(0, 15)
            state["lat"] += speed * math.sin(math.radians(state["heading"]))
            state["lon"] += speed * math.cos(math.radians(state["heading"]))

            accel_var = {
                "RESTING": random.uniform(0.001, 0.004),
                "GRAZING": random.uniform(0.01, 0.04),
                "WALKING": random.uniform(0.05, 0.25),
                "RUNNING": random.uniform(0.3, 0.8),
            }.get(state["behavior"], 0)

            data = {
                "cow_id": cow_id,
                "lat": state["lat"],
                "lon": state["lon"],
                "alt": 410 + random.gauss(0, 2),
                "speed": speed * 111320,
                "sats": random.randint(4, 12),
                "behavior": state["behavior"],
                "accel_var": accel_var,
                "time": datetime.now(timezone.utc).strftime("%H:%M:%S"),
                "date": datetime.now(timezone.utc).strftime("%Y-%m-%d"),
            }
            process_tracker_update(data)

        time.sleep(2)


# ── Routes ──
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/data", methods=["POST"])
def api_data():
    """HTTP endpoint for receiving tracker data (alternative to serial)."""
    data = request.get_json()
    if data:
        process_tracker_update(data)
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "no data"}), 400


@app.route("/api/cows")
def api_cows():
    return jsonify(tracker_data)


@app.route("/api/history/<int:cow_id>")
def api_history(cow_id):
    return jsonify(tracker_history.get(cow_id, []))


@app.route("/api/alerts")
def api_alerts():
    return jsonify(alerts)


@app.route("/api/geofence", methods=["GET", "POST"])
def api_geofence():
    if request.method == "POST":
        data = request.get_json()
        geofence.update(data)
        socketio.emit("geofence_update", geofence)
        return jsonify({"status": "ok"})
    return jsonify(geofence)


# ── Main ──
if __name__ == "__main__":
    mode = "simulate"
    serial_port = None

    for arg in sys.argv[1:]:
        if arg.startswith("--serial="):
            serial_port = arg.split("=")[1]
            mode = "serial"
        elif arg == "--simulate":
            mode = "simulate"

    if mode == "serial" and serial_port:
        t = threading.Thread(target=serial_reader, args=(serial_port,), daemon=True)
        t.start()
    else:
        print("[Mode] Simulation mode — generating fake cow data")
        t = threading.Thread(target=simulate_cows, args=(5,), daemon=True)
        t.start()

    print("\n  Dashboard: http://localhost:5000\n")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
