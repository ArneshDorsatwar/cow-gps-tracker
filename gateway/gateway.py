"""
Cow GPS Tracker — Raspberry Pi RFM69 Gateway
Receives packets from cow collars via RFM69HCW and forwards to the dashboard.

Wiring (RFM69HCW -> Raspberry Pi):
    VIN  -> Pin 17 (3.3V)
    GND  -> Pin 20 (GND)
    SCK  -> Pin 23 (GPIO 11, SCLK)
    MISO -> Pin 21 (GPIO 9, MISO)
    MOSI -> Pin 19 (GPIO 10, MOSI)
    CS   -> Pin 24 (GPIO 8, CE0)
    RST  -> Pin 22 (GPIO 25)
    G0   -> Pin 18 (GPIO 24)

Setup:
    sudo raspi-config  -> Interface Options -> SPI -> Enable
    pip install adafruit-circuitpython-rfm69 requests

Usage:
    python gateway.py
    python gateway.py --dashboard http://192.168.0.100:5000
"""

import time
import json
import sys
import requests

# RFM69 config — must match ESP32 firmware
NETWORK_ID = 100
GATEWAY_ID = 0
RF_FREQ_MHZ = 915.0

# Dashboard URL
DASHBOARD_URL = "http://localhost:5000"

def expand_json(short_json):
    """Expand shortened JSON keys from the collar."""
    key_map = {
        "id": "cow_id",
        "la": "lat",
        "lo": "lon",
        "al": "alt",
        "sp": "speed",
        "sa": "sats",
        "bh": "behavior",
        "av": "accel_var",
        "t": "time",
    }
    expanded = {}
    for k, v in short_json.items():
        new_key = key_map.get(k, k)
        expanded[new_key] = v
    return expanded


def forward_to_dashboard(data):
    """Send tracker data to the dashboard via HTTP API."""
    try:
        r = requests.post(f"{DASHBOARD_URL}/api/data", json=data, timeout=5)
        if r.status_code == 200:
            print(f"  -> Dashboard OK")
        else:
            print(f"  -> Dashboard error: {r.status_code}")
    except requests.exceptions.ConnectionError:
        print(f"  -> Dashboard not reachable at {DASHBOARD_URL}")
    except Exception as e:
        print(f"  -> Dashboard error: {e}")


def main():
    global DASHBOARD_URL

    for arg in sys.argv[1:]:
        if arg.startswith("--dashboard="):
            DASHBOARD_URL = arg.split("=", 1)[1]

    print("=" * 50)
    print("  Cow GPS Tracker — RFM69 Gateway")
    print("=" * 50)
    print(f"  Frequency:    {RF_FREQ_MHZ} MHz")
    print(f"  Network ID:   {NETWORK_ID}")
    print(f"  Gateway ID:   {GATEWAY_ID}")
    print(f"  Dashboard:    {DASHBOARD_URL}")
    print("=" * 50)

    try:
        import board
        import busio
        import digitalio
        import adafruit_rfm69
    except ImportError as e:
        print(f"\nError: {e}")
        print("Install with: pip install adafruit-circuitpython-rfm69")
        print("\nRunning in DEMO mode (no radio)...\n")
        demo_mode()
        return

    # Setup SPI
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    # Setup pins
    cs = digitalio.DigitalInOut(board.CE0)
    reset = digitalio.DigitalInOut(board.D25)

    # Initialize RFM69
    try:
        rfm69 = adafruit_rfm69.RFM69(spi, cs, reset, RF_FREQ_MHZ)
        rfm69.node = GATEWAY_ID
        rfm69.encryption_key = None  # No encryption for simplicity
        print(f"\nRFM69 initialized! Listening for packets...\n")
    except Exception as e:
        print(f"\nFailed to initialize RFM69: {e}")
        print("Check wiring and SPI is enabled (sudo raspi-config)")
        return

    packet_count = 0

    while True:
        packet = rfm69.receive(timeout=5.0)

        if packet is not None:
            packet_count += 1
            rssi = rfm69.last_rssi

            try:
                # Decode packet — skip the 4-byte header (target, sender, control, flags)
                text = packet[3:].decode("utf-8", errors="replace").strip()
                print(f"[Packet #{packet_count}] RSSI: {rssi} dBm")
                print(f"  Raw: {text}")

                if text.startswith("{"):
                    data = json.loads(text)
                    data = expand_json(data)
                    data["rssi"] = rssi
                    print(f"  Cow #{data.get('cow_id', '?')} | "
                          f"Lat: {data.get('lat', 0):.5f} | "
                          f"Lon: {data.get('lon', 0):.5f} | "
                          f"Behavior: {data.get('behavior', '?')}")
                    forward_to_dashboard(data)
                else:
                    print(f"  (not JSON, skipping)")

            except Exception as e:
                print(f"  Error parsing: {e}")
                print(f"  Raw bytes: {packet.hex()}")

            print()


def demo_mode():
    """Run without hardware for testing the gateway -> dashboard pipeline."""
    import random
    import math

    print("Generating fake packets every 3 seconds...\n")

    lat, lon = 33.7199, -112.1065
    behaviors = ["RESTING", "GRAZING", "GRAZING", "WALKING"]

    while True:
        behavior = random.choice(behaviors)
        speed = {"RESTING": 0, "GRAZING": 0.5, "WALKING": 1.5, "RUNNING": 4.0}[behavior]
        lat += random.gauss(0, 0.0001)
        lon += random.gauss(0, 0.0001)

        data = {
            "cow_id": 1,
            "lat": lat,
            "lon": lon,
            "alt": 410 + random.gauss(0, 2),
            "speed": speed + random.gauss(0, 0.2),
            "sats": random.randint(4, 10),
            "behavior": behavior,
            "accel_var": random.uniform(0.001, 0.3),
            "rssi": random.randint(-90, -40),
            "time": time.strftime("%H:%M:%S"),
        }

        print(f"[Demo] Cow #{data['cow_id']} | {behavior} | "
              f"({lat:.5f}, {lon:.5f}) | RSSI: {data['rssi']} dBm")
        forward_to_dashboard(data)
        time.sleep(3)


if __name__ == "__main__":
    main()
