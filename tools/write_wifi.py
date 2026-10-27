"""Writes esp32cam/camera_stream/wifi_settings.h.  Usage: write_wifi.py "WiFi name" "password" """
import sys
from pathlib import Path

c = lambda s: s.replace("\\", "\\\\").replace('"', '\\"')
name = sys.argv[1] if len(sys.argv) > 1 else ""
password = sys.argv[2] if len(sys.argv) > 2 else ""
path = Path(__file__).resolve().parent.parent / "esp32cam" / "camera_stream" / "wifi_settings.h"
path.write_text(
    "// upload_camera.sh writes this file. Empty WIFI_NAME = the camera makes its own WiFi (AP_NAME).\n"
    f'#define WIFI_NAME "{c(name)}"\n'
    f'#define WIFI_PASSWORD "{c(password)}"\n'
    '#define AP_NAME "FlyCar-CAM"\n'
    '#define AP_PASSWORD "change-me-123"\n')
print(f"WiFi saved: {'the camera makes its own WiFi FlyCar-CAM' if not name else name}")
