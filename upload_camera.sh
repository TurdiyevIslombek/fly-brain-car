#!/bin/bash
# Uploads esp32cam/camera_stream to the ESP32-CAM (sitting on its ESP32-CAM-MB USB board).
# Unplug the Arduino while you do this.
set -e
cd "$(dirname "$0")"
ACLI=tools/bin/arduino-cli; [ -x "$ACLI" ] || ACLI=arduino-cli
CUR=$(sed -n 's/^#define WIFI_NAME "\(.*\)"$/\1/p' esp32cam/camera_stream/wifi_settings.h)
echo "Camera WiFi now: ${CUR:-none: the camera makes its own WiFi \"FlyCar-CAM\" (password change-me-123)}"
read -r -p "Change it? [y/N] " ANSWER
if [[ "$ANSWER" =~ ^[Yy] ]]; then
  read -r -p "WiFi name (2.4 GHz only; leave empty = camera makes its own WiFi): " NAME
  PASS=""
  if [ -n "$NAME" ]; then read -r -s -p "WiFi password: " PASS; echo; fi
  .venv/bin/python tools/write_wifi.py "$NAME" "$PASS"
fi
PORT="${1:-$(.venv/bin/python tools/find_port.py esp32)}"
if [ -z "$PORT" ]; then
  echo "No ESP32-CAM found. Put it on the ESP32-CAM-MB board and plug the USB cable into the Mac (a data cable)."
  exit 1
fi
echo "Uploading esp32cam/camera_stream to $PORT (about 1 minute) ..."
if ! "$ACLI" compile --fqbn esp32:esp32:esp32cam --upload -p "$PORT" esp32cam/camera_stream; then
  echo
  echo "Upload failed. Hold the IO0 button on the MB board, press and release RST, let go of IO0, then run this again."
  exit 1
fi
.venv/bin/python tools/read_serial.py "$PORT" 30
