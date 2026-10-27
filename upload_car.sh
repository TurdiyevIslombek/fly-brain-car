#!/bin/bash
# Uploads arduino/car/car.ino to the Arduino Uno. Close the car app first (it uses the same USB port).
set -e
cd "$(dirname "$0")"
ACLI=tools/bin/arduino-cli; [ -x "$ACLI" ] || ACLI=arduino-cli
PORT="${1:-$(.venv/bin/python tools/find_port.py uno)}"
if [ -z "$PORT" ]; then
  echo "No Arduino found. Plug in the USB cable (a data cable, not charge-only) and try again."
  exit 1
fi
echo "Uploading arduino/car to $PORT ..."
"$ACLI" compile --fqbn arduino:avr:uno --upload -p "$PORT" arduino/car
echo
echo "Done. Start the app:  bash run.sh"
