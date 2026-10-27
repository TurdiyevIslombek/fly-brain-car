#!/bin/bash
# One time: installs everything this project needs on the Mac (Python libraries, arduino-cli, Arduino + ESP32 support).
set -e
cd "$(dirname "$0")"
chmod +x ./*.sh 2>/dev/null || true
ESP_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

echo "=== 1/4  Python ==="
if ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null; then
  echo "Python 3.9 or newer is needed. Install it from https://www.python.org/downloads/ (or run: xcode-select --install), then run this again."
  exit 1
fi
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

echo "=== 2/4  arduino-cli (uploads code to the Arduino and the ESP32-CAM) ==="
mkdir -p tools/bin
if [ ! -x tools/bin/arduino-cli ]; then
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$PWD/tools/bin" sh || true
fi
if [ ! -x tools/bin/arduino-cli ]; then
  if [ "$(uname -m)" = "arm64" ]; then ARCH=macOS_ARM64; else ARCH=macOS_64bit; fi
  curl -fL -o /tmp/arduino-cli.tgz "https://github.com/arduino/arduino-cli/releases/download/v1.1.1/arduino-cli_1.1.1_${ARCH}.tar.gz"
  tar -xzf /tmp/arduino-cli.tgz -C tools/bin arduino-cli
fi
ACLI=tools/bin/arduino-cli

echo "=== 3/4  Arduino Uno support + Servo library ==="
"$ACLI" core update-index --additional-urls "$ESP_URL"
"$ACLI" core install arduino:avr
"$ACLI" lib update-index
"$ACLI" lib install Servo

echo "=== 4/4  ESP32 support (big download: a few minutes) ==="
"$ACLI" core install esp32:esp32 --additional-urls "$ESP_URL"

echo
echo "Setup done. Next: plug in the Arduino and run   bash upload_car.sh"
