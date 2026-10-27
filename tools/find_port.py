"""Prints the USB port of the Arduino Uno (find_port.py uno) or the ESP32-CAM (find_port.py esp32)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.robot_link import find_ports

kind = sys.argv[1] if len(sys.argv) > 1 else "uno"
ports = find_ports(kind)
if len(ports) > 1:
    print(f"Several possible ports: {', '.join(ports)}. Using {ports[0]}.", file=sys.stderr)
print(ports[0] if ports else "")
