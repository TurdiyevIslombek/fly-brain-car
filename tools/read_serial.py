"""Shows what the ESP32-CAM prints (its address).  Usage: read_serial.py PORT [seconds]"""
import sys
import time
import serial

port = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25
s = serial.Serial()
s.port, s.baudrate, s.timeout = port, 115200, 0.2
s.dtr = s.rts = False                     # keep the ESP32-CAM-MB from holding the board in reset
s.open()
s.rts = True                              # short reset pulse: restart the camera program
time.sleep(0.1)
s.rts = False
print(f"--- camera messages ({seconds:.0f} s) ---")
end = time.time() + seconds
while time.time() < end:
    line = s.readline().decode(errors="ignore").rstrip()
    if line:
        print(line)
        if line.startswith("CAMERA READY"):
            print("\nOpen in Safari:", line.split()[2].replace(":81/stream", ""), "  (the Mac must be on the same WiFi)")
            break
s.close()
