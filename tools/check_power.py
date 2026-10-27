"""Does the servo get enough power while the motors run?
Put the car on a box first: the wheels must NOT touch anything. Battery on.

The car sweeps its sensor (like stage 2) while the motors run at 0, half and full speed.
The Arduino measures its own 5 V rail (= the servo's power) the whole time.
Watch the sensor: it should keep sweeping left-right in all three parts."""
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config
from common.robot_link import RobotLink, DIRS

input("Car on a box, wheels in the air, battery on? Press Enter to start (Ctrl+C to quit). ")
robot = RobotLink(sys.argv[1] if len(sys.argv) > 1 else config.PORT)
print(f"Connected: {robot.port}\n")
robot.mode(2)
results = []
try:
    for name, speed in (("motors OFF ", 0), ("motors HALF", 130), ("motors FULL", 255)):
        print(f"{name}: 5 seconds ...", flush=True)
        end, low, pings, events = time.time() + 5, None, {d: [] for d in DIRS}, []
        while time.time() < end:
            robot.wheels(speed, speed)
            for m in robot.poll():
                if m[0] == "P":
                    pings[m[1]].append(m[2])
            events += robot.take_events()
            v = getattr(robot, "vcc_mv", 0)
            if v:
                low = v if low is None else min(low, v)
            time.sleep(0.05)
        swept = sum(1 for d in DIRS if pings[d])
        results.append((name, low, swept, events))
        robot.wheels(0, 0)
        time.sleep(0.8)
finally:
    robot.wheels(0, 0)
    robot.mode(0)
    robot.close()

print("\n                lowest 5 V    directions pinged")
for name, low, swept, events in results:
    v = "  (old car code?)" if low is None else f"{low / 1000:6.2f} V"
    print(f"  {name}    {v}        {swept} of {len(DIRS)}     {', '.join(events)}")
if any("ARDUINO RESTARTED" in e for r in results for e in r[3]):
    print("\nThe Arduino RESTARTED: the battery voltage fell so low that the Arduino switched off for a moment.")
lows = [low for _, low, _, _ in results if low]
if not lows:
    print("\nNo 5 V numbers: upload the new car code first (pull D0, bash upload_car.sh, D0 back).")
elif min(lows) < 4500:
    print("\n5 V DROPS when the motors run: that is why the servo stops turning while driving.\n"
          "Fix: charge the batteries; servo + to the LM2596 5 V output (not the Arduino 5V pin);\n"
          "a 470-1000 uF capacitor across the servo's + and - (long leg to +).")
else:
    print("\n5 V stays fine. If the sensor still did not sweep: check the servo signal wire (D3)\n"
          "and that the sensor's wires are not pulling it back (leave them loose).")
