"""Checks that the servo looks the right way. The car must be connected by USB, battery on."""
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config
from common.robot_link import RobotLink

robot = RobotLink(sys.argv[1] if len(sys.argv) > 1 else config.PORT)
print(f"Connected: {robot.port}")
robot.mode(0)
wrong = False
for d, name in (("C", "STRAIGHT AHEAD"), ("L", "a bit to the car's LEFT (35 degrees)"),
                ("LL", "far to the car's LEFT (70 degrees)"), ("R", "a bit to the car's RIGHT (35 degrees)"),
                ("RR", "far to the car's RIGHT (70 degrees)")):
    robot.look(d)
    time.sleep(0.6)
    answer = input(f"The sensor should now look {name}. Does it? [y/n] ").strip().lower()
    wrong |= answer.startswith("n")
robot.look("C")
time.sleep(0.4)
robot.close()
if wrong:
    print("\nFix in arduino/car/car.ino, then: bash upload_car.sh\n"
          "  left and right swapped   -> LEFT_SIGN = -1\n"
          "  'straight' is a bit off  -> change CENTER_DEG (e.g. 85 or 95)\n"
          "  turns too little / much  -> NEAR_DEG and WIDE_DEG")
else:
    print("\nServo OK.")
