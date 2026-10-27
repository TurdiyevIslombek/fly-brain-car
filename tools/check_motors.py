"""Checks the wheels. Lift the car so the wheels are in the air. USB connected, battery on."""
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config
from common.robot_link import RobotLink

robot = RobotLink(sys.argv[1] if len(sys.argv) > 1 else config.PORT)
print(f"Connected: {robot.port}")
robot.mode(2)
time.sleep(0.2)
tests = [("LEFT wheels FORWARD", 180, 0), ("RIGHT wheels FORWARD", 0, 180),
         ("both FORWARD", 180, 180), ("both BACKWARD", -180, -180)]
bad = []
for name, left, right in tests:
    input(f"\nPress Enter, then watch: {name} for 1.5 s ")
    end = time.time() + 1.5
    while time.time() < end:
        robot.wheels(left, right)            # repeat: the Arduino stops after 0.3 s without a command
        time.sleep(0.1)
    robot.wheels(0, 0)
    if input("Was that right? [y/n] ").strip().lower().startswith("n"):
        bad.append(name)
robot.close()
if bad:
    print("\nWrong:", ", ".join(bad))
    print("A side turns BACKWARD instead of forward: swap that motor pair's two wires on the L298N (OUT1/OUT2 or OUT3/OUT4).")
    print("LEFT and RIGHT are mixed up: swap the ENA/IN1/IN2 wires with ENB/IN3/IN4, or the motor wires OUT1/2 with OUT3/4.")
    print("A side doesn't move: ENA/ENB jumper still on? Battery on? Loose wire?")
else:
    print("\nMotors OK.")
