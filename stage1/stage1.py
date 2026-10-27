"""Stage 1: the Arduino drives by itself with if-else rules (stage1() in arduino/car/car.ino).
The laptop only switches it on and shows what the sensor sees."""
import time


class Stage1:
    number = 1
    title = "Stage 1: if-else rules on the Arduino (no brain)"

    def __init__(self, robot):
        self.robot = robot
        self.wheels = (0, 0)
        self.reflex_until = 0.0

    def start(self):
        self.wheels = (0, 0)
        self.reflex_until = 0.0
        self.robot.mode(1)

    def stop(self):
        pass

    def on_message(self, msg):
        if msg[0] == "W":
            self.wheels = (msg[1], msg[2])
        elif msg[0] in ("STUCK", "TRAP"):          # the Arduino's stuck reflex / big escape
            self.reflex_until = time.time() + 1.5   # (it backs up, then turns)

    def tick(self, swap):
        return {"wheels": self.wheels, "eyes": None, "hz": None, "spikes": None, "frame": None,
                "reflex": time.time() < self.reflex_until}
