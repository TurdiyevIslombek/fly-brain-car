"""Stage 2: one ultrasonic sensor on a servo sweeps 5 directions like a radar (LL L C R RR).
Looming in each direction -> the fly's eye neurons -> fly brain -> wheels.
Left eye = the left directions (LL, L) + straight ahead; right eye = R, RR + straight ahead."""
import time
import config
from common.brain_driver import looming_hz
from common.reflex import StuckReflex, Window, STUCK_S
from common.robot_link import DIRS

WIDE_WEIGHT = 0.6     # the 70-degree directions count a bit less: they are beside the car, not ahead

MOVED_CM = 10         # a direction changed more than this in STUCK_S seconds = the car moved
                      # (bigger than sonar noise: close to a wall readings jump 6 -> 14 -> 7 cm)


class SonarEyes:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last = {d: None for d in DIRS}       # (cm, arduino ms, laptop time) of the latest ping
        self.loom = {d: 0.0 for d in DIRS}
        self.recent = Window(STUCK_S)             # for the stuck reflex

    def no_movement(self):
        """True if, during the last STUCK_S seconds, every direction stayed within MOVED_CM.
        Also True if EVERY ping said "nothing" (400): in a 2 m arena that never happens while driving
        freely - it means the sensor is pressed against something and hears no echo.
        False if something changed, or if we can't tell (too few echoes)."""
        allv = [cm for d in DIRS for cm in self.recent.values(d, STUCK_S)]
        if len(allv) >= 6 and min(allv) >= 400:
            return True
        seen = 0
        for d in DIRS:
            v = [cm for cm in self.recent.values(d, STUCK_S) if cm < 400]
            if len(v) >= 2:
                seen += 1
                if max(v) - min(v) > MOVED_CM:
                    return False
        return seen >= 2

    def open_side_is_right(self):
        cm = lambda d: self.last[d][0] if self.last[d] else 400
        side = lambda a, b: min(cm(a), 200) + min(cm(b), 200)
        return side("R", "RR") >= side("L", "LL")

    def on_ping(self, d, cm, ms):
        prev = self.last[d]
        if prev is None:
            self.loom[d] = looming_hz(cm, cm, 1.0)
        else:
            self.loom[d] = looming_hz(cm, prev[0], (ms - prev[1]) / 1000.0)
        self.last[d] = (cm, ms, time.time())

    def eyes(self):
        now = time.time()
        loom = {d: (0.0 if self.last[d] is None or now - self.last[d][2] > config.STALE_S else self.loom[d])
                for d in DIRS}
        left = min(config.MAX_HZ, loom["C"] + loom["L"] + WIDE_WEIGHT * loom["LL"])
        right = min(config.MAX_HZ, loom["C"] + loom["R"] + WIDE_WEIGHT * loom["RR"])
        return left, right


class Stage2:
    number = 2
    title = "Stage 2: fly brain + ultrasonic sensor on a servo"

    def __init__(self, robot, driver):
        self.robot, self.driver = robot, driver
        self.sonar = SonarEyes()
        self.reflex = StuckReflex()

    def start(self):
        self.sonar.reset()
        self.reflex.reset()
        self.driver.reset()
        self.robot.mode(2)

    def stop(self):
        pass

    def on_message(self, msg):
        if msg[0] == "P":
            self.sonar.recent.add(msg[1], msg[2])
            self.sonar.on_ping(msg[1], msg[2], msg[3])

    def tick(self, swap):
        eye_L, eye_R = self.sonar.eyes()
        if swap:
            eye_L, eye_R = eye_R, eye_L
        spikes, hz, brain_wheels = self.driver.tick(eye_L, eye_R)
        wheels, reflex = self.reflex.wheels(brain_wheels, self.sonar.open_side_is_right(), self.sonar.no_movement())
        self.robot.wheels(*wheels)
        return {"wheels": wheels, "eyes": self.driver.eyes_in, "hz": hz, "spikes": spikes, "frame": None,
                "reflex": reflex}
