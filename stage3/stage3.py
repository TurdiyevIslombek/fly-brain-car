"""Stage 3: the ESP32-CAM is the fly's eye. Looming from the pictures -> fly brain -> wheels."""
import config
from common.camera import Camera
from common.reflex import StuckReflex, Window, STUCK_S
from stage3.vision import looming_from_frames

MOVING_PX_S = getattr(config, "CAM_MOVING_PX_S", 3.0)   # image motion above this = the car is moving
MOVED_CM = 10          # sonar straight ahead changed more than this = the car moved


class Stage3:
    number = 3
    title = "Stage 3: fly brain + camera"

    def __init__(self, robot, driver, camera_source):
        self.robot, self.driver, self.source = robot, driver, camera_source
        self.camera = None
        self.reflex = StuckReflex()
        self.recent = Window(STUCK_S)                     # image motion + sonar, for the stuck reflex
        self._clear()

    def _clear(self):
        self.prev, self.prev_t, self.last_n = None, 0.0, -1
        self.eyes, self.raw = (0.0, 0.0), {}
        self.hist = []                                    # last 3 eye measurements

    def start(self):
        if self.camera is None:
            self.camera = Camera(self.source, config.CAMERA_FLIP, config.CAMERA_MIRROR)
        self._clear()
        self.recent.clear()
        self.reflex.reset()
        self.driver.reset()
        self.robot.mode(3)

    def stop(self):
        # Leaving stage 3: stop the video. Otherwise it keeps filling the WiFi in stages 1 and 2,
        # and the Arduino's small sensor messages wait behind it (seconds of delay).
        self.close()

    def close(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None

    def on_message(self, msg):
        # The Arduino pings straight ahead in stage 3 too. The BRAIN never gets it (its only eye is
        # the camera); it is used only by the stuck reflex, like a touch sensor: "am I moving at all?"
        # (Camera motion alone was not reliable: blank white boards give the camera nothing to track.)
        if msg[0] == "P" and msg[1] == "C":
            self.recent.add("sonar", msg[2])

    def _stuck_now(self):
        sonar = self.recent.values("sonar", STUCK_S)
        if len(sonar) >= 6:
            near = [cm for cm in sonar if cm < 400]
            if not near:
                return True                               # no echo at all while driving: pressed on something
            return len(near) >= 4 and max(near) - min(near) <= MOVED_CM
        motion = self.recent.values("motion", STUCK_S)    # no sonar (old Arduino code): use the camera
        return len(motion) >= 5 and max(motion) < MOVING_PX_S

    def tick(self, swap):
        got = self.camera.latest()
        if got is None:                                   # no picture yet: stand still
            self.robot.wheels(0, 0)
            return {"wheels": (0, 0), "eyes": (0.0, 0.0), "hz": None, "spikes": None, "frame": None}
        rgb, gray, n, t = got
        if self.camera.age() > getattr(config, "CAM_STALE_S", 0.5):
            # The stream froze (WiFi hiccup): the fly is blind, so the car stops.
            # Forget the old picture, so the next one is not compared with a stale one
            # (that would look like a huge sudden looming).
            self._clear()
            self.recent.clear()
            spikes, hz, _ = self.driver.tick(0.0, 0.0)
            self.robot.wheels(0, 0)
            return {"wheels": (0, 0), "eyes": (0.0, 0.0), "hz": hz, "spikes": spikes, "frame": rgb}
        if n != self.last_n:                              # a new picture: measure looming and motion
            if self.prev is None:
                self.eyes, self.raw = (0.0, 0.0), {}
            else:
                eye_L, eye_R, self.raw = looming_from_frames(self.prev, gray, t - self.prev_t)
                # one shaky frame must not steer the car: use the middle of the last 3 measurements
                self.hist = (self.hist + [(eye_L, eye_R)])[-3:]
                self.eyes = tuple(sorted(h[i] for h in self.hist)[len(self.hist) // 2] for i in (0, 1))
                self.recent.add("motion", self.raw["motion"])
            self.prev, self.prev_t, self.last_n = gray, t, n
        eye_L, eye_R = self.eyes
        if swap:
            eye_L, eye_R = eye_R, eye_L
        spikes, hz, brain_wheels = self.driver.tick(eye_L, eye_R)
        # escape toward the side that looms less (the more open side)
        wheels, reflex = self.reflex.wheels(brain_wheels, self.eyes[1] <= self.eyes[0], self._stuck_now())
        self.robot.wheels(*wheels)
        return {"wheels": wheels, "eyes": self.driver.eyes_in, "hz": hz, "spikes": spikes, "frame": rgb,
                "reflex": reflex, "raw": self.raw}
