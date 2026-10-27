"""A simulated car in a room: same messages as the real Arduino (car.ino), for testing without hardware."""
import math
import time
import numpy as np
import config
from common.robot_link import DEFAULT_ANGLES

ROOM_W, ROOM_H = 300.0, 200.0
OBSTACLES = [(60, 60, 95, 95), (150, 0, 165, 70), (200, 120, 240, 155), (120, 140, 150, 200), (235, 40, 260, 65)]
WALLS = [(-10, -10, ROOM_W + 10, 0), (-10, ROOM_H, ROOM_W + 10, ROOM_H + 10),
         (-10, 0, 0, ROOM_H), (ROOM_W, 0, ROOM_W + 10, ROOM_H)]
BOXES = OBSTACLES + WALLS
INVISIBLE = []      # obstacles the car bumps into but the sonar misses (round bottle, angled brick). For tests.
RADIUS, TRACK, VMAX, SENSOR_OFFSET = 11.0, 14.0, 40.0, 10.0      # VMAX: cm/s at MOTOR_MAX_V
SETTLE_MS, WATCHDOG_MS = 110, 300
# motors, same as car.ino: battery-scaled power + soft start
BATTERY_V, MOTOR_MAX_V, MOTOR_MIN_V, DRIVER_DROP_V = 12.0, 6.0, 1.8, 2.0
MOTOR_SUPPLY_V = BATTERY_V - DRIVER_DROP_V
MAX_PWM = min(255, int(255 * MOTOR_MAX_V / MOTOR_SUPPLY_V))
MIN_PWM = int(255 * MOTOR_MIN_V / MOTOR_SUPPLY_V)
RAMP_PER_MS = 2
STALL_V = 1.5                      # below this the real motors don't turn
STOP_CM, SIDE_CM, FAST, SLOW, TURN, BACK_MS, SPIN_MS = 25, 20, 200, 90, 180, 400, 350
VEER_CM = 40
PATH_CM = 28          # a 35-degree echo closer than this is in front of a wheel
DEAD35 = False        # tuning: dead end judged from the 35-degree beams only
# The radar sweep (car.ino): 5 directions, 35-degree steps, servo travel time 30 ms + 2.2 ms/degree.
# RADAR = False simulates the old 3-direction scan (L C R C at +-45 degrees, 110 ms) for comparison.
RADAR = True
SWEEP = ["C", "L", "LL", "L", "C", "R", "RR", "R"]
OLD_SWEEP, OLD_ANGLES = ["L", "C", "R", "C"], {"L": 45, "C": 0, "R": -45}
DIRS5 = ("LL", "L", "C", "R", "RR")
STUCK_CM, STUCK_MS, ESCAPE_BACK_MS = 4, 1500, 700
DEADEND_CM, CLOSE_CM, MEMORY_MS, BIG_BACK_MS, BIG_SPIN = 35, 10, 3000, 800, 3
FRESH_DEADEND = False
HOLD_NEAR_CM, HOLD_MS, MISS_NEAR_CM = 60, 2500, 150     # missed-echo filter (same as car.ino)
TOO_CLOSE_CM = 3                                       # closer than this the real sensor hears no echo


def ray_box(ox, oy, dx, dy, box):
    x0, y0, x1, y1 = box
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0):
        return None
    return max(tmin, 0.0)


def push_out(x, y):
    hit = False
    for _ in range(3):
        for x0, y0, x1, y1 in BOXES + INVISIBLE:
            cx, cy = min(max(x, x0), x1), min(max(y, y0), y1)
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            if 1e-6 < d < RADIUS:
                hit = True
                x, y = cx + dx / d * RADIUS, cy + dy / d * RADIUS
    return x, y, hit


class FakeRobot:
    port = "simulation (no robot)"

    def __init__(self, seed=1):
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.h = 40.0, 30.0, math.radians(30)
        self.t0 = time.monotonic()
        self.ms = 0
        self.inbox, self.out = [], [("READY",), ("MODE", 0)]
        self.ready = True
        self.mode_n, self.L, self.R, self.last_cmd = 0, 0, 0, 0
        self.outL = self.outR = 0                       # motor PWM after the soft-start ramp
        self.scanning, self.scan_idx, self.servo_at, self.servo = False, 0, 0, "C"
        self.dist = {"L": 400, "C": 400, "R": 400}
        self.fresh_c, self.s1, self.s1_until, self.spin_right, self.last_w = False, 0, 0, True, 0
        self.bumps = 0
        self.held = {d: False for d in DIRS5}
        self.held_since = {d: 0 for d in DIRS5}
        self.seen_at, self.escape_at = {d: 0 for d in DIRS5}, 0
        self.servo_deg, self.settle = 0, SETTLE_MS
        self.stuck_ref, self.stuck_since = -1, 0
        self.trap_count, self.last_escape, self.last_turn_right, self.same_trap, self.big = 0, None, True, False, False

    # ---- same interface as RobotLink ----
    def send(self, text):
        self.inbox.append(text.strip())

    def mode(self, n):
        self.send(f"MODE,{int(n)}")

    def wheels(self, left, right):
        self.send(f"M,{int(left)},{int(right)}")

    def look(self, direction):
        self.send(f"LOOK,{direction}")

    def close(self):
        pass

    def poll(self):
        target = int((time.monotonic() - self.t0) * 1000)
        target = min(target, self.ms + 1000)            # after a long pause, don't simulate more than 1 s
        while self.ms < target:
            self._loop()
            self._physics(0.005)
            self.ms += 5
        out, self.out = self.out, []
        return out

    # ---- the Arduino sketch, in Python ----
    def _drive(self, left, right):
        self.L, self.R = int(np.clip(left, -255, 255)), int(np.clip(right, -255, 255))

    def _set_mode(self, m):
        self.mode_n = m
        self._drive(0, 0)
        self.s1 = 0
        self.stuck_ref, self.stuck_since = -1, self.ms
        self.trap_count, self.last_escape = 0, None
        self.scanning = m in (1, 2, 3)            # stage 3: straight ahead only
        self.scan_idx = 0
        self._aim(self._sweep()[0] if m in (1, 2) else "C")
        self.held = {d: False for d in DIRS5}
        self.held_since = {d: 0 for d in DIRS5}
        self.dist = {d: 400 for d in DIRS5}
        self.servo_at = self.last_cmd = self.ms
        self.out.append(("MODE", m))

    @staticmethod
    def _sweep():
        return SWEEP if RADAR else OLD_SWEEP

    @staticmethod
    def _angle(d):
        return DEFAULT_ANGLES[d] if RADAR else OLD_ANGLES[d]

    def _aim(self, d):
        deg = self._angle(d)
        self.settle = 30 + 2.2 * abs(deg - self.servo_deg) if RADAR else SETTLE_MS
        self.servo, self.servo_deg, self.servo_at = d, deg, self.ms

    def _loop(self):
        for text in self.inbox:
            p = text.split(",")
            if p[0] == "MODE" and len(p) == 2 and p[1] in "0123":
                self._set_mode(int(p[1]))
            elif p[0] == "LOOK" and len(p) == 2 and self.mode_n == 0 and p[1] in DIRS5:
                self._aim(p[1])
            elif p[0] == "M" and len(p) == 3 and self.mode_n in (2, 3):
                try:
                    self._drive(int(p[1]), int(p[2]))
                    self.last_cmd = self.ms
                except ValueError:
                    pass
        self.inbox = []
        if self.scanning and self.ms - self.servo_at >= self.settle:
            d = "C" if self.mode_n == 3 else self._sweep()[self.scan_idx]
            cm = self._hold(d, self.sonar(d))
            self.dist[d] = cm
            self.seen_at[d] = self.ms
            if d == "C":
                self.fresh_c = True
                if cm >= 400:
                    self.stuck_ref, self.stuck_since = -1, self.ms
                elif self.stuck_ref < 0 or abs(cm - self.stuck_ref) > STUCK_CM:
                    self.stuck_ref, self.stuck_since = cm, self.ms
            self.out.append(("P", d, cm, self.ms))
            if self.mode_n != 3:
                self.scan_idx = (self.scan_idx + 1) % len(self._sweep())
                self._aim(self._sweep()[self.scan_idx])
            else:
                self.servo_at, self.settle = self.ms, 100
        if self.mode_n == 1:
            self._stage1()
            if self.ms - self.last_w >= 200:
                self.last_w = self.ms
                self.out.append(("W", self.L, self.R))
        if self.mode_n in (2, 3) and self.ms - self.last_cmd > WATCHDOG_MS:
            self._drive(0, 0)

    def _hold(self, d, cm):
        """Same missed-echo filter as scanStep() in car.ino."""
        prev, hold = self.dist[d], False
        if cm >= 400:
            if prev < HOLD_NEAR_CM:
                hold = not self.held[d] or self.ms - self.held_since[d] < HOLD_MS
            elif prev < MISS_NEAR_CM:
                hold = not self.held[d]
        if hold:
            if not self.held[d]:
                self.held_since[d] = self.ms
            self.held[d] = True
            return prev
        if cm < 400:
            self.held[d] = False
        else:
            self.held[d], self.held_since[d] = True, -10 ** 9
        return cm

    def _escape(self, back_ms):
        self.same_trap = self.last_escape is not None and self.ms - self.last_escape < MEMORY_MS
        self.trap_count = self.trap_count + 1 if self.same_trap else 1
        self.last_escape = self.escape_at = self.ms
        self.held = {d: True for d in DIRS5}           # backing up + turning: believe 400s again
        self.held_since = {d: -10 ** 9 for d in DIRS5}
        self.big = self.trap_count >= 3
        if self.big:
            self.out.append(("TRAP", "repeat"))
            self.trap_count = 0
        self.s1, self.s1_until = 1, self.ms + (max(back_ms, BIG_BACK_MS) if self.big else back_ms)
        self._drive(-TURN, -TURN)

    # stage 1 rules, same as stage1() in car.ino
    def _near(self, a, b):
        return min(self.dist[a], self.dist[b]) if RADAR else self.dist[a]

    def _space(self, a, b):
        return min(self.dist[a], 200) + min(self.dist[b], 200) if RADAR else self.dist[a]

    def _ahead(self):
        a = self.dist["C"]
        if RADAR:
            for s in ("L", "R"):
                if self.dist[s] < PATH_CM:
                    a = min(a, self.dist[s] * 82 // 100)
        return a

    def _stage1(self):
        if self.s1 == 1:
            self._drive(-TURN, -TURN)
            if self.ms >= self.s1_until:
                nl = self.dist["L"] if DEAD35 else self._near("L", "LL")
                nr = self.dist["R"] if DEAD35 else self._near("R", "RR")
                if not self.big and nl < DEADEND_CM and nr < DEADEND_CM:
                    self.out.append(("TRAP", "deadend"))
                    self.big = True
                self.spin_right = (self.last_turn_right if self.same_trap
                                   else self._space("R", "RR") > self._space("L", "LL"))
                self.last_turn_right = self.spin_right
                times = BIG_SPIN if self.big else (2 if self.same_trap else 1)
                self.s1, self.s1_until = 2, self.ms + SPIN_MS * times
            return
        if self.s1 == 2:
            self._drive(TURN, -TURN) if self.spin_right else self._drive(-TURN, TURN)
            if self.ms >= self.s1_until:
                self.s1, self.fresh_c = 3, False
                self._drive(0, 0)
                if RADAR:
                    self.scan_idx = 0
                    self._aim("C")
            return
        if self.s1 == 3:
            self._drive(0, 0)
            if self.fresh_c:
                self.s1 = 0
                self.stuck_ref, self.stuck_since = -1, self.ms
            return
        if self.stuck_ref >= 0 and self.ms - self.stuck_since > STUCK_MS:
            self.out.append(("STUCK",))
            self.stuck_ref, self.stuck_since = -1, self.ms
            self._escape(ESCAPE_BACK_MS)
            return
        d = self.dist
        if self._ahead() < STOP_CM:
            self._escape(BACK_MS)
            return
        if RADAR:
            left = d["L"] < VEER_CM or d["LL"] < SIDE_CM
            right = d["R"] < VEER_CM or d["RR"] < SIDE_CM
            if left and right:
                diff = self._space("L", "LL") - self._space("R", "RR")
                left, right = diff < -10, diff > 10
        else:
            left, right = d["L"] < SIDE_CM, d["R"] < SIDE_CM
        if left:
            self._drive(FAST, -SLOW if self._near("L", "LL") < CLOSE_CM else SLOW)
        elif right:
            self._drive(-SLOW if self._near("R", "RR") < CLOSE_CM else SLOW, FAST)
        else:
            self._drive(FAST, FAST)

    # ---- the room ----
    def sonar(self, direction):
        sx = self.x + SENSOR_OFFSET * math.cos(self.h)
        sy = self.y + SENSOR_OFFSET * math.sin(self.h)
        best = 400.0
        for spread in (-7.5, 0.0, 7.5):
            a = self.h + math.radians(self._angle(direction) + spread)
            for b in BOXES:
                t = ray_box(sx, sy, math.cos(a), math.sin(a), b)
                if t is not None:
                    best = min(best, t)
        cm = best + self.rng.normal(0, 0.5)
        return 400 if cm < TOO_CLOSE_CM else int(np.clip(cm, 2, 400))

    def _physics(self, dt):
        def lift(cmd):                     # same as lift() in car.ino
            a = abs(cmd)
            return 0 if a < 12 else (1 if cmd > 0 else -1) * (MIN_PWM + a * (MAX_PWM - MIN_PWM) // 255)

        def step_toward(cur, target, step):   # same as stepToward() in car.ino
            if cur * target < 0:
                target = 0
            return min(cur + step, target) if target > cur else max(cur - step, target)

        step = int(RAMP_PER_MS * dt * 1000)
        self.outL = step_toward(self.outL, lift(self.L), step)
        self.outR = step_toward(self.outR, lift(self.R), step)

        def speed(pwm):
            volts = pwm / 255.0 * MOTOR_SUPPLY_V
            return 0.0 if abs(volts) < STALL_V else VMAX * volts / MOTOR_MAX_V
        vl, vr = speed(self.outL), speed(self.outR)
        self.h += (vr - vl) / TRACK * dt
        v = (vl + vr) / 2
        x, y, hit = push_out(self.x + v * math.cos(self.h) * dt, self.y + v * math.sin(self.h) * dt)
        if hit:          # touching something: the real car's corner catches, so it does not slide along it
            self.bumps += 1
            self.stuck_ms = getattr(self, "stuck_ms", 0) + dt * 1000
        else:
            self.x, self.y = x, y

    def snapshot(self):
        return {"x": self.x, "y": self.y, "h": self.h, "servo": self.servo}
