"""Closed-loop control test in a simulated copy of the real arena (arena_sizes.png: 200 x 170 cm,
4 corner pieces, wedge on the white wall, middle board with bricks).

Same code as the real car: common/fake_robot.py (a Python copy of car.ino: radar sweep, motors,
soft start) + stage2/stage2.py (sonar -> eye neurons) + common/brain_driver.py + common/reflex.py.
Only the room geometry and a simulated clock are added here.

Conditions (same arena, same start positions, same reflex):
  fly        real FlyWire wiring (what the car runs)
  shuffled   same synapses, targets shuffled (a different random wiring for every run)
  nobrain    no brain: drive straight at BASE; the stuck reflex still works
  stage1     the stage 1 if-else rules on the Arduino (for reference)
  swapped    real wiring, but left and right eyes swapped

usage: python3 arena_test.py CONDITION SEED [SECONDS]  -> prints one JSON line
"""
import json
import math
import sys
import numpy as np
from pathlib import Path

SIM = {"t": 1_000_000.0}
import time  # noqa: E402
time.time = lambda: SIM["t"]
time.monotonic = lambda: SIM["t"]

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "brain"))
import config  # noqa: E402
from common import fake_robot as fr  # noqa: E402

# ---- motors exactly as on the car now (car.ino BATTERY_V 8.0) ----
fr.BATTERY_V = 8.0
fr.MOTOR_SUPPLY_V = fr.BATTERY_V - fr.DRIVER_DROP_V
fr.MAX_PWM = min(255, int(255 * fr.MOTOR_MAX_V / fr.MOTOR_SUPPLY_V))
fr.MIN_PWM = int(255 * fr.MOTOR_MIN_V / fr.MOTOR_SUPPLY_V)

# ---- the real arena as line segments (cm). x along board A (0..200), y from A toward C (0..170) ----
W_, H_ = 200.0, 170.0
POLY = [(35, 0), (165, 0), (200, 35), (200, 55), (172, 85), (200, 115), (200, 135), (165, 170),
        (35, 170), (0, 135), (0, 35)]                       # inside outline: corners D, wedge E
F_BOX = (55, 105, 95, 118)                                   # middle board F + bricks behind
SEGS = [(POLY[i], POLY[(i + 1) % len(POLY)]) for i in range(len(POLY))]
fx0, fy0, fx1, fy1 = F_BOX
SEGS += [((fx0, fy0), (fx1, fy0)), ((fx1, fy0), (fx1, fy1)), ((fx1, fy1), (fx0, fy1)), ((fx0, fy1), (fx0, fy0))]
SEG = np.array([[a[0], a[1], b[0], b[1]] for a, b in SEGS], float)


def ray_dist(ox, oy, dx, dy):
    best = 1e9
    for x0, y0, x1, y1 in SEG:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x0 - ox) * ey - (y0 - oy) * ex) / den
        u = ((x0 - ox) * dy - (y0 - oy) * dx) / den
        if t >= 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def seg_push(x, y):
    hit = False
    for _ in range(3):
        for x0, y0, x1, y1 in SEG:
            ex, ey = x1 - x0, y1 - y0
            u = min(1, max(0, ((x - x0) * ex + (y - y0) * ey) / (ex * ex + ey * ey)))
            cx, cy = x0 + u * ex, y0 + u * ey
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            if 1e-6 < d < fr.RADIUS:
                hit = True
                x, y = cx + dx / d * fr.RADIUS, cy + dy / d * fr.RADIUS
    return x, y, hit


fr.push_out = seg_push


def inside(x, y):
    from matplotlib.path import Path as MPath
    if not MPath(POLY).contains_point((x, y)):
        return False
    return not (fx0 <= x <= fx1 and fy0 <= y <= fy1)


def clearance(x, y):
    best = 1e9
    for x0, y0, x1, y1 in SEG:
        ex, ey = x1 - x0, y1 - y0
        u = min(1, max(0, ((x - x0) * ex + (y - y0) * ey) / (ex * ex + ey * ey)))
        best = min(best, math.hypot(x - x0 - u * ex, y - y0 - u * ey))
    return best


CELL = 10.0
FREE = {(i, j) for i in range(20) for j in range(17)
        if inside((i + .5) * CELL, (j + .5) * CELL) and clearance((i + .5) * CELL, (j + .5) * CELL) >= fr.RADIUS}


class ArenaRobot(fr.FakeRobot):
    def sonar(self, direction):
        sx = self.x + fr.SENSOR_OFFSET * math.cos(self.h)
        sy = self.y + fr.SENSOR_OFFSET * math.sin(self.h)
        best = 400.0
        for spread in (-7.5, 0.0, 7.5):
            a = self.h + math.radians(self._angle(direction) + spread)
            best = min(best, ray_dist(sx, sy, math.cos(a), math.sin(a)))
        cm = best + self.rng.normal(0, 0.5)
        return 400 if cm < fr.TOO_CLOSE_CM else int(np.clip(cm, 2, 400))


class NoBrain:
    """No neurons: always drive straight ahead at BASE (what the brain does when it sees nothing)."""
    def __init__(self):
        self.eyes_in, self.step_ms = (0.0, 0.0), None

    def reset(self):
        self.eyes_in = (0.0, 0.0)

    def tick(self, eye_L, eye_R, ms=None):
        self.eyes_in = (eye_L, eye_R)
        return np.array([], int), {}, (config.BASE, config.BASE)


def start_pose(seed):
    rng = np.random.default_rng(seed)
    while True:
        x, y = rng.uniform(15, 185), rng.uniform(15, 155)
        if inside(x, y) and clearance(x, y) > 30:
            return x, y, rng.uniform(-math.pi, math.pi)


def run(cond, seed, seconds=60.0):
    robot = ArenaRobot(seed=seed)
    robot.x, robot.y, robot.h = start_pose(seed)
    if cond == "stage1":
        from stage1.stage1 import Stage1
        stage = Stage1(robot)
    else:
        from stage2.stage2 import Stage2
        if cond == "nobrain":
            driver = NoBrain()
        else:
            from common.brain_driver import BrainDriver
            import wiring
            config.SEED = seed
            driver = BrainDriver()
            driver.brain.rng = np.random.default_rng(seed)
            if cond == "shuffled":
                wiring.install(driver, wiring.shuffled(seed))
        driver.step_ms = config.TICK_MS
        stage = Stage2(robot, driver)
    stage.start()
    swap = cond == "swapped"
    ticks = int(seconds * 1000 / config.TICK_MS)
    visited, reflex_ticks, dist, path = set(), 0, 0.0, []
    bumps0 = robot.bumps
    for k in range(ticks):
        for _ in range(config.TICK_MS // 5):
            robot._loop()
            px, py = robot.x, robot.y
            robot._physics(0.005)
            dist += math.hypot(robot.x - px, robot.y - py)
            robot.ms += 5
            SIM["t"] += 0.005
        msgs, robot.out = robot.out, []
        for m in msgs:
            stage.on_message(m)
        info = stage.tick(swap)
        reflex_ticks += bool(info.get("reflex"))
        c = (int(robot.x // CELL), int(robot.y // CELL))
        if c in FREE:
            visited.add(c)
        if k % 4 == 0:
            path.append((round(robot.x, 1), round(robot.y, 1)))
    return {"cond": cond, "seed": seed, "seconds": seconds,
            "contact_s": round((robot.bumps - bumps0) * 0.005, 2),
            "reflex_pct": round(100 * reflex_ticks / ticks, 1),
            "explored_pct": round(100 * len(visited) / len(FREE), 1),
            "dist_m": round(dist / 100, 2), "path": path}


if __name__ == "__main__":
    cond, seed = sys.argv[1], int(sys.argv[2])
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    t0 = time.perf_counter() if hasattr(time, "perf_counter") else 0
    r = run(cond, seed, secs)
    print(json.dumps(r), flush=True)
