"""The fly brain (2,282 real neurons) turned into wheel speeds. Used by stage 2 and stage 3."""
import sys
from collections import deque
from pathlib import Path
import numpy as np
import config

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "brain"))
from flybrain import FlyBrain  # noqa: E402

OUT = ["DNa01_L", "DNa01_R", "DNa02_L", "DNa02_R", "MDN_L", "MDN_R"]


def looming_hz(cm, prev_cm, dt_s):
    """How fast an obstacle grows in view -> eye firing rate (Hz). Only approaching counts."""
    size = 2 * np.arctan(config.OBJ_CM / 2 / max(cm, 1.0))
    prev = 2 * np.arctan(config.OBJ_CM / 2 / max(prev_cm, 1.0))
    grow = max(0.0, (size - prev) / max(dt_s, 1e-3))
    # closeness adds smoothly: nothing beyond 1.25 x TOUCH_CM, full strength at 0.5 x TOUCH_CM
    # (before: a hard switch at TOUCH_CM, which made the eyes jump 0 -> 250 and the car stop-go)
    near = (1.25 * config.TOUCH_CM - cm) / (0.75 * config.TOUCH_CM)
    x = grow / config.LOOM_SAT + float(np.clip(near, 0, 1))
    return config.MAX_HZ * float(np.clip(x, 0, 1))


class BrainDriver:
    def __init__(self):
        self.brain = FlyBrain(ROOT / "brain" / "locomotion", seed=config.SEED)
        self.member = {k: np.isin(np.arange(self.brain.N), self.brain.groups[k]) for k in OUT}
        self.size = {k: len(self.brain.groups[k]) for k in OUT}
        self.history = deque()
        self.step_ms = None      # app.py sets this to the real time since the last tick
        self.eyes_in = (0.0, 0.0)        # what the eye neurons really received (after smoothing)
        self.wheels_s = (0.0, 0.0)       # smoothed wheel speeds

    def reset(self):
        self.brain.reset()
        self.history.clear()
        self.eyes_in = (0.0, 0.0)
        self.wheels_s = (0.0, 0.0)

    @staticmethod
    def _follow(old, new, dt_ms, tau_ms):
        return old + (new - old) * (1.0 - np.exp(-dt_ms / tau_ms))

    def _smooth_eyes(self, eye_L, eye_R, ms):
        """Eyes rise fast (a threat must count at once) and fade slowly (one noisy frame or a sonar ping
        that missed must not make the car forget the obstacle). Real fly eye neurons also integrate
        over ~0.1-0.3 s; here it is an engineering choice: EYE_RISE_MS / EYE_FADE_MS in config.py."""
        rise, fade = getattr(config, "EYE_RISE_MS", 40.0), getattr(config, "EYE_FADE_MS", 200.0)
        out = []
        for old, new in zip(self.eyes_in, (eye_L, eye_R)):
            out.append(self._follow(old, new, ms, rise if new > old else fade))
        self.eyes_in = tuple(out)
        return self.eyes_in

    def tick(self, eye_L, eye_R, ms=None):
        """Run the brain for `ms` milliseconds. Returns (spikes, hz, (left wheel, right wheel))."""
        ms = ms or self.step_ms or config.TICK_MS
        eye_L, eye_R = self._smooth_eyes(eye_L, eye_R, ms)
        stim = {"loom_L": eye_L, "loom_R": eye_R}
        counts = {k: 0 for k in OUT}
        fired = []
        for _ in range(int(ms / self.brain.dt)):
            s = self.brain.step(stim)
            if s.size:
                fired.append(s)
                for k in OUT:
                    counts[k] += int(self.member[k][s].sum())
        self.history.append((ms, counts))
        while sum(h[0] for h in self.history) > config.WINDOW_MS:
            self.history.popleft()
        win = sum(h[0] for h in self.history) / 1000.0
        hz = {k: sum(h[1][k] for h in self.history) / self.size[k] / win for k in OUT}
        turn = (hz["DNa01_R"] + hz["DNa02_R"]) - (hz["DNa01_L"] + hz["DNa02_L"])     # > 0 = turn right
        back = (hz["MDN_L"] + hz["MDN_R"]) / 2
        forward = config.BASE - config.K_BACK * back
        # backward is blind (the sensor looks forward): never reverse faster than REVERSE_MAX
        rev = getattr(config, "REVERSE_MAX", 150)
        target = (float(np.clip(forward + config.K_TURN * turn, -rev, 255)),
                  float(np.clip(forward - config.K_TURN * turn, -rev, 255)))
        # wheels follow the brain over ~WHEEL_SMOOTH_MS instead of jumping every tick (motors have
        # inertia anyway; sudden jumps only make the car shake)
        tau = getattr(config, "WHEEL_SMOOTH_MS", 120.0)
        self.wheels_s = tuple(self._follow(o, n, ms, tau) for o, n in zip(self.wheels_s, target))
        left, right = (int(round(w)) for w in self.wheels_s)
        spikes = np.concatenate(fired) if fired else np.array([], dtype=int)
        return spikes, hz, (left, right)
