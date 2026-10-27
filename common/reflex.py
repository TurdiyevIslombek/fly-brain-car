"""The stuck reflex (stages 2 and 3). This is NOT the fly brain: it is a hand-written safety rule
on top of it, like the stage 1 rules on the Arduino, so all 3 stages get the same help.

If the car has been trying to drive for STUCK_S seconds but its sensor shows no movement
(a corner caught on something, or pushing against something the eyes missed),
it backs up, turns, and gives control back to the brain.

Short-term memory (no map: the car can't measure where it is, only what happened recently):
  - escapes less than MEMORY_S apart are "the same trap": keep turning the SAME way
    (turning left, then right, then left is how robots stay stuck in corners) and turn further;
  - the 3rd escape in the same trap is a big one: back up longer, turn about half a circle.
Every tick the reflex drives is logged as reflex=1 in the CSV."""
import time
import config

STUCK_S = getattr(config, "STUCK_S", 1.5)          # no movement for this long while driving = stuck
BACK_S = getattr(config, "ESCAPE_BACK_S", 0.7)      # then back up this long ...
TURN_S = getattr(config, "ESCAPE_TURN_S", 0.45)     # ... and turn this long (x2 same trap, x3 big)
MEMORY_S = getattr(config, "ESCAPE_MEMORY_S", 8.0)  # escapes closer together than this = same trap
ESCAPE_SPEED = 180
TRYING = 60                                         # average |wheel| above this = trying to move


class StuckReflex:
    def __init__(self):
        self.fired = 0
        self.reset()

    def reset(self):
        self.trying_since = None
        self.back_until = self.turn_until = 0.0
        self.turn_right = True
        self.last_escape, self.trap_count, self.big = 0.0, 0, False

    def active(self):
        return time.time() < self.turn_until

    def wheels(self, brain_wheels, right_is_open, stuck_now):
        """brain_wheels: what the brain wants. right_is_open: which side looks freer.
        stuck_now: the stage's sensor says "no movement during the last STUCK_S seconds".
        Returns (wheels to send, True if the reflex is driving)."""
        now = time.time()
        if now < self.back_until:
            return (-ESCAPE_SPEED, -ESCAPE_SPEED), True
        if now < self.turn_until:
            s = ESCAPE_SPEED
            return ((s, -s) if self.turn_right else (-s, s)), True
        trying = (abs(brain_wheels[0]) + abs(brain_wheels[1])) / 2 > TRYING
        if not trying:
            self.trying_since = None
        elif self.trying_since is None:
            self.trying_since = now
        if trying and stuck_now and now - self.trying_since >= STUCK_S:
            self._escape(now, right_is_open)
            return (-ESCAPE_SPEED, -ESCAPE_SPEED), True
        return brain_wheels, False

    def _escape(self, now, right_is_open):
        same = self.last_escape and now - self.last_escape < MEMORY_S
        self.trap_count = self.trap_count + 1 if same else 1
        self.big = self.trap_count >= 3
        if self.big:
            self.trap_count = 0
        if not same:
            self.turn_right = right_is_open        # new trap: turn toward the open side; same trap: keep going
        self.last_escape = now
        self.fired += 1
        turns = 3 if self.big else (2 if same else 1)
        self.back_until = now + BACK_S * (1.4 if self.big else 1.0)
        self.turn_until = self.back_until + TURN_S * turns
        self.trying_since = None


class Window:
    """Recent sensor values with their times, to ask "did anything change in the last N seconds?"."""
    def __init__(self, seconds):
        self.seconds, self.items = seconds, []

    def add(self, key, value, t=None):
        t = time.time() if t is None else t
        self.items.append((t, key, value))
        self.items = [x for x in self.items if t - x[0] <= self.seconds + 0.5]

    def values(self, key, seconds):
        now = time.time()
        return [v for t, k, v in self.items if k == key and now - t <= seconds]

    def clear(self):
        self.items = []
