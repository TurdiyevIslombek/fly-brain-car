"""The window: stage buttons on top, what the robot senses on the left, the fly brain on the right."""
import csv
import math
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.patches import Rectangle, Polygon
import config
from common.robot_link import DIRS, scan_angles

ROOT = Path(__file__).resolve().parent.parent
ROLE_COLORS = {"eye_L": (0.15, 0.6, 1.0), "eye_R": (0.15, 0.6, 1.0), "steer_L": (0.2, 0.85, 0.35),
               "steer_R": (0.2, 0.85, 0.35), "back": (1.0, 0.25, 0.25), "escape": (1.0, 0.85, 0.1),
               "forward": (0.7, 0.5, 1.0), "other": (1.0, 0.55, 0.1)}
BIG = ("steer_L", "steer_R", "back", "escape", "forward")
GREY = np.array([0.22, 0.24, 0.26])
GREEN, RED, BLUE, DIM = "#4ade80", "#f87171", "#38bdf8", "#555b63"
FADE = 0.6
MAX_BEAM = 150.0
RADAR_S = 2.5          # radar dots stay this long, fading (what the sensor saw in the last 2.5 s)
STAGE_COLORS = {1: "#a16207", 2: "#15803d", 3: "#1d4ed8"}

for k in ("keymap.save", "keymap.fullscreen", "keymap.home", "keymap.back", "keymap.forward", "keymap.pan",
          "keymap.zoom", "keymap.grid", "keymap.grid_minor", "keymap.yscale", "keymap.xscale"):
    if k in plt.rcParams:
        plt.rcParams[k] = []
plt.rcParams["toolbar"] = "None"


def load_positions():
    with open(ROOT / "brain" / "locomotion" / "positions.csv") as f:
        rows = list(csv.DictReader(f))
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    roles = np.array([r["role"] for r in rows])
    return x, y, roles


class View:
    def __init__(self, on_stage, on_stop, on_swap, fake=False):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(14, 8.4))
        try:
            self.fig.canvas.manager.set_window_title("Fly-brain car")
        except Exception:
            pass
        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self.on_stage, self.on_stop, self.on_swap = on_stage, on_stop, on_swap

        # ---- buttons ----
        labels = {1: "1   STAGE 1\nif-else (Arduino only)", 2: "2   STAGE 2\nfly brain + sonar",
                  3: "3   STAGE 3\nfly brain + camera"}
        self.buttons = {}
        for i, n in enumerate((1, 2, 3)):
            ax = self.fig.add_axes([0.01 + i * 0.165, 0.915, 0.155, 0.07])
            b = Button(ax, labels[n], color="#1f2937", hovercolor="#374151")
            b.label.set_fontsize(11)
            b.on_clicked(lambda _e, n=n: self.on_stage(n))
            self.buttons[n] = b
        ax = self.fig.add_axes([0.505, 0.915, 0.11, 0.07])
        self.stop_button = Button(ax, "STOP\n(space)", color="#7f1d1d", hovercolor="#991b1b")
        self.stop_button.label.set_fontsize(11)
        self.stop_button.on_clicked(lambda _e: self.on_stop())
        ax = self.fig.add_axes([0.625, 0.915, 0.15, 0.07])
        self.swap_button = Button(ax, "swap eyes: OFF\n(s) control test", color="#1f2937", hovercolor="#374151")
        self.swap_button.label.set_fontsize(10)
        self.swap_button.on_clicked(lambda _e: self.on_swap())
        self.title = self.fig.text(0.01, 0.875, "STOPPED", fontsize=13, fontweight="bold", va="center")
        self.status = self.fig.text(0.99, 0.875, "", fontsize=9, va="center", ha="right", color="#9ca3af")

        # ---- left: sonar view ----
        a = self.ax_sonar = self.fig.add_axes([0.02, 0.08, 0.40, 0.75])
        a.set_xlim(-160, 160)
        a.set_ylim(-60, 175)
        a.set_aspect("equal")
        a.axis("off")
        a.set_title("what the robot senses (seen from above)   ·  radar: last 2.5 s", fontsize=11)
        a.add_patch(Rectangle((-14, -20), 28, 34, color="#d97706"))
        a.add_patch(Rectangle((-5, 12), 10, 6, color="#e5e7eb"))
        for r in (50, 100, 150):                                   # radar range rings (cm)
            t = np.linspace(-math.pi * 0.55, math.pi * 0.55, 60)
            a.plot(-np.sin(t) * r, 16 + np.cos(t) * r, color="#1f2937", lw=1, zorder=0)
            a.text(3, 16 + r + 2, f"{r} cm", color="#4b5563", fontsize=8)
        self.angles = scan_angles()
        self.beams, self.beam_txt = {}, {}
        for d in DIRS:
            self.beams[d] = a.plot([0, 0], [16, 16], color=BLUE, lw=3, solid_capstyle="round")[0]
            self.beam_txt[d] = a.text(0, 0, "", ha="center", fontsize=10, color=BLUE)
        self.hits, self.prev_dir = [], None                        # radar memory: (x, y, time)
        self.radar = a.scatter([], [], s=40, color=GREEN, zorder=5)
        self.head = a.plot([0, 0], [16, 30], color="white", lw=2)[0]
        self.arrows = a.quiver([-24, 24], [-2, -2], [0, 0], [0, 0], color=[GREEN, GREEN], angles="xy",
                               scale_units="xy", scale=1, width=0.025)

        # ---- left: camera view (stage 3) ----
        c = self.ax_cam = self.fig.add_axes([0.02, 0.15, 0.40, 0.67])
        c.axis("off")
        c.set_title("what the robot sees (ESP32-CAM)", fontsize=11)
        self.img, self.halves = None, []
        self.cam_msg = c.text(0.5, 0.5, "waiting for the camera...", ha="center", fontsize=12,
                              transform=c.transAxes, color="#9ca3af")
        self.eye_txt = [c.text(0.25, -0.07, "", transform=c.transAxes, ha="center", fontsize=12, color=BLUE),
                        c.text(0.75, -0.07, "", transform=c.transAxes, ha="center", fontsize=12, color=BLUE)]
        c.set_visible(False)

        # ---- simulated room (only without a robot) ----
        self.ax_map = None
        if fake:
            from common import fake_robot as fr
            m = self.ax_map = self.fig.add_axes([0.02, 0.09, 0.12, 0.17])
            m.set_xlim(0, fr.ROOM_W)
            m.set_ylim(0, fr.ROOM_H)
            m.set_aspect("equal")
            m.set_xticks([])
            m.set_yticks([])
            m.set_title("simulated room", fontsize=8)
            for x0, y0, x1, y1 in fr.OBSTACLES:
                m.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="#6b7280"))
            self.car_marker = Polygon([[0, 0], [0, 0], [0, 0]], closed=True, color="#f59e0b")
            m.add_patch(self.car_marker)
            self.trail = m.plot([], [], color="#f59e0b", lw=0.8, alpha=0.6)[0]
            self.trail_xy = []

        # ---- right: the fly brain ----
        x, y, roles = load_positions()
        big = np.isin(roles, BIG)
        self.order = np.argsort(big, kind="stable")              # big neurons drawn last (on top)
        self.hot = np.array([ROLE_COLORS[r] for r in roles])[self.order]
        self.base_size = np.where(big, 45.0, 6.0)[self.order]
        self.glow = np.zeros(len(x))
        b = self.ax_brain = self.fig.add_axes([0.45, 0.30, 0.54, 0.52])
        self.sc = b.scatter(x[self.order], y[self.order], s=self.base_size, c=[GREY] * len(x), linewidths=0)
        b.invert_yaxis()
        b.set_aspect("equal")
        b.axis("off")
        self.brain_title = b.set_title("", fontsize=11)
        b.text(0.0, 0.98, "LEFT EYE", transform=b.transAxes, color=BLUE, fontsize=11, fontweight="bold", va="top")
        b.text(1.0, 0.98, "RIGHT EYE", transform=b.transAxes, color=BLUE, fontsize=11, fontweight="bold",
               va="top", ha="right")
        for i, (label, role) in enumerate([("eyes", "eye_L"), ("steering", "steer_L"), ("moonwalker", "back"),
                                           ("escape", "escape"), ("in between", "other")]):
            b.text(0.06 + i * 0.19, -0.04, "● " + label, transform=b.transAxes, color=ROLE_COLORS[role],
                   fontsize=9)
        self.n_neurons = len(x)

        # ---- bars ----
        names = ["eye L", "eye R", "steer L", "steer R", "moonwalker"]
        colors = [ROLE_COLORS["eye_L"]] * 2 + [ROLE_COLORS["steer_L"]] * 2 + [ROLE_COLORS["back"]]
        self.ax_bars = self.fig.add_axes([0.53, 0.07, 0.43, 0.17])
        self.bars = self.ax_bars.barh(names, [0] * 5, color=colors)
        self.ax_bars.invert_yaxis()
        self.ax_bars.set_xlim(0, 260)
        self.ax_bars.set_xlabel("spikes per second", fontsize=9)
        self.ax_bars.tick_params(labelsize=9)
        self.wheel_txt = self.fig.text(0.22, 0.035, "", ha="center", fontsize=14, fontweight="bold")

    # ---- input ----
    def _key(self, e):
        k = (e.key or "").lower()
        if k in ("1", "2", "3"):
            self.on_stage(int(k))
        elif k == " ":
            self.on_stop()
        elif k == "s":
            self.on_swap()

    # ---- drawing ----
    def set_stage(self, n, title, swap):
        for m, b in self.buttons.items():
            col = STAGE_COLORS[m] if m == n else "#1f2937"
            b.color = col
            b.ax.set_facecolor(col)
        self.title.set_text(title if n else "STOPPED")
        self.title.set_color("#fbbf24" if swap and n in (2, 3) else "white")
        self.ax_cam.set_visible(n == 3)
        self.ax_sonar.set_visible(n != 3)
        if self.ax_map is not None:
            self.ax_map.set_visible(n != 3)
        on = n in (2, 3)
        self.brain_title.set_text(f"fly brain: {self.n_neurons:,} simulated neurons, wired like the real fly (FlyWire connectome)"
                                  if on else "fly brain: OFF (stage 1 uses if-else rules on the Arduino)")
        self.brain_title.set_color("white" if on else DIM)

    def set_swap(self, swap):
        self.swap_button.label.set_text("swap eyes: ON\n(s) control test" if swap else
                                        "swap eyes: OFF\n(s) control test")
        c = "#92400e" if swap else "#1f2937"
        self.swap_button.color = c
        self.swap_button.ax.set_facecolor(c)

    def update(self, stage, info, dists, last_dir, status, snapshot=None):
        # sonar
        if stage != 3:
            now = time.time()
            if last_dir != self.prev_dir and dists.get(last_dir) is not None:   # a new ping: add a dot
                self.prev_dir = last_dir
                cm = dists[last_dir]
                if cm < 400:
                    a = math.radians(self.angles[last_dir])
                    self.hits.append((-math.sin(a) * min(cm, 170), 16 + math.cos(a) * min(cm, 170), now))
            self.hits = [h for h in self.hits if now - h[2] < RADAR_S]
            if self.hits:
                self.radar.set_offsets([(h[0], h[1]) for h in self.hits])
                age = np.array([(now - h[2]) / RADAR_S for h in self.hits])
                self.radar.set_facecolors([(0.29, 0.87, 0.5, 1 - 0.85 * x) for x in age])
            else:
                self.radar.set_offsets(np.empty((0, 2)))
            for d in DIRS:
                cm = dists.get(d)
                a = math.radians(self.angles[d])
                length = min(cm if cm is not None else 0, MAX_BEAM)
                x, y = -math.sin(a) * length, 16 + math.cos(a) * length
                self.beams[d].set_data([0, x], [16, y])
                self.beams[d].set_alpha(1.0 if d == last_dir else 0.45)
                self.beam_txt[d].set_position((x * 1.1 - math.sin(a) * 6, y + 6))
                self.beam_txt[d].set_text("" if cm is None else (f"{cm} cm" if cm < 400 else "far"))
            a = math.radians(self.angles.get(last_dir or "C", 0))
            self.head.set_data([0, -math.sin(a) * 16], [16, 16 + math.cos(a) * 16])
        L, R = info["wheels"]
        self.arrows.set_UVC([0, 0], [L / 255 * 45, R / 255 * 45])
        self.arrows.set_color([GREEN if L >= 0 else RED, GREEN if R >= 0 else RED])
        reflex = info.get("reflex")
        self.wheel_txt.set_text(f"wheels   L {L:+4d}    R {R:+4d}" + ("   STUCK REFLEX" if reflex else ""))
        self.wheel_txt.set_color("#fb923c" if reflex else (RED if (L < 0 and R < 0) else GREEN))
        # camera
        if stage == 3:
            frame = info.get("frame")
            if frame is not None:
                h, w = frame.shape[:2]
                if self.img is None:
                    self.img = self.ax_cam.imshow(frame)
                    for x0 in (0, w / 2):
                        r = Rectangle((x0, 0), w / 2, h, color=BLUE, alpha=0.0)
                        self.ax_cam.add_patch(r)
                        self.halves.append(r)
                    self.ax_cam.axvline(w / 2, color="white", lw=1, alpha=0.5)
                else:
                    self.img.set_data(frame)
                self.cam_msg.set_text("")
            eyes = info.get("eyes") or (0.0, 0.0)
            for r, e in zip(self.halves, eyes):
                r.set_alpha(0.45 * min(e, config.MAX_HZ) / config.MAX_HZ)
            for t, e, side in zip(self.eye_txt, eyes, ("left", "right")):
                t.set_text(f"{side} eye: {e:.0f} Hz")
        # brain
        spikes = info.get("spikes")
        self.glow *= FADE
        if spikes is not None and len(spikes):
            self.glow[spikes] = 1.0
        g = self.glow[self.order][:, None]
        self.sc.set_facecolors(GREY * (1 - g) + self.hot * g)
        self.sc.set_sizes(self.base_size + 30 * g[:, 0])
        # bars
        eyes, hz = info.get("eyes") or (0.0, 0.0), info.get("hz") or {}
        vals = [eyes[0], eyes[1], hz.get("DNa01_L", 0) + hz.get("DNa02_L", 0),
                hz.get("DNa01_R", 0) + hz.get("DNa02_R", 0), (hz.get("MDN_L", 0) + hz.get("MDN_R", 0)) / 2]
        for bar, v in zip(self.bars, vals):
            bar.set_width(min(v, 260))
        self.status.set_text(status)
        # simulated room
        if self.ax_map is not None and snapshot:
            x, y, h = snapshot["x"], snapshot["y"], snapshot["h"]
            pts = [(x + 14 * math.cos(h), y + 14 * math.sin(h)),
                   (x + 9 * math.cos(h + 2.5), y + 9 * math.sin(h + 2.5)),
                   (x + 9 * math.cos(h - 2.5), y + 9 * math.sin(h - 2.5))]
            self.car_marker.set_xy(pts)
            self.trail_xy = (self.trail_xy + [(x, y)])[-400:]
            self.trail.set_data(*zip(*self.trail_xy))

    def show(self):
        """Draw everything (slow: the app does this ~10 times a second)."""
        plt.pause(0.001)

    def wait(self, seconds):
        """Wait without drawing, but still react to button clicks and keys."""
        self.fig.canvas.start_event_loop(seconds)

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def save(self, path):
        self.fig.savefig(path, dpi=70)
