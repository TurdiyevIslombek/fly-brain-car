"""Fly-brain car: press STAGE 1, STAGE 2 or STAGE 3.
   bash run.sh            the real robot (Arduino on USB)
   bash run.sh --fake     no robot: a simulated car in a simulated room"""
import argparse
import csv
import sys
import time
from pathlib import Path
import config

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def make_parser():
    ap = argparse.ArgumentParser(description="Fly-brain car")
    ap.add_argument("--fake", action="store_true", help="no robot: simulated car and room")
    ap.add_argument("--port", default=config.PORT, help="Arduino port (default: auto)")
    ap.add_argument("--camera", default=None, help='stage 3 camera: auto, webcam, test, or http://IP:81/stream')
    ap.add_argument("--stage", type=int, choices=[1, 2, 3], help="start in this stage")
    ap.add_argument("--ticks", type=int, help=argparse.SUPPRESS)       # for automatic tests
    ap.add_argument("--save", help=argparse.SUPPRESS)
    ap.add_argument("--keys", help=argparse.SUPPRESS)                 # for automatic tests: "30:2,60:s"
    return ap


class Logger:
    """One CSV file per run in runs/: every tick, what the robot sensed and did."""
    FIELDS = ["t", "stage", "swap", "LL_cm", "L_cm", "C_cm", "R_cm", "RR_cm", "eye_L", "eye_R", "steer_L", "steer_R",
              "moonwalker", "wheel_L", "wheel_R", "reflex", "exp_L", "exp_R", "cam_motion", "vcc_mv", "event"]

    def __init__(self):
        self.file = self.writer = None

    def start(self, stage, swap):
        self.stop()
        (ROOT / "runs").mkdir(exist_ok=True)
        name = f"stage{stage}{'_swapped' if swap else ''}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.path = ROOT / "runs" / name
        self.file = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.t0 = time.time()

    def row(self, stage, swap, dists, info, event="", vcc_mv=0):
        if self.writer is None:
            return
        eyes, hz, raw = info.get("eyes") or (0, 0), info.get("hz") or {}, info.get("raw") or {}
        self.writer.writerow({
            "t": round(time.time() - self.t0, 3), "stage": stage, "swap": int(swap),
            "LL_cm": dists.get("LL"), "L_cm": dists.get("L"), "C_cm": dists.get("C"), "R_cm": dists.get("R"),
            "RR_cm": dists.get("RR"), "vcc_mv": vcc_mv or "",
            "eye_L": round(eyes[0], 1), "eye_R": round(eyes[1], 1),
            "steer_L": round(hz.get("DNa01_L", 0) + hz.get("DNa02_L", 0), 1),
            "steer_R": round(hz.get("DNa01_R", 0) + hz.get("DNa02_R", 0), 1),
            "moonwalker": round((hz.get("MDN_L", 0) + hz.get("MDN_R", 0)) / 2, 1),
            "wheel_L": info["wheels"][0], "wheel_R": info["wheels"][1],
            "reflex": int(bool(info.get("reflex"))),          # 1 = the stuck reflex drove, not the brain
            "exp_L": round(raw["exp_L"], 3) if "exp_L" in raw else "",
            "exp_R": round(raw["exp_R"], 3) if "exp_R" in raw else "",
            "cam_motion": round(raw["motion"], 1) if "motion" in raw else "",
            "event": event})                                 # LINK LOST, ARDUINO RESTARTED, NO SONAR ...

    def stop(self):
        if self.file:
            self.file.close()
            print(f"saved {self.path.relative_to(ROOT)}")
        self.file = self.writer = None


def main(argv=None):
    a = make_parser().parse_args(argv)
    if a.ticks:
        import matplotlib
        matplotlib.use("Agg")
    from common.brain_driver import BrainDriver
    from stage1.stage1 import Stage1
    from stage2.stage2 import Stage2
    from stage3.stage3 import Stage3
    from visualization.view import View

    if a.fake:
        from common.fake_robot import FakeRobot
        robot = FakeRobot()
        camera = a.camera or "test"
    else:
        from common.robot_link import RobotLink
        try:
            robot = RobotLink(a.port)
        except Exception as e:
            sys.exit(f"\n{e}\n(No robot? Try:  bash run.sh --fake)\n")
        if not robot.ready:
            print("Warning: the Arduino did not say READY. Did you upload arduino/car (bash upload_car.sh)?")
        camera = a.camera or config.CAMERA
    print(f"Connected: {robot.port}. Loading the fly brain...")
    driver = BrainDriver()
    stages = {1: Stage1(robot), 2: Stage2(robot, driver), 3: Stage3(robot, driver, camera)}
    state = {"stage": 0, "swap": False, "dists": {}, "last_dir": "C", "arduino_mode": None,
             "last_p": time.time(), "no_sonar": False, "restarts": 0, "notice": ("", 0.0),
             "low_5v": 0.0, "vcc_min": 0}
    log = Logger()

    def set_stage(n):
        if n == state["stage"]:
            return
        if state["stage"]:
            stages[state["stage"]].stop()
        state["stage"] = n
        state["dists"] = {}
        state["last_p"], state["no_sonar"] = time.time(), False
        if n:
            stages[n].start()
            log.start(n, state["swap"])
            view.set_stage(n, stages[n].title, state["swap"])
        else:
            robot.mode(0)
            log.stop()
            view.set_stage(0, "", False)

    def toggle_swap():
        state["swap"] = not state["swap"]
        view.set_swap(state["swap"])
        n = state["stage"]
        if n:
            log.start(n, state["swap"])
            view.set_stage(n, stages[n].title, state["swap"])

    view = View(set_stage, lambda: set_stage(0), toggle_swap, fake=a.fake)
    view.set_stage(0, "", False)
    if a.stage:
        set_stage(a.stage)
    idle = {"wheels": (0, 0), "eyes": None, "hz": None, "spikes": None, "frame": None}
    ticks, rate, last = 0, 0.0, time.time()
    period = config.TICK_MS / 1000.0                       # one tick = TICK_MS of real time
    draw_every = 1.0 / getattr(config, "DRAW_FPS", 10)     # redraw the window ~10 times a second
    last_draw, prev_start = 0.0, time.time()
    keys = dict(k.split(":") for k in a.keys.split(",")) if a.keys else {}
    try:
        while view.is_open() and (a.ticks is None or ticks < a.ticks):
            tick_start = time.time()
            if not a.ticks:      # the brain runs as long as really passed (between 1 and 2 ticks)
                real_ms = (tick_start - prev_start) * 1000.0
                driver.step_ms = min(max(real_ms, config.TICK_MS), 2 * config.TICK_MS)
            prev_start = tick_start
            for msg in robot.poll():
                if msg[0] == "P":
                    state["dists"][msg[1]] = msg[2]
                    state["last_dir"] = msg[1]
                    state["last_p"] = time.time()
                elif msg[0] == "MODE":
                    state["arduino_mode"] = msg[1]
                if state["stage"]:
                    stages[state["stage"]].on_message(msg)
            if str(ticks) in keys:
                view._key(type("Key", (), {"key": keys[str(ticks)].replace("space", " ")})())
            n = state["stage"]
            events = robot.take_events() if hasattr(robot, "take_events") else []
            # stages 1 and 2 must get sonar pings all the time; none for 1.5 s = something is wrong
            quiet = time.time() - state["last_p"]
            if n in (1, 2) and state["arduino_mode"] == n and quiet > 1.5 and not state["no_sonar"]:
                state["no_sonar"] = True
                events.append("NO SONAR")
            elif quiet < 1.5:
                state["no_sonar"] = False
            # the Arduino's own 5 V (= the servo's power when the servo is on the Arduino's 5V pin)
            vcc = getattr(robot, "vcc_mv", 0)
            if vcc and vcc < getattr(config, "LOW_5V_MV", 4500):
                if time.time() - state["low_5v"] > 5:
                    events.append(f"LOW 5V {vcc / 1000:.2f} V")
                state["low_5v"] = time.time()
                state["vcc_min"] = vcc if not state["vcc_min"] else min(state["vcc_min"], vcc)
            for ev in events:
                print(f"[{time.strftime('%H:%M:%S')}] {ev}", flush=True)
                if ev == "ARDUINO RESTARTED":
                    state["restarts"] += 1
                    state["arduino_mode"] = None          # it answers MODE,n again in a moment
                state["notice"] = (ev, time.time())
            info = stages[n].tick(state["swap"]) if n else idle
            log.row(n, state["swap"], state["dists"], info, "+".join(events), vcc)
            now = time.time()
            rate = 0.9 * rate + 0.1 / max(now - last, 1e-3)
            last = now
            status = f"{robot.port}   |   {rate:4.1f} ticks/s"
            if getattr(robot, "lost", False):
                status += f"   |   LINK LOST, motors stopped: {getattr(robot, 'why', '')}"
            if n == 3 and stages[3].camera is not None:
                cam = stages[3].camera
                status += f"   |   {cam.status}   {cam.fps:.0f} fps"
                if cam.age() > getattr(config, "CAM_STALE_S", 0.5):
                    status += f"   |   CAMERA FROZEN {cam.age():.1f} s: car stopped"
            if info.get("reflex"):
                status += "   |   STUCK -> reflex backs up and turns"
            if state["no_sonar"]:
                status += f"   |   NO SONAR DATA {quiet:.1f} s (sensor wires? WiFi?)"
            if time.time() - state["low_5v"] < 3:
                status += (f"   |   5 V DROPPED to {state['vcc_min'] / 1000:.2f} V: the servo can't turn "
                           f"(weak battery / motors pulling too much)")
            elif state["vcc_min"] and time.time() - state["low_5v"] > 10:
                state["vcc_min"] = 0
            if state["restarts"]:
                status += f"   |   Arduino restarted {state['restarts']}x (battery dips: charge it)"
            ev, ev_t = state["notice"]
            if ev and now - ev_t < 3 and ev not in ("NO SONAR",):
                status += f"   |   {ev}"
            if n and state["arduino_mode"] != n:
                status += "   |   waiting for the Arduino..."
            view.update(n, info, state["dists"], state["last_dir"], status,
                        robot.snapshot() if hasattr(robot, "snapshot") else None)
            if a.ticks:
                time.sleep(0.02)
            else:
                if now - last_draw >= draw_every:                # drawing is slow: not every tick
                    view.show()
                    last_draw = time.time()
                spare = period - (time.time() - tick_start)
                view.wait(max(spare, 0.001))                     # buttons and keys keep working
            ticks += 1
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        stages[3].close()
        log.stop()
        if a.save:
            view.save(a.save)
        print("Stopped. Motors off.")


if __name__ == "__main__":
    main()
