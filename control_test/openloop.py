"""Open-loop test: show the brain a looming object on one side and read the steering neurons.
No car, no room, no reflex: only the wiring decides the answer.

For each wiring: 0.3 s warm-up, then 1.0 s of looming on the LEFT eye only, then the RIGHT eye only,
then BOTH eyes (something straight ahead). Measured: firing rates (Hz) of the steering neurons
DNa01+DNa02 on each side, and of the backward-walking neurons MDN ("moonwalkers").
turn = right steering - left steering (> 0 = the car turns right).
"""
import json
import sys
import time
import numpy as np
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "brain"))
from flybrain import FlyBrain, W_SYN  # noqa: E402
import wiring  # noqa: E402

OUT = ["DNa01_L", "DNa02_L", "DNa01_R", "DNa02_R", "MDN_L", "MDN_R"]
HZ = 250.0


def measure(W, seed=0, ms=1000):
    b = FlyBrain(HERE.parent / "brain" / "locomotion", seed=seed)
    b.W = (W * W_SYN).astype(np.float32).toarray()
    res = {}
    for name, stim in (("left", {"loom_L": HZ}), ("right", {"loom_R": HZ}), ("both", {"loom_L": HZ, "loom_R": HZ})):
        b.reset()
        b.run({}, 300, OUT)
        c = b.run(stim, ms, OUT)
        hz = {k: c[k] / len(b.groups[k]) / (ms / 1000) for k in OUT}
        res[name] = {"turn": (hz["DNa01_R"] + hz["DNa02_R"]) - (hz["DNa01_L"] + hz["DNa02_L"]),
                     "back": (hz["MDN_L"] + hz["MDN_R"]) / 2,
                     "steer_L": hz["DNa01_L"] + hz["DNa02_L"], "steer_R": hz["DNa01_R"] + hz["DNa02_R"]}
    # how much of the whole brain fires during "both" (runaway check)
    b.reset()
    b.run({}, 300, [])
    n_spk = 0
    for _ in range(int(500 / b.dt)):
        n_spk += b.step({"loom_L": HZ, "loom_R": HZ}).size
    res["mean_rate_all_hz"] = n_spk / b.N / 0.5
    return res


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    t0 = time.time()
    out = {"real": measure(wiring.load_real())}
    print("real", json.dumps(out["real"]), f"{time.time() - t0:.1f}s", flush=True)
    for k in range(n):
        out[f"shuf{k}"] = measure(wiring.shuffled(k))
        r = out[f"shuf{k}"]
        print(f"shuf{k} L-turn {r['left']['turn']:7.1f} R-turn {r['right']['turn']:7.1f} "
              f"both-back {r['both']['back']:6.1f} all {r['mean_rate_all_hz']:.1f}", flush=True)
    (HERE / "openloop.json").write_text(json.dumps(out, indent=1))
