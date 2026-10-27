"""Run every condition x seed as a separate process (2 at a time). Appends JSON lines to an output file.
usage: python3 batch.py CONDITIONS SEEDS [SECONDS] [OUTFILE]
The default output is results.jsonl, which already holds the published runs, so finished runs are skipped.
To reproduce from scratch, give a new file:  python3 batch.py nobrain,stage1,fly,shuffled,swapped 20 60 my_results.jsonl"""
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
conds = sys.argv[1].split(",")
seeds = range(int(sys.argv[2]))
secs = sys.argv[3] if len(sys.argv) > 3 else "60"
OUT = HERE / (sys.argv[4] if len(sys.argv) > 4 else "results.jsonl")
done = set()
if OUT.exists():
    for line in OUT.read_text().splitlines():
        d = json.loads(line)
        done.add((d["cond"], d["seed"]))
jobs = [(c, s) for s in seeds for c in conds if (c, s) not in done]


def one(job):
    c, s = job
    r = subprocess.run([sys.executable, str(HERE / "arena_test.py"), c, str(s), secs],
                       capture_output=True, text=True)
    line = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ""
    if not line.startswith("{"):
        return f"FAIL {c} {s}: {r.stderr[-300:]}"
    with open(OUT, "a") as f:
        f.write(line + "\n")
    d = json.loads(line)
    return f"{c:9s} {s:3d} contact {d['contact_s']:5.2f}s reflex {d['reflex_pct']:5.1f}% explored {d['explored_pct']:5.1f}%"


with ThreadPoolExecutor(2) as ex:
    for msg in ex.map(one, jobs):
        print(msg, flush=True)
