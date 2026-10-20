"""A local page for watching training: loss and validation curves, live.

    python training/perception/watch.py          # then open the printed URL

Reads every run under ``runs/``: segmenter runs (train.py) and driving-model
runs (train_drive.py). New runs write ``metrics.jsonl`` (per-step loss and
per-epoch validation); older ones are read from ``train.log``. Nothing here
imports torch, so it can run beside a training job without touching the GPU.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = HERE / "runs"
PLOTS = RUNS / "plots"
EPOCH_LINE = re.compile(r"epoch (\d+)/(\d+) loss ([\d.]+) val IoU ([\d.]+) \((\d+) s\)")
STALE_S = 300  # a run whose files have not changed this long is not running


def read_run(path: Path) -> dict | None:
    metrics, log = path / "metrics.jsonl", path / "train.log"
    run = {"name": path.name, "steps": [], "epochs": [], "total_epochs": None, "done": False, "args": {}}
    source = None
    if metrics.exists():
        source = metrics
        for line in metrics.read_text().splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue  # a line still being written
            kind = row.get("type")
            if kind == "start":
                run["args"] = row["args"]
                run["total_epochs"] = int(row["args"]["epochs"])
                run["parameters"] = row.get("parameters")
                run["device"] = row.get("device")
            elif kind == "step":
                run["steps"].append({k: v for k, v in row.items() if k != "type"})
            elif kind == "epoch":
                run["epochs"].append({k: v for k, v in row.items() if k != "type" and isinstance(v, (int, float))})
            elif kind == "done":
                run["done"] = True
    elif log.exists():
        source = log
        text = log.read_text()
        for m in EPOCH_LINE.finditer(text):
            epoch, total, loss, iou, t = int(m[1]), int(m[2]), float(m[3]), float(m[4]), float(m[5])
            run["total_epochs"] = total
            run["epochs"].append({"epoch": epoch, "step": None, "train_loss": loss, "val_iou": iou, "t": t})
        run["done"] = "saved " in text
    if source is None:
        return None
    args = run["args"]
    if any("best_lap_s" in e for e in run["epochs"]) or "population" in args:
        run["kind"] = "optimizer"
    elif any("line_error_near_m" in e for e in run["epochs"]) or ("frames" in args and "tracks" not in args):
        run["kind"] = "drive"
    else:
        run["kind"] = "segmenter"
    # Steps per epoch, to put per-step points on an epoch axis.
    if run["kind"] == "drive" and args.get("frames"):
        run["steps_per_epoch"] = int(args["frames"]) // int(args["batch"])
    elif args.get("tracks"):
        run["steps_per_epoch"] = int(args["tracks"]) * int(args["frames"]) // int(args["batch"])
    # The true steps per epoch, read off the log: the first step logged in epoch 2 is the epoch length.
    # (A run that adds extra data, such as DAgger frames, has more steps than its args imply.)
    second = [r["step"] for r in run["steps"] if r.get("epoch") == 2]
    if second:
        run["steps_per_epoch"] = min(second)
    for e in run["epochs"]:
        if run["kind"] == "drive" and "train_loss" not in e and "path_loss" in e:
            e["train_loss"] = e["path_loss"] + e["speed_loss"] + float(args.get("mask_weight", 0.5)) * e["mask_loss"]
        if run["kind"] == "drive":
            e["val_iou"] = e.get("mask_iou", e.get("val_iou"))
    age = time.time() - source.stat().st_mtime
    run["updated_s_ago"] = age
    run["status"] = "finished" if run["done"] else "running" if age < STALE_S else "stopped"
    done = len(run["epochs"])
    if run["total_epochs"] and done and run["status"] == "running":
        per_epoch = run["epochs"][-1]["t"] / done
        run["eta_s"] = per_epoch * (run["total_epochs"] - done)
    return run


def all_runs():
    if not RUNS.exists():
        return []
    runs = [read_run(p) for p in sorted(RUNS.iterdir()) if p.is_dir() and not p.is_symlink()]
    return [r for r in runs if r]


# --------------------------------------------------------------- lap plots
# Top-down pictures of a model driving the track (tools/sim_video/lap_plot.py).
# The page can ask for a new one; the server runs it as a subprocess, one at a
# time, and only for a trained model it finds under runs/ or the expert.

JOB: dict = {"proc": None, "label": "", "started": 0.0, "log": PLOTS / "last.log"}
EXPERT = "expert-oracle"


def plot_options() -> list[str]:
    models = sorted(p.parent.name for p in RUNS.glob("drive-*/model.pt") if not p.parent.is_symlink())
    return [*models, EXPERT]


def list_plots() -> list[dict]:
    out = []
    if PLOTS.exists():
        for png in PLOTS.glob("*.png"):
            meta = {}
            side = png.with_suffix(".json")
            if side.exists():
                try:
                    meta = json.loads(side.read_text())
                except json.JSONDecodeError:
                    meta = {}
            out.append({"file": png.name, "made": png.stat().st_mtime, **meta})
    return sorted(out, key=lambda r: -r["made"])


def job_state() -> dict:
    proc = JOB["proc"]
    running = proc is not None and proc.poll() is None
    failed = proc is not None and not running and proc.returncode not in (0, None)
    return {
        "running": running,
        "label": JOB["label"],
        "elapsed_s": time.time() - JOB["started"] if running else 0.0,
        "failed": failed,
    }


def start_plot(run: str, cap: float, race: bool) -> dict:
    if job_state()["running"]:
        raise RuntimeError("a lap is already being drawn")
    if run not in plot_options():
        raise ValueError("unknown model")
    if not 0 < cap <= 25:
        raise ValueError("speed cap must be in (0, 25] m/s")
    PLOTS.mkdir(parents=True, exist_ok=True)
    out = PLOTS / f"{run}-cap{cap:g}{'-race' if race else ''}-{int(time.time())}.png"
    cmd = [
        sys.executable,
        str(ROOT / "tools/sim_video/lap_plot.py"),
        "--cap",
        str(cap),
        "--title",
        run,
        "--out",
        str(out),
    ]
    cmd += ["--oracle-line"] if run == EXPERT else ["--weights", str(RUNS / run / "model.pt")]
    if race:
        cmd.append("--race-limits")
    env = {
        **os.environ,
        "PYTHONPATH": f"{HERE}{os.pathsep}{ROOT / 'src/mcq_sim'}",
        "VECLIB_MAXIMUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
    }
    JOB.update(
        proc=subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=JOB["log"].open("w"), stderr=subprocess.STDOUT),
        label=f"{run} at {cap:g} m/s",
        started=time.time(),
    )
    return job_state()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def send(self, body: bytes, kind: str):
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send((HERE / "web" / "watch.html").read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/runs":
            self.send(json.dumps(all_runs()).encode(), "application/json")
        elif self.path == "/api/plots":
            body = {"plots": list_plots(), "options": plot_options(), "job": job_state()}
            self.send(json.dumps(body).encode(), "application/json")
        elif self.path.startswith("/plots/") and self.path.endswith(".png") and "/" not in self.path[7:]:
            target = PLOTS / self.path[7:]
            if target.exists():
                self.send(target.read_bytes(), "image/png")
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/api/plot":
            return self.send_error(404)
        try:
            size = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(min(size, 4096)) or b"{}")
            state = start_plot(str(data["run"]), float(data["cap"]), bool(data.get("race", False)))
            self.send(json.dumps(state).encode(), "application/json")
        except (ValueError, KeyError, RuntimeError, TypeError) as error:
            self.send(json.dumps({"error": str(error)}).encode(), "application/json")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8770)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Watching {RUNS}: http://127.0.0.1:{server.server_port} (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
