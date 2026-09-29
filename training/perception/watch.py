"""A local page for watching training: loss and validation curves, live.

    python training/perception/watch.py          # then open the printed URL

Reads every run under ``runs/``. New runs write ``metrics.jsonl`` (per-step loss
and per-epoch validation); older ones are read from ``train.log``. Nothing here
imports torch, so it can run beside a training job without touching the GPU.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = HERE / "runs"
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
                run["total_epochs"] = row["args"]["epochs"]
                run["parameters"] = row.get("parameters")
                run["device"] = row.get("device")
            elif kind == "step":
                run["steps"].append({"step": row["step"], "loss": row["loss"], "t": row["t"]})
            elif kind == "epoch":
                run["epochs"].append({k: row[k] for k in ("epoch", "step", "train_loss", "val_iou", "t")})
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
        else:
            self.send_error(404)


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
