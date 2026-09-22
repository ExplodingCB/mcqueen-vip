"""Local simulator viewer. The browser displays Python physics; no second model."""

from __future__ import annotations

import base64
import csv
import io
import json
import math
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

from PIL import Image

from mcq_sim.environment import Simulator


def png_data(array):
    buffer = io.BytesIO()
    Image.fromarray(array).save(buffer, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def serve(sim: Simulator, port=8765):
    running = False
    initial_policy, initial_segmenter = sim.policy_name, sim.segmenter
    web = Path(__file__).resolve().parent / "web"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, data, kind="application/json", status=200):
            if isinstance(data, (dict, list)):
                data = json.dumps(data, allow_nan=False).encode()
            elif isinstance(data, str):
                data = data.encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def trusted(self):
            host = self.headers.get("Host", "")
            allowed = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
            origin = self.headers.get("Origin")
            return host in allowed and (origin is None or origin in {f"http://{h}" for h in allowed})

        def snapshot(self):
            rgb, truth = sim.frame()
            return {
                "state": asdict(sim.kart.state),
                "report": sim.report(),
                "running": running,
                "command": sim.command,
                "camera": png_data(rgb),
                "mask": png_data(truth.astype("uint8") * 255),
                "prediction": png_data(sim.last_prediction.astype("uint8") * 255)
                if sim.last_prediction is not None
                else None,
                "prediction_time_s": sim.last_frame_time,
                "speed_cap": sim.speed_cap,
            }

        def do_GET(self):
            if not self.trusted():
                return self.send({"error": "local origin required"}, status=403)
            path = urlparse(self.path).path
            if path in ("/", "/app.js", "/style.css"):
                name = "index.html" if path == "/" else path[1:]
                kind = {"index.html": "text/html", "app.js": "text/javascript", "style.css": "text/css"}[name]
                return self.send((web / name).read_bytes(), kind)
            if path == "/api/track":
                left, right = sim.track.width_at(sim.track.s)
                lx, ly, _ = sim.track.cartesian(sim.track.s, left)
                rx, ry, _ = sim.track.cartesian(sim.track.s, -right)
                return self.send(
                    {
                        "center": list(zip(sim.track.x.tolist(), sim.track.y.tolist(), strict=True)),
                        "left": list(zip(lx.tolist(), ly.tolist(), strict=True)),
                        "right": list(zip(rx.tolist(), ry.tolist(), strict=True)),
                        "length": sim.track.length,
                        "meta": sim.track.meta,
                        "vehicle": asdict(sim.vehicle_params),
                    }
                )
            if path == "/api/state":
                return self.send(self.snapshot())
            if path == "/api/aerial.png" and sim.track.meta.get("aerial"):
                return self.send(Path(sim.track.meta["aerial_path"]).read_bytes(), "image/png")
            if path == "/api/report":
                return self.send(sim.report())
            if path == "/api/log.csv":
                buffer = io.StringIO()
                if sim.log:
                    writer = csv.DictWriter(buffer, fieldnames=list(sim.log[0]))
                    writer.writeheader()
                    writer.writerows(sim.log)
                return self.send(buffer.getvalue(), "text/csv")
            return self.send({"error": "not found"}, status=404)

        def do_POST(self):
            nonlocal running
            if not self.trusted():
                return self.send({"error": "local origin required"}, status=403)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("invalid request size")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("request must be a JSON object")
                manual = data.get("manual", [0, 0, 1])
                if (
                    not isinstance(manual, list)
                    or len(manual) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in manual)
                ):
                    raise ValueError("invalid controls")
                path = urlparse(self.path).path
                if path == "/api/control":
                    action = data.get("action")
                    if action == "reset":
                        running = False
                        sim.reset()
                    elif action == "play":
                        running = not bool(sim.reason)
                    elif action == "pause":
                        running = False
                    elif action == "mode":
                        mode = data["mode"]
                        if mode not in ("reference", "camera-demo", "manual", initial_policy):
                            raise ValueError("unknown mode")
                        from mcq_sim.camera import DemoSegmenter

                        sim.policy_name = mode
                        sim.segmenter = (
                            initial_segmenter
                            if mode == initial_policy
                            else (DemoSegmenter() if mode == "camera-demo" else None)
                        )
                        running = False
                        sim.reset()
                    elif action == "speed":
                        value = float(data["value"])
                        if not math.isfinite(value) or not 1 <= value <= 8:
                            raise ValueError("speed cap must be 1 to 8 m/s")
                        sim.speed_cap = value
                    elif action == "parameter":
                        key, value = data["key"], float(data["value"])
                        limits = {"friction": (0.2, 1.4), "mass": (100, 250)}
                        if (
                            key not in limits
                            or not math.isfinite(value)
                            or not limits[key][0] <= value <= limits[key][1]
                        ):
                            raise ValueError("parameter outside slider range")
                        sim.vehicle_params = replace(sim.vehicle_params, **{key: value})
                        running = False
                        sim.reset()
                    elif action == "step":
                        running = False
                        sim.step(manual)
                    else:
                        raise ValueError("unknown action")
                elif path == "/api/tick":
                    if running:
                        for _ in range(2):
                            sim.step(manual)
                        if sim.reason:
                            running = False
                else:
                    return self.send({"error": "not found"}, status=404)
                return self.send(self.snapshot())
            except (ValueError, KeyError, TypeError) as exc:
                return self.send({"error": str(exc)}, status=400)

    server = HTTPServer(("127.0.0.1", port), Handler)
    print(f"Simulator: http://127.0.0.1:{server.server_port} (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
