"""Local simulator viewer. The browser displays Python physics; no second model."""

from __future__ import annotations

import base64
import csv
import io
import json
import math
import os
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from PIL import Image

from mcq_sim.environment import MAX_SPEED_CAP, Simulator
from mcq_sim.params import RACE_LIMITS, load_params


def png_data(array):
    buffer = io.BytesIO()
    Image.fromarray(array).save(buffer, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def serve(sim: Simulator, port=8765, allow_hosts=()):
    running = False
    demo = {"model": None, "race": False}
    runs = Path(__file__).resolve().parents[3] / "training" / "perception" / "runs"
    extra_hosts = {h.strip().lower() for h in allow_hosts if h.strip()}
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
            if host in allowed and (origin is None or origin in {f"http://{h}" for h in allowed}):
                return True
            name = host.rsplit(":", 1)[0].lower()
            if name in extra_hosts:  # a named host the operator allowed, such as a tailnet address
                return origin is None or urlparse(origin).hostname == name
            return False

        def demo_models(self):
            names = sorted(p.parent.name for p in runs.glob("drive-*/model.pt") if not p.parent.is_symlink())
            return [*names, "expert"]

        def demo_snapshot(self):
            """A light state for the demo page: where the kart is, what it sees and
            what the model wants, with a JPEG camera and a small mask overlay."""
            s, stack, perception = sim.kart.state, sim.stack, sim.perception
            rgb = perception.last_rgb if perception is not None and perception.last_rgb is not None else sim.frame()[0]
            jpeg = io.BytesIO()
            Image.fromarray(rgb).save(jpeg, format="JPEG", quality=72)
            mask = None
            prediction = getattr(perception, "last_prediction", None)
            if prediction is not None:
                small = prediction[::3, ::3]
                overlay = np.zeros((*small.shape, 4), dtype=np.uint8)
                overlay[small] = (255, 43, 214, 125)
                buffer = io.BytesIO()
                Image.fromarray(overlay, "RGBA").save(buffer, format="PNG")
                mask = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")
            drive = getattr(perception, "last_drive", None)
            thin = lambda a: [[round(float(x), 2), round(float(y), 2)] for x, y in a[::2]]  # noqa: E731
            estimate = sim.estimate_snapshot()
            return {
                "t": s.t,
                "x": s.x,
                "y": s.y,
                "yaw": s.yaw,
                "v": s.v,
                "steer": s.steer,
                "throttle": s.throttle,
                "brake": s.brake,
                "a_lat": s.a_lat,
                "progress": sim.progress,
                "length": sim.track.length,
                "laps": int(sim.progress / sim.track.length),
                "clearance": sim.min_clearance if math.isfinite(sim.min_clearance) else None,
                "reason": sim.reason or (stack.stop_reason if stack is not None else None),
                "running": running,
                "finished": sim.progress >= sim.track.length,
                "speed_cap": sim.speed_cap,
                "model": demo["model"],
                "race": demo["race"],
                "camera": "data:image/jpeg;base64," + base64.b64encode(jpeg.getvalue()).decode("ascii"),
                "mask": mask,
                "drive": [[float(p[0]), float(p[1]), float(v)] for p, v in zip(drive[0], drive[1], strict=True)]
                if drive is not None
                else None,
                "model_line": thin(stack.model_line) if stack is not None and stack.model_line is not None else None,
                "perceived": [thin(b) for b in stack.perceived] if stack is not None and stack.perceived else None,
                "counters": {
                    "plans": getattr(stack, "model_plans", 0),
                    "vetoes": getattr(stack, "model_vetoes", 0),
                    "corrections": getattr(stack, "model_corrections", 0),
                },
                "estimate": estimate,
            }

        def snapshot(self):
            rgb, truth = sim.frame()
            # with the stack driving, the model's output lives on its perception
            prediction = getattr(sim.perception, "last_prediction", None) if sim.stack is not None else None
            prediction = sim.last_prediction if prediction is None else prediction
            return {
                "state": asdict(sim.kart.state),
                "report": sim.report(),
                "running": running,
                "command": sim.command,
                "camera": png_data(rgb),
                "mask": png_data(truth.astype("uint8") * 255),
                "prediction": png_data(prediction.astype("uint8") * 255) if prediction is not None else None,
                "model_line": sim.stack.model_line.tolist()
                if sim.stack is not None and sim.stack.model_line is not None
                else None,
                "perceived": [b.tolist() for b in sim.stack.perceived]
                if sim.stack is not None and sim.stack.perceived is not None
                else None,
                "prediction_time_s": sim.last_frame_time,
                "speed_cap": sim.speed_cap,
                "estimate": sim.estimate_snapshot(),
            }

        def do_GET(self):
            if not self.trusted():
                return self.send({"error": "local origin required"}, status=403)
            path = urlparse(self.path).path
            if path == "/demo":
                return self.send((web / "demo.html").read_bytes(), "text/html")
            if path == "/api/demo/info":
                return self.send(
                    {
                        "models": self.demo_models(),
                        "model": demo["model"],
                        "race": demo["race"],
                        "camera": asdict(sim.camera_params),
                        "aerial": sim.track.meta.get("aerial"),
                        "track_id": sim.track.track_id,
                    }
                )
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
            if path == "/api/reference":
                from mcq_sim.stack import reference_for

                ref = reference_for(sim.track, sim.vehicle_params.half_width)
                return self.send({"line": list(zip(ref.x.tolist(), ref.y.tolist(), strict=True))})
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
                        if mode not in ("reference", "camera-demo", "manual", "stack", initial_policy):
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
                    elif action == "demo_model":
                        name = data["name"]
                        if name not in self.demo_models():
                            raise ValueError("unknown model")
                        if name == "expert":  # the map-driven stack: privileged, for comparison
                            sim.perception_spec = "map"
                        else:
                            os.environ["MCQ_DRIVER_WEIGHTS"] = str(runs / name / "model.pt")
                            sim.perception_spec, sim.perception_model = "driver:create", None
                        sim.policy_name = "stack"
                        demo["model"] = name
                        running = False
                        sim.reset()
                    elif action == "race_limits":
                        demo["race"] = bool(data.get("on"))
                        sim.tuning = load_params().override(dict(RACE_LIMITS) if demo["race"] else {})
                        running = False
                        sim.reset()
                    elif action == "speed":
                        value = float(data["value"])
                        if not math.isfinite(value) or not 1 <= value <= MAX_SPEED_CAP:
                            raise ValueError(f"speed cap must be 1 to {MAX_SPEED_CAP:g} m/s")
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
                elif path == "/api/demo/tick":
                    steps = max(1, min(int(data.get("steps", 2)), 12))
                    if running:
                        for _ in range(steps):
                            sim.step()
                            stack = sim.stack
                            stopped = stack is not None and stack.stop_reason and sim.kart.state.v < 0.05
                            if sim.reason or stopped or sim.progress >= sim.track.length:
                                running = False  # a crash, a safety stop at rest, or one lap done
                                break
                    return self.send(self.demo_snapshot())
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

    sim.camera.skip_truth = True  # the viewer shows a picture; it does not score the mask against the truth
    server = HTTPServer(("127.0.0.1", port), Handler)
    print(f"Simulator: http://127.0.0.1:{server.server_port} (Ctrl+C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
