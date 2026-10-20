"""Render a closed-loop lap to MP4: whole-track map with the perceived corridor,
the camera frame, and the model's mask (red = false pavement, blue = missed)."""

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from mcq_sim.environment import Simulator
from mcq_sim.track import Track

spec, out = sys.argv[1], Path(sys.argv[2])
speed_cap = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
track = Track.load("tracks/purdue_gp")
tuning = None
if len(sys.argv) > 4 and sys.argv[4] == "race":  # the faster acceleration limits the faster teacher was tuned with
    from mcq_sim.params import RACE_LIMITS, load_params

    tuning = load_params().override(RACE_LIMITS)
sim = Simulator(track, policy="stack", perception=spec, speed_cap=speed_cap, tuning=tuning)

W, H = 1440, 810
MAP = 810
left_w, right_w = track.width_at(track.s)
lx, ly, _ = track.cartesian(track.s, left_w)
rx, ry, _ = track.cartesian(track.s, -right_w)
allx, ally = np.concatenate([lx, rx]), np.concatenate([ly, ry])
pad = 8
scale = (MAP - 2 * pad) / max(np.ptp(allx), np.ptp(ally))
x0, y0 = allx.min(), ally.min()


def px(x, y):
    return (pad + (np.asarray(x) - x0) * scale, MAP - pad - (np.asarray(y) - y0) * scale)


base = Image.new("RGB", (MAP, MAP), (46, 78, 40))
d = ImageDraw.Draw(base)
poly = list(zip(*px(lx, ly), strict=True)) + list(zip(*px(rx[::-1], ry[::-1]), strict=True))
d.polygon(poly, fill=(80, 80, 84))
d.line(list(zip(*px(lx, ly), strict=True)) + [tuple(np.array(px(lx[0], ly[0])).tolist())], fill=(240, 210, 60), width=2)
d.line(list(zip(*px(rx, ry), strict=True)) + [tuple(np.array(px(rx[0], ry[0])).tolist())], fill=(240, 210, 60), width=2)
# the expert's racing line, thin white, for comparison with what the model drives
ref = sim.stack.reference
rs = np.arange(0, ref.length, 0.5)
rlx, rly, _ = ref.cartesian(rs, np.zeros_like(rs))
d.line(list(zip(*px(rlx, rly), strict=True)), fill=(235, 235, 235), width=1)
from mcq_sim.perception import CorridorTracker  # noqa: E402

projector = CorridorTracker(sim.camera)
try:
    font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 22)
    small = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 17)
except OSError:
    font = small = ImageFont.load_default()

trail = []
step_s, next_frame, n = 0.2, 0.0, 0
with tempfile.TemporaryDirectory() as tmp:
    while sim.kart.state.t < 240 and not sim.reason and sim.progress < track.length:
        sim.step()
        s = sim.kart.state
        if s.t + 1e-9 < next_frame:
            continue
        next_frame += step_s
        trail.append((s.x, s.y))
        frame = Image.new("RGB", (W, H), (18, 18, 20))
        m = base.copy()
        md = ImageDraw.Draw(m)
        if len(trail) > 1:
            md.line([tuple(np.array(px(*p)).tolist()) for p in trail], fill=(30, 150, 255), width=3)
        per = getattr(sim.stack, "perceived", None)
        if per is not None:
            for edge in per:
                md.line(list(zip(*px(edge[:, 0], edge[:, 1]), strict=True)), fill=(255, 43, 214), width=3)
        line = getattr(sim.stack, "model_line", None)
        if line is not None:
            md.line(list(zip(*px(line[:, 0], line[:, 1]), strict=True)), fill=(255, 214, 0), width=4)
        cx, cy = px(s.x, s.y)
        md.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=(227, 36, 36), outline=(255, 255, 255))
        frame.paste(m, (0, 0))
        p = sim.perception
        if p is None:  # driving from the map: show the camera alone, larger
            rgb, _ = sim.frame()
            frame.paste(Image.fromarray(rgb).resize((630, 354)), (MAP, 40))
        elif p.last_rgb is not None:
            cam = Image.fromarray(p.last_rgb).resize((630, 354))
            viz = p.last_rgb.astype(float).copy()
            viz[p.last_prediction] = 0.5 * viz[p.last_prediction] + 0.5 * np.array([255, 43, 214])
            viz[p.last_prediction & ~p.last_truth] = [255, 0, 0]
            viz[~p.last_prediction & p.last_truth] = [0, 80, 255]
            drive = getattr(p, "last_drive", None)
            if drive is not None:
                cd = ImageDraw.Draw(cam)
                r, c, seen = projector._pixels(drive[0][:, 0], drive[0][:, 1])
                k = 630 / 480
                for i in np.flatnonzero(seen):
                    cd.ellipse([c[i] * k - 5, r[i] * k - 5, c[i] * k + 5, r[i] * k + 5], fill=(255, 214, 0))
                    cd.text((c[i] * k + 7, r[i] * k - 8), f"{drive[1][i]:.0f}", font=small, fill=(255, 255, 255))
            frame.paste(cam, (MAP, 40))
            frame.paste(Image.fromarray(viz.astype(np.uint8)).resize((630, 354)), (MAP, 40 + 354 + 40))
        fd = ImageDraw.Draw(frame)
        fd.text(
            (MAP + 10, 8),
            "Front camera (orthophoto drape); yellow: model's line, m/s",
            font=small,
            fill=(230, 230, 230),
        )
        fd.text(
            (MAP + 10, 40 + 354 + 8),
            "Model mask: magenta pavement, red false, blue missed",
            font=small,
            fill=(230, 230, 230),
        )
        stop = sim.stack.stop_reason or sim.reason or ""
        fd.text(
            (12, 10),
            f"t {s.t:5.1f} s   v {s.v:4.1f} m/s   progress {100 * sim.progress / track.length:5.1f}%   {stop}",
            font=font,
            fill=(255, 255, 255),
        )
        fd.text(
            (12, MAP - 30),
            "white: expert racing line   yellow: model's line   blue: driven   magenta: corridor",
            font=small,
            fill=(255, 255, 255),
        )
        frame.save(f"{tmp}/{n:05d}.png")
        n += 1
    report = sim.report()
    print(
        {
            k: report.get(k)
            for k in (
                "passed",
                "stop_reason",
                "sim_time_s",
                "minimum_body_clearance_m",
                "boundary_violations",
                "model_plans",
                "model_vetoes",
            )
        }
    )
    # 0.2 s of sim per frame at 20 fps is 4x real time
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "20",
            "-i",
            f"{tmp}/%05d.png",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "23",
            str(out),
        ],
        check=True,
    )
print(out, n, "frames")
