"""Held-out evaluation of a segmenter, in the terms docs/08 section 7 uses.

Frames come from the tracks the model never trained on, drawn with the same
randomized appearance as training but rendered through the kart's nominal
480 x 270 camera, which is the camera the model is deployed behind. Reported:

  iou            mean pavement IoU per frame
  edge_error_m   mean absolute lateral error of the left and right edge at about
                 15 m ahead, after ground-plane projection, against the edges the
                 true mask gives (docs/08 acceptance: under 0.3 m)
  edge_found     of frames where the true pavement has both edges at 15 m, the
                 fraction where the model's mask also does

The color-threshold demo segmenter is scored on the same frames as a floor: a
model that does not beat it has learned nothing.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import synth
from mcq_sim.camera import CameraParams, DemoSegmenter, TrackCamera, mask_iou
from mcq_sim.perception import edges_from_mask
from mcq_sim.track import Track

ROOT = Path(__file__).resolve().parents[2]
RANGE_M = 15.0


def edge_at(edge: np.ndarray, target=RANGE_M):
    """Lateral position of a forward-ordered edge polyline at ``target`` metres."""
    if edge is None or edge[:, 0].min() > target or edge[:, 0].max() < target:
        return None
    order = np.argsort(edge[:, 0])
    return float(np.interp(target, edge[order, 0], edge[order, 1]))


def evaluate_on(track: Track, segmenter, n_frames: int, seed: int, plain=False):
    rng = np.random.default_rng(seed)
    nominal = CameraParams()
    camera = TrackCamera(track, nominal)
    ious, errors, found, has_truth, ms = [], [], 0, 0, []
    for _ in range(n_frames):
        x, y, yaw = synth.sample_pose(track, rng)
        look = synth.Look.draw(rng)
        look.camera = nominal
        if plain:  # the simulator's own camera, for the closed-loop interface
            from types import SimpleNamespace

            rgb, truth = camera.render(SimpleNamespace(x=x, y=y, yaw=yaw))
        else:
            rgb, truth = synth.render(track, x, y, yaw, look, rng)
        started = time.perf_counter()
        probability = segmenter.predict(rgb)
        ms.append((time.perf_counter() - started) * 1000)
        prediction = np.asarray(probability) >= 0.5
        ious.append(mask_iou(prediction, truth))
        truth_edges, predicted_edges = edges_from_mask(truth, camera), edges_from_mask(prediction, camera)
        if truth_edges is None or any(edge_at(e) is None for e in truth_edges):
            continue  # nothing to score here: the true pavement is not visible at that range
        has_truth += 1
        if predicted_edges is None or any(edge_at(e) is None for e in predicted_edges):
            continue
        found += 1
        errors.extend(abs(edge_at(t) - edge_at(p)) for t, p in zip(truth_edges, predicted_edges, strict=True))
    return {
        "frames": n_frames,
        "iou": float(np.mean(ious)),
        "iou_p10": float(np.percentile(ious, 10)),
        "edge_error_m": float(np.mean(errors)) if errors else None,
        "edge_error_p90_m": float(np.percentile(errors, 90)) if errors else None,
        "edge_found": found / has_truth if has_truth else None,
        "edge_frames": has_truth,
        "inference_ms_p95": float(np.percentile(ms, 95)),
    }


def held_out_tracks():
    return {
        "purdue_gp": Track.load(ROOT / "tracks/purdue_gp"),
        "synthetic_oval": Track.load(ROOT / "tracks/synthetic_oval"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--out")
    args = parser.parse_args()
    report = {}
    contenders = {"color_threshold_demo": DemoSegmenter()}
    if args.weights:
        from segmenter import Segmenter

        contenders["trained"] = Segmenter(args.weights)
    for name, segmenter in contenders.items():
        report[name] = {}
        for track_name, track in held_out_tracks().items():
            report[name][track_name] = {
                "randomized_appearance": evaluate_on(track, segmenter, args.frames, seed=11),
                "simulator_camera": evaluate_on(track, segmenter, args.frames, seed=12, plain=True),
            }
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text + "\n")


if __name__ == "__main__":
    main()
