"""Tier A frames: pull camera images out of a kart session log with the pose
at each exposure, so ``project_labels.py`` can label them from the survey.

    python -m mcq_training.extract_mcap 2026-11-02_1410_purdue_kartA_practice.mcap \\
        --image-topic /camera/front/image/compressed --ego-topic /ego_state \\
        --fps 5 --track purdue --out training/frames

Writes ``frames/<source>/<session>/<t_ms>.jpg``, a ``poses.csv`` next to them
(frame, t_ns, x, y, yaw, v, gnss_status) and appends rows to
``frames/index.csv`` with ``license`` set to ``own``. The pose is interpolated
between the two EgoState messages bracketing the image stamp (plus
``--time-offset`` for exposure latency); frames without a bracketing pose,
below ``--min-speed`` or without an RTK fix are skipped, because a label
projected through a bad pose is worse than no label.
"""

from __future__ import annotations

import argparse
import io
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from mcq_training import DATA
from mcq_training.dataset import FRAME_COLUMNS, append_rows

GNSS_NAMES = {0: "NONE", 1: "FLOAT", 2: "FIXED"}
POSE_COLUMNS = ["frame", "t_ns", "x", "y", "yaw", "v", "gnss_status"]


def iter_messages(bag: Path, topic: str):
    """(log_time_ns, decoded message) for one topic, in log-time order."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    with open(bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, _channel, message, decoded in reader.iter_decoded_messages(topics=[topic]):
            yield int(message.log_time), decoded


def stamp_ns(msg, fallback_ns: int) -> int:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns
    t = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return t if t > 0 else fallback_ns


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseTrack:
    """EgoState samples with interpolation by time."""

    def __init__(self):
        self.t: list[int] = []
        self.x: list[float] = []
        self.y: list[float] = []
        self.yaw: list[float] = []
        self.v: list[float] = []
        self.gnss: list[int] = []

    def add(self, t_ns: int, msg) -> None:
        self.t.append(t_ns)
        self.x.append(float(msg.pose.position.x))
        self.y.append(float(msg.pose.position.y))
        self.yaw.append(yaw_from_quaternion(msg.pose.orientation))
        self.v.append(float(getattr(msg, "v", 0.0)))
        self.gnss.append(int(getattr(msg, "gnss_status", 2)))

    def finalize(self) -> None:
        order = np.argsort(self.t)
        self._t = np.asarray(self.t, dtype=np.int64)[order]
        self._x = np.asarray(self.x)[order]
        self._y = np.asarray(self.y)[order]
        self._yaw = np.asarray(self.yaw)[order]
        self._v = np.asarray(self.v)[order]
        self._gnss = np.asarray(self.gnss)[order]

    def __len__(self) -> int:
        return len(self.t)

    def at(self, t_ns: int, max_gap_ns: int) -> dict | None:
        i = int(np.searchsorted(self._t, t_ns))
        if i <= 0 or i >= len(self._t):
            return None
        t0, t1 = self._t[i - 1], self._t[i]
        if t1 - t0 > max_gap_ns:
            return None
        a = (t_ns - t0) / max(t1 - t0, 1)
        yaw0, yaw1 = self._yaw[i - 1], self._yaw[i]
        dyaw = (yaw1 - yaw0 + np.pi) % (2 * np.pi) - np.pi
        return {
            "x": float(self._x[i - 1] + a * (self._x[i] - self._x[i - 1])),
            "y": float(self._y[i - 1] + a * (self._y[i] - self._y[i - 1])),
            "yaw": float((yaw0 + a * dyaw + np.pi) % (2 * np.pi) - np.pi),
            "v": float(self._v[i - 1] + a * (self._v[i] - self._v[i - 1])),
            "gnss_status": int(min(self._gnss[i - 1], self._gnss[i])),
        }


def decode_image(msg) -> tuple[Image.Image, bytes | None]:
    """PIL image from a CompressedImage or Image message; the raw JPEG bytes
    are returned too so they can be written unchanged."""
    data = msg.data
    raw = bytes(data) if not isinstance(data, bytes) else data
    if hasattr(msg, "format"):
        fmt = (msg.format or "").lower()
        im = Image.open(io.BytesIO(raw))
        im.load()
        return im.convert("RGB"), (raw if "jpeg" in fmt or "jpg" in fmt or im.format == "JPEG" else None)
    enc = msg.encoding.lower()
    h, w = int(msg.height), int(msg.width)
    arr = np.frombuffer(raw, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        arr = arr.reshape(h, int(msg.step) // 3 if msg.step else w, 3)[:, :w]
        if enc == "bgr8":
            arr = arr[..., ::-1]
        return Image.fromarray(np.ascontiguousarray(arr), "RGB"), None
    if enc in ("mono8", "8uc1"):
        return Image.fromarray(arr.reshape(h, -1)[:, :w], "L").convert("RGB"), None
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


def extract(
    bag: Path,
    image_topic: str,
    ego_topic: str,
    out_root: Path,
    source: str,
    video_id: str,
    fps: float,
    track: str = "",
    camera: str = "chassis_front",
    time_offset_s: float = 0.0,
    min_speed: float = 0.5,
    require_fixed: bool = True,
    max_pose_gap_s: float = 0.05,
    long_edge: int | None = None,
    limit: int | None = None,
) -> dict:
    poses = PoseTrack()
    for log_time, msg in iter_messages(bag, ego_topic):
        poses.add(stamp_ns(msg, log_time), msg)
    if len(poses) == 0:
        raise SystemExit(f"no {ego_topic} messages in {bag}")
    poses.finalize()

    dest = out_root / source / video_id
    dest.mkdir(parents=True, exist_ok=True)
    period_ns = int(round(1e9 / fps))
    offset_ns = int(round(time_offset_s * 1e9))
    gap_ns = int(round(max_pose_gap_s * 1e9))
    stats = {"images": 0, "kept": 0, "no_pose": 0, "slow": 0, "no_fix": 0}
    index_rows, pose_rows = [], []
    next_t = None
    for log_time, msg in iter_messages(bag, image_topic):
        stats["images"] += 1
        t = stamp_ns(msg, log_time)
        if next_t is not None and t < next_t:
            continue
        next_t = (t if next_t is None else next_t) + period_ns
        pose = poses.at(t + offset_ns, gap_ns)
        if pose is None:
            stats["no_pose"] += 1
            continue
        if pose["v"] < min_speed:
            stats["slow"] += 1
            continue
        if require_fixed and pose["gnss_status"] != 2:
            stats["no_fix"] += 1
            continue
        im, raw = decode_image(msg)
        if long_edge and max(im.size) > long_edge:
            scale = long_edge / max(im.size)
            im = im.resize((round(im.width * scale), round(im.height * scale)), Image.BILINEAR)
            raw = None
        t_ms = t // 1_000_000
        name = f"{t_ms:012d}.jpg"
        path = dest / name
        if raw is not None:
            path.write_bytes(raw)
        else:
            im.save(path, quality=92)
        rel = str(Path(source) / video_id / name)
        index_rows.append(
            {
                "frame": rel,
                "source": source,
                "video_id": video_id,
                "t_ms": t_ms,
                "camera": camera,
                "license": "own",
                "track": track,
                "dhash": "",
            }
        )
        pose_rows.append({"frame": rel, "t_ns": t, **pose})
        stats["kept"] += 1
        if limit and stats["kept"] >= limit:
            break
    append_rows(out_root / "index.csv", FRAME_COLUMNS, index_rows)
    append_rows(dest / "poses.csv", POSE_COLUMNS, pose_rows)
    stats["poses"] = len(poses)
    stats["dest"] = str(dest)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--image-topic", default="/camera/front/image/compressed")
    ap.add_argument("--ego-topic", default="/ego_state")
    ap.add_argument("--out", type=Path, default=DATA / "frames")
    ap.add_argument("--source", default=None, help="source id; default kart_<bag stem>")
    ap.add_argument("--video-id", default=None, help="default: the bag stem")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--track", default="", help="track id the session was driven on")
    ap.add_argument("--camera", default="chassis_front")
    ap.add_argument("--time-offset", type=float, default=0.0, help="s added to the image stamp before pose lookup")
    ap.add_argument("--min-speed", type=float, default=0.5)
    ap.add_argument("--allow-float", action="store_true", help="keep frames without an RTK fix")
    ap.add_argument("--long-edge", type=int, default=None, help="resize so the long edge is this many px")
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args(argv)
    video_id = args.video_id or args.bag.stem
    source = args.source or f"kart_{args.bag.stem}"
    stats = extract(
        args.bag,
        args.image_topic,
        args.ego_topic,
        args.out,
        source,
        video_id,
        args.fps,
        args.track,
        args.camera,
        args.time_offset,
        args.min_speed,
        not args.allow_float,
        long_edge=args.long_edge,
        limit=args.limit,
    )
    print(
        f"{stats['images']} images, {stats['poses']} poses: kept {stats['kept']}, "
        f"no pose {stats['no_pose']}, standing {stats['slow']}, no fix {stats['no_fix']} -> {stats['dest']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
