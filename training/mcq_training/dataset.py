"""Dataset assembly: join the frame index with the label index, assign tiers,
split by video so no clip leaks between train and validation, and write the
split files a training run reads.

Inputs are the two CSVs the earlier stages maintain:

* ``frames/index.csv`` from ``extract_frames.py`` and ``extract_mcap.py``
  (frame, source, video_id, t_ms, camera, license, track, dhash)
* ``labels/index.csv`` from ``project_labels.py``, ``autolabel.py`` and the
  review export (frame, label, source, video_id, method, status, reviewer,
  date, note)

Output is ``datasets/<name>/{train,val,test}.csv`` plus ``manifest.json``
with counts, hashes and the configuration, so a run can name exactly what
it trained on. Pixels never move: the CSVs point into the frame and label
roots.
"""

from __future__ import annotations

import csv
import fnmatch
import hashlib
import json
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml

FRAME_COLUMNS = ["frame", "source", "video_id", "t_ms", "camera", "license", "track", "dhash"]
LABEL_COLUMNS = ["frame", "label", "source", "video_id", "method", "status", "reviewer", "date", "note"]
SPLIT_COLUMNS = ["frame", "label", "source", "video_id", "tier", "camera", "track", "method", "status", "weight"]
TIERS = ("A", "B", "C")
STATUS_RANK = {"reviewed": 2, "auto": 1}


@dataclass
class Sample:
    frame: str
    label: str
    source: str
    video_id: str
    tier: str
    camera: str
    track: str
    method: str
    status: str
    weight: float = 1.0


# ------------------------------------------------------------------ CSV I/O
def read_rows(path: Path) -> list[dict]:
    path = Path(path)
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def append_rows(path: Path, columns: list[str], rows: list[dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in columns})


def write_split(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SPLIT_COLUMNS)
        w.writeheader()
        for s in samples:
            w.writerow(asdict(s))


def read_split(path: Path) -> list[Sample]:
    out = []
    for r in read_rows(path):
        r["weight"] = float(r.get("weight") or 1.0)
        out.append(Sample(**{k: r.get(k, "") for k in Sample.__dataclass_fields__}))
    return out


# ------------------------------------------------------------------ rules
def matches(patterns: list[str], *names: str) -> bool:
    return any(fnmatch.fnmatchcase(n, p) for p in patterns for n in names if n)


def tier_for(row: dict, tiers: dict[str, list[str]]) -> str:
    """Explicit source or video patterns per tier first; then the license:
    our own footage is A, Creative Commons or granted permission is B, the
    rest is C."""
    for tier in TIERS:
        if matches(tiers.get(tier, []), row.get("source", ""), row.get("video_id", "")):
            return tier
    lic = (row.get("license") or "").lower()
    if lic == "own":
        return "A"
    if "creative commons" in lic or lic in ("cc-by", "granted", "permission"):
        return "B"
    return "C"


def join(frames: list[dict], labels: list[dict], statuses: tuple[str, ...] = ("auto", "reviewed")) -> list[dict]:
    """One row per labeled frame. A reviewed label beats an automatic one for
    the same frame; a rejected one removes the frame."""
    best: dict[str, dict] = {}
    rejected: set[str] = set()
    for lab in labels:
        status = lab.get("status", "")
        if status == "rejected":
            rejected.add(lab["frame"])
            continue
        if status not in statuses:
            continue
        cur = best.get(lab["frame"])
        if cur is None or STATUS_RANK.get(status, 0) >= STATUS_RANK.get(cur.get("status", ""), 0):
            best[lab["frame"]] = lab
    out = []
    for fr in frames:
        lab = best.get(fr["frame"])
        if lab is None or fr["frame"] in rejected:
            continue
        out.append({**fr, "label": lab["label"], "method": lab.get("method", ""), "status": lab.get("status", "")})
    return out


def split_hash(key: str) -> float:
    """Deterministic value in [0, 1) so a video keeps its split across rebuilds."""
    h = hashlib.sha1(key.encode("utf-8")).digest()
    return int.from_bytes(h[:8], "big") / 2**64


def assign(rows: list[dict], config: dict) -> dict[str, list[Sample]]:
    tiers = config.get("tiers", {}) or {}
    weights = {t: float(v) for t, v in (config.get("weights") or {}).items()}
    test_patterns = config.get("test", []) or []
    val_patterns = config.get("val", []) or []
    val_frac = float(config.get("val_fraction", 0.15))
    splits: dict[str, list[Sample]] = {"train": [], "val": [], "test": []}
    for r in rows:
        tier = tier_for(r, tiers)
        s = Sample(
            frame=r["frame"],
            label=r["label"],
            source=r.get("source", ""),
            video_id=r.get("video_id", ""),
            tier=tier,
            camera=r.get("camera", ""),
            track=r.get("track", ""),
            method=r.get("method", ""),
            status=r.get("status", ""),
            weight=weights.get(tier, 1.0),
        )
        if matches(test_patterns, s.source, s.video_id):
            splits["test"].append(s)
        elif matches(val_patterns, s.source, s.video_id) or split_hash(s.video_id or s.frame) < val_frac:
            splits["val"].append(s)
        else:
            splits["train"].append(s)
    return splits


# ------------------------------------------------------------------ build
def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_hash(repo: Path) -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=repo, capture_output=True, text=True, check=True
        ).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def counts(samples: list[Sample]) -> dict:
    out: dict = {"frames": len(samples), "videos": len({s.video_id for s in samples}), "tier": {}, "status": {}}
    for s in samples:
        out["tier"][s.tier] = out["tier"].get(s.tier, 0) + 1
        out["status"][s.status] = out["status"].get(s.status, 0) + 1
    return out


def build(config: dict, out_dir: Path, frames_index: Path, labels_index: Path, repo: Path | None = None) -> dict:
    frames = read_rows(frames_index)
    labels = read_rows(labels_index)
    rows = join(frames, labels, tuple(config.get("statuses", ["auto", "reviewed"])))
    splits = assign(rows, config)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "name": config.get("name", out_dir.name),
        "config": config,
        "frames_index": str(frames_index),
        "labels_index": str(labels_index),
        "git": git_hash(repo or Path.cwd()),
        "splits": {},
    }
    for name, samples in splits.items():
        path = out_dir / f"{name}.csv"
        write_split(path, samples)
        manifest["splits"][name] = {**counts(samples), "sha256": sha256_file(path)}
    with open(out_dir / "manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
    return manifest


def main(argv=None) -> int:
    import argparse

    from mcq_training import DATA, REPO

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, type=Path, help="dataset yaml (see configs/dataset_example.yaml)")
    ap.add_argument("--frames-index", type=Path, default=DATA / "frames" / "index.csv")
    ap.add_argument("--labels-index", type=Path, default=DATA / "labels" / "index.csv")
    ap.add_argument("--out", type=Path, default=None, help="default datasets/<name>")
    args = ap.parse_args(argv)
    with open(args.config) as f:
        config = yaml.safe_load(f) or {}
    out = args.out or DATA / "datasets" / config.get("name", args.config.stem)
    manifest = build(config, out, args.frames_index, args.labels_index, REPO)
    for name, c in manifest["splits"].items():
        print(f"{name:5s} {c['frames']:7d} frames {c['videos']:5d} videos  tiers {c['tier']}  status {c['status']}")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
