#!/usr/bin/env python3
"""Expand the YouTube entries in sources.yaml into candidate videos, filter them,
fetch metadata (including the license field) and download what passes.

Typical use:
    python fetch_youtube.py --dry-run                # list candidates, print hours per source
    python fetch_youtube.py --cc-only                # download Creative Commons videos only (default)
    python fetch_youtube.py --sources purdue_gp_onboard_search --allow-standard-license
    python fetch_youtube.py --stats                  # summarize the manifest

Nothing is downloaded from a source whose permission field is not "granted"
unless the video carries a Creative Commons Attribution license. See
docs/08-training-data.md section 4.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import quote_plus

import yaml

HERE = Path(__file__).resolve().parent
CC_MARKER = "creative commons"


@dataclass
class Source:
    id: str
    kind: str
    camera: str = "mixed"
    url: str | None = None
    query: str | None = None
    max_results: int = 200
    track: str | None = None
    permission: str = "none"
    min_seconds: int = 120
    max_seconds: int = 7200
    title_include: list[str] = field(default_factory=list)
    title_exclude: list[str] = field(default_factory=list)
    crop: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    notes: str = ""

    def target(self, sort: str = "relevance", cc_filter: bool = False) -> tuple[str, list[str]]:
        """Return (url_or_query, extra yt-dlp args) for the listing step."""
        if self.kind != "youtube_search":
            if self.url is None:
                raise ValueError(f"source {self.id} needs a url")
            return self.url, []
        if sort == "relevance" and not cc_filter:
            # yt-dlp's built-in search extractor (verified against yt-dlp 2026.08); relevance order
            return f"ytsearch{self.max_results}:{self.query}", []
        # YouTube results page with its filter parameter, read by yt-dlp's youtube:search_url extractor.
        # sp=CAI%3D is "sort by upload date"; sp=EgIwAQ%3D%3D is the Creative Commons feature filter;
        # sp=CAISAjAB combines both (protobuf: sort=2, features=cc).
        sp = {("date", False): "CAI%253D", ("relevance", True): "EgIwAQ%253D%253D", ("date", True): "CAISAjAB"}[(sort, cc_filter)]
        return (f"https://www.youtube.com/results?search_query={quote_plus(self.query)}&sp={sp}",
                ["--playlist-end", str(self.max_results)])


def load_sources(path: Path) -> list[Source]:
    doc = yaml.safe_load(path.read_text())
    defaults = doc.get("defaults", {})
    sources = []
    for entry in doc.get("youtube", []):
        merged = {**defaults, **entry}
        merged["title_exclude"] = list(defaults.get("title_exclude", [])) + list(entry.get("title_exclude", []))
        sources.append(Source(**merged))
    return sources


# ---------------------------------------------------------------- filtering (pure functions, unit tested)

def passes_duration(duration: float | None, src: Source) -> bool:
    if duration is None:
        return True  # flat listings sometimes lack it; metadata step re-checks
    return src.min_seconds <= duration <= src.max_seconds


def passes_title(title: str | None, src: Source) -> bool:
    t = (title or "").lower()
    if src.title_include and not any(k.lower() in t for k in src.title_include):
        return False
    return not any(k.lower() in t for k in src.title_exclude)


def is_cc(license_str: str | None) -> bool:
    return bool(license_str) and CC_MARKER in license_str.lower()


def allowed_to_download(license_str: str | None, src: Source, cc_only: bool) -> bool:
    if is_cc(license_str):
        return True
    if cc_only:
        return False
    return src.permission == "granted"


def candidate_ok(entry: dict, src: Source) -> bool:
    return passes_duration(entry.get("duration"), src) and passes_title(entry.get("title"), src)


# ---------------------------------------------------------------- yt-dlp wrappers

def ytdlp() -> str:
    exe = shutil.which("yt-dlp")
    if exe is None:
        sys.exit("yt-dlp not found; pip install -r requirements.txt")
    return exe


def list_candidates(src: Source, sleep: float, sort: str = "relevance", cc_filter: bool = False) -> list[dict]:
    target, extra = src.target(sort, cc_filter)
    cmd = [ytdlp(), "--flat-playlist", "-j", "--ignore-errors", "--sleep-requests", str(sleep), *extra, target]
    out = subprocess.run(cmd, capture_output=True, text=True)
    entries = []
    for line in out.stdout.splitlines():
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if e.get("id"):
            entries.append({"id": e["id"], "title": e.get("title"), "duration": e.get("duration"),
                            "uploader": e.get("uploader") or e.get("channel"), "url": e.get("url") or e.get("webpage_url")})
    if not entries and out.returncode != 0:
        err = out.stderr.strip().splitlines()
        print(f"  warning: yt-dlp returned nothing for {src.id}: {err[-1] if err else 'no error text'}")
    return entries


def fetch_metadata(video_id: str, sleep: float) -> dict | None:
    cmd = [ytdlp(), "-j", "--skip-download", "--sleep-requests", str(sleep), f"https://www.youtube.com/watch?v={video_id}"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0 or not out.stdout.strip():
        return None
    m = json.loads(out.stdout.splitlines()[-1])
    return {"id": m.get("id"), "title": m.get("title"), "duration": m.get("duration"), "uploader": m.get("uploader"),
            "channel_id": m.get("channel_id"), "upload_date": m.get("upload_date"), "license": m.get("license"),
            "width": m.get("width"), "height": m.get("height"), "fps": m.get("fps"),
            "url": m.get("webpage_url") or f"https://www.youtube.com/watch?v={video_id}"}


def download(video_id: str, dest: Path, max_height: int, sleep: float) -> Path | None:
    dest.mkdir(parents=True, exist_ok=True)
    fmt = f"bv*[height<={max_height}][ext=mp4]+ba[ext=m4a]/b[height<={max_height}]/b"
    cmd = [ytdlp(), "-f", fmt, "--merge-output-format", "mp4", "--write-info-json", "--no-overwrites",
           "--sleep-requests", str(sleep), "-o", str(dest / "%(id)s.%(ext)s"), f"https://www.youtube.com/watch?v={video_id}"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0:
        print(f"  download failed for {video_id}: {out.stderr.strip().splitlines()[-1] if out.stderr.strip() else 'unknown error'}")
        return None
    files = sorted(dest.glob(f"{video_id}.mp4")) or sorted(dest.glob(f"{video_id}.*[!n]"))
    return files[0] if files else None


# ---------------------------------------------------------------- manifest

def read_manifest(path: Path) -> dict[str, dict]:
    rows = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                rows[r["id"]] = r
    return rows


def append_manifest(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")


def hours(seconds: float) -> str:
    return f"{seconds / 3600:.1f} h"


def print_stats(manifest: dict[str, dict]) -> None:
    by_source: dict[str, float] = defaultdict(float)
    by_license: dict[str, float] = defaultdict(float)
    by_camera: dict[str, float] = defaultdict(float)
    for r in manifest.values():
        d = r.get("duration") or 0
        by_source[r.get("source", "?")] += d
        by_license["cc-by" if is_cc(r.get("license")) else "standard"] += d
        by_camera[r.get("camera", "?")] += d
    total = sum(by_source.values())
    print(f"{len(manifest)} videos, {hours(total)} total")
    for name, table in (("by source", by_source), ("by license", by_license), ("by camera", by_camera)):
        print(f"\n{name}:")
        for k, v in sorted(table.items(), key=lambda kv: -kv[1]):
            print(f"  {k:36s} {hours(v)}")


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sources-file", type=Path, default=HERE / "sources.yaml")
    ap.add_argument("--out", type=Path, default=HERE / "raw" / "youtube", help="download root")
    ap.add_argument("--manifest", type=Path, default=HERE / "manifest.jsonl")
    ap.add_argument("--sources", help="comma-separated source ids to process (default: all)")
    ap.add_argument("--dry-run", action="store_true", help="list and filter candidates, download nothing")
    ap.add_argument("--metadata-only", action="store_true", help="fetch per-video metadata (license) but do not download")
    ap.add_argument("--cc-only", dest="cc_only", action="store_true", default=True,
                    help="download only Creative Commons videos (default)")
    ap.add_argument("--allow-standard-license", dest="cc_only", action="store_false",
                    help="also download standard-license videos from sources whose permission is 'granted'")
    ap.add_argument("--max-hours", type=float, default=None, help="stop after this many new hours in this run")
    ap.add_argument("--max-height", type=int, default=1080)
    ap.add_argument("--sleep", type=float, default=1.0, help="seconds between yt-dlp requests")
    ap.add_argument("--search-sort", choices=["relevance", "date"], default="relevance",
                    help="ordering for youtube_search sources; 'date' is useful for reruns that pick up new uploads")
    ap.add_argument("--search-cc", action="store_true",
                    help="ask YouTube search for Creative Commons results only (fewer metadata calls; hides the full picture in --dry-run)")
    ap.add_argument("--stats", action="store_true", help="print manifest summary and exit")
    args = ap.parse_args()

    manifest = read_manifest(args.manifest)
    if args.stats:
        print_stats(manifest)
        return 0

    sources = load_sources(args.sources_file)
    if args.sources:
        wanted = set(args.sources.split(","))
        sources = [s for s in sources if s.id in wanted]
        missing = wanted - {s.id for s in sources}
        if missing:
            sys.exit(f"unknown source ids: {sorted(missing)}")

    seen: set[str] = set(manifest)
    new_seconds = 0.0
    for src in sources:
        cands = list_candidates(src, args.sleep, args.search_sort, args.search_cc)
        kept = [c for c in cands if candidate_ok(c, src) and c["id"] not in seen]
        kept_seconds = sum(c.get("duration") or 0 for c in kept)
        print(f"[{src.id}] {len(cands)} listed, {len(kept)} pass filters, {hours(kept_seconds)} (camera={src.camera}, permission={src.permission})")
        if args.dry_run:
            continue
        for c in kept:
            if args.max_hours is not None and new_seconds >= args.max_hours * 3600:
                print("max-hours reached")
                print_stats(read_manifest(args.manifest))
                return 0
            meta = fetch_metadata(c["id"], args.sleep)
            if meta is None or not passes_duration(meta.get("duration"), src):
                continue
            row = {**meta, "source": src.id, "camera": src.camera, "track": src.track, "crop": src.crop,
                   "permission": src.permission, "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "path": None}
            if not allowed_to_download(meta.get("license"), src, args.cc_only):
                row["status"] = "skipped_license"
            elif args.metadata_only:
                row["status"] = "metadata_only"
            else:
                path = download(c["id"], args.out / src.id / c["id"], args.max_height, args.sleep)
                row["status"] = "downloaded" if path else "download_failed"
                row["path"] = str(path.relative_to(HERE)) if path else None
                if path:
                    new_seconds += meta.get("duration") or 0
            append_manifest(args.manifest, row)
            seen.add(c["id"])
            print(f"  {row['status']:16s} {c['id']} {hours(meta.get('duration') or 0):>7s} {meta.get('license') or 'standard'} | {meta.get('title')}")
    if not args.dry_run:
        print()
        print_stats(read_manifest(args.manifest))
    return 0


if __name__ == "__main__":
    sys.exit(main())
