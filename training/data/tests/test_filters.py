import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import extract_frames as ef  # noqa: E402
import fetch_youtube as fy  # noqa: E402


def src(**kw):
    base = dict(id="t", kind="youtube_search", query="kart onboard", camera="helmet")
    base.update(kw)
    return fy.Source(**base)


def test_duration_filter():
    s = src(min_seconds=120, max_seconds=600)
    assert fy.passes_duration(300, s)
    assert not fy.passes_duration(30, s)
    assert not fy.passes_duration(6000, s)
    assert fy.passes_duration(None, s)


def test_title_filters():
    s = src(title_include=["onboard"], title_exclude=["crash compilation"])
    assert fy.passes_title("KA100 Onboard at New Castle", s)
    assert not fy.passes_title("Kart crash compilation onboard", s)
    assert not fy.passes_title("Pit walk vlog", s)
    assert fy.passes_title(None, src())


def test_license_gate():
    cc = "Creative Commons Attribution license (reuse allowed)"
    assert fy.is_cc(cc) and not fy.is_cc(None) and not fy.is_cc("Standard YouTube License")
    assert fy.allowed_to_download(cc, src(permission="none"), cc_only=True)
    assert not fy.allowed_to_download(None, src(permission="none"), cc_only=True)
    assert not fy.allowed_to_download(None, src(permission="none"), cc_only=False)
    assert fy.allowed_to_download(None, src(permission="granted"), cc_only=False)
    assert not fy.allowed_to_download(None, src(permission="granted"), cc_only=True)


def test_sources_yaml_loads_and_search_target():
    sources = fy.load_sources(Path(__file__).resolve().parents[1] / "sources.yaml")
    ids = [s.id for s in sources]
    assert len(ids) == len(set(ids)), "duplicate source ids"
    s = next(x for x in sources if x.id == "q_kart_onboard")
    assert s.target() == ("ytsearch300:kart onboard", [])
    url, extra = s.target(sort="date")
    assert url.startswith("https://www.youtube.com/results?search_query=kart+onboard&sp=CAI") and extra == ["--playlist-end", "300"]
    assert "sp=CAISAjAB" in s.target(sort="date", cc_filter=True)[0]
    assert "sp=EgIwAQ" in s.target(cc_filter=True)[0]
    ch = next(x for x in sources if x.kind == "youtube_channel")
    assert ch.target() == (ch.url, [])
    assert "crash compilation" in s.title_exclude  # defaults merged in
    assert all(x.camera in {"helmet", "chassis_front", "chassis_rear", "trackside", "mixed"} for x in sources)
    assert all(x.permission in {"none", "requested", "granted"} for x in sources)


def test_crop_filter():
    assert ef.crop_filter([0, 0, 0, 0]) is None
    assert ef.crop_filter([0.0, 0.12, 0.0, 0.0]) == "crop=iw*1.0000:ih*0.8800:iw*0.0000:ih*0.0000"


def test_extract_on_synthetic_video(tmp_path):
    exe = ef.ffmpeg_exe()
    video = tmp_path / "synthetic.mp4"
    # 12 s clip: 6 s of a static frame then 6 s of a moving test pattern
    subprocess.run([exe, "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i", "color=c=gray:s=640x360:d=6",
                    "-f", "lavfi", "-i", "testsrc=s=640x360:d=6", "-filter_complex", "[0:v][1:v]concat=n=2:v=1:a=0",
                    "-pix_fmt", "yuv420p", str(video)], check=True)
    out = tmp_path / "frames"
    subprocess.run([sys.executable, str(Path(ef.__file__)), "--videos", str(video), "--out", str(out), "--fps", "2",
                    "--skip-start", "0", "--skip-end", "0", "--long-edge", "320"], check=True)
    rows = (out / "index.csv").read_text().splitlines()
    kept = len(rows) - 1
    assert 1 <= kept < 24, f"expected duplicates of the static half to be dropped, kept {kept}"
    assert (out / "manual" / "synthetic").exists()
