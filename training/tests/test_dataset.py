import json

from mcq_training import dataset as D


def _frames():
    rows = []
    for vid in ("v1", "v2", "v3", "v4"):
        for t in range(10):
            rows.append(
                {
                    "frame": f"yt/{vid}/{t:09d}.jpg",
                    "source": "yt",
                    "video_id": vid,
                    "t_ms": str(t * 500),
                    "camera": "helmet",
                    "license": "Creative Commons Attribution" if vid == "v1" else "standard",
                    "track": "",
                    "dhash": "0",
                }
            )
    for t in range(10):
        rows.append(
            {
                "frame": f"kart_s1/kart_s1/{t:09d}.jpg",
                "source": "kart_s1",
                "video_id": "kart_s1",
                "t_ms": str(t * 500),
                "camera": "chassis_front",
                "license": "own",
                "track": "purdue",
                "dhash": "0",
            }
        )
    return rows


def _labels(frames):
    labs = []
    for fr in frames:
        labs.append({**fr, "label": fr["frame"].replace(".jpg", ".png"), "method": "sam3", "status": "auto"})
    # One frame reviewed (wins over auto), one rejected (dropped), one frame unlabeled.
    labs.append({**labs[0], "label": "yt/v1/reviewed.png", "method": "manual", "status": "reviewed", "reviewer": "cc"})
    labs.append({**labs[1], "status": "rejected"})
    labs.pop(2)
    return labs


def test_tier_rules():
    assert D.tier_for({"source": "kart_x", "license": "own"}, {}) == "A"
    assert D.tier_for({"source": "yt", "license": "Creative Commons Attribution license"}, {}) == "B"
    assert D.tier_for({"source": "yt", "license": "standard"}, {}) == "C"
    assert D.tier_for({"source": "kart_chaser", "license": "standard"}, {"B": ["kart_chaser"]}) == "B"
    assert D.tier_for({"source": "x", "video_id": "abc", "license": ""}, {"A": ["ab*"]}) == "A"


def test_join_prefers_reviewed_and_drops_rejected():
    frames = _frames()
    rows = D.join(frames, _labels(frames))
    by_frame = {r["frame"]: r for r in rows}
    assert by_frame["yt/v1/000000000.jpg"]["label"] == "yt/v1/reviewed.png"
    assert "yt/v1/000000001.jpg" not in by_frame
    assert "yt/v1/000000002.jpg" not in by_frame
    assert len(rows) == len(frames) - 2


def test_split_by_video_is_leak_free_and_stable(tmp_path):
    frames = _frames()
    labels = _labels(frames)
    D.append_rows(tmp_path / "frames.csv", D.FRAME_COLUMNS, frames)
    D.append_rows(tmp_path / "labels.csv", D.LABEL_COLUMNS, labels)
    config = {"name": "t", "val_fraction": 0.5, "val": ["v4"], "test": ["kart_*"], "weights": {"A": 4, "B": 2, "C": 1}}
    m = D.build(config, tmp_path / "ds", tmp_path / "frames.csv", tmp_path / "labels.csv", tmp_path)
    splits = {n: D.read_split(tmp_path / "ds" / f"{n}.csv") for n in ("train", "val", "test")}
    assert {s.video_id for s in splits["test"]} == {"kart_s1"}
    assert all(s.tier == "A" and s.weight == 4.0 for s in splits["test"])
    assert not ({s.video_id for s in splits["train"]} & {s.video_id for s in splits["val"]})
    assert "v4" in {s.video_id for s in splits["val"]}
    assert len(splits["train"]) + len(splits["val"]) == 38
    m2 = D.build(config, tmp_path / "ds2", tmp_path / "frames.csv", tmp_path / "labels.csv", tmp_path)
    assert m["splits"]["train"]["sha256"] == m2["splits"]["train"]["sha256"]
    manifest = json.loads((tmp_path / "ds" / "manifest.json").read_text())
    assert manifest["splits"]["test"]["tier"] == {"A": 10}
    assert set(manifest["splits"]) == {"train", "val", "test"}
