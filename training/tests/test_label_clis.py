import numpy as np
from PIL import Image

from mcq_training import autolabel as A
from mcq_training import project_labels as P
from mcq_training.dataset import FRAME_COLUMNS, append_rows, read_rows
from mcq_training.extract_mcap import POSE_COLUMNS
from mcq_training.labels import BACKGROUND, IGNORE, KART, PAVEMENT, load_label


def _frames(tmp_path, oval, n=5, size=(320, 200)):
    root = tmp_path / "frames"
    fdir = root / "kart_t" / "sess"
    fdir.mkdir(parents=True)
    poses, index = [], []
    for i in range(n):
        x, y, psi = oval.cartesian(10.0 + 4.0 * i, 0.0)
        rel = f"kart_t/sess/{i:012d}.jpg"
        Image.fromarray(np.full((size[1], size[0], 3), 90, dtype=np.uint8)).save(root / rel)
        poses.append(
            {
                "frame": rel,
                "t_ns": i * 10**9,
                "x": float(x[0]),
                "y": float(y[0]),
                "yaw": float(psi[0]),
                "v": 4.0,
                "gnss_status": 2 if i < n - 1 else 1,
            }
        )
        index.append(
            {
                "frame": rel,
                "source": "kart_t",
                "video_id": "sess",
                "t_ms": i * 1000,
                "camera": "chassis_front",
                "license": "own",
                "track": "oval",
                "dhash": "",
            }
        )
    append_rows(fdir / "poses.csv", POSE_COLUMNS, poses)
    append_rows(root / "index.csv", FRAME_COLUMNS, index)
    return root, fdir


def test_project_labels_cli(tmp_path, oval, cam):
    root, fdir = _frames(tmp_path, oval)
    (tmp_path / "tracks").mkdir()
    oval.save(tmp_path / "tracks" / "oval")
    cam.save(tmp_path / "cam.yaml")
    labels = tmp_path / "labels"
    rc = P.main(
        [
            "--track", str(tmp_path / "tracks" / "oval"),
            "--frames-dir", str(fdir),
            "--camera", str(tmp_path / "cam.yaml"),
            "--labels-root", str(labels),
            "--preview", "2",
        ]
    )  # fmt: skip
    assert rc == 0
    rows = read_rows(labels / "index.csv")
    assert len(rows) == 4  # the frame without an RTK fix is skipped
    assert all(r["method"] == "projection" and r["status"] == "auto" for r in rows)
    assert rows[0]["source"] == "kart_t" and rows[0]["video_id"] == "sess"
    lab = load_label(labels / rows[0]["label"])
    assert lab.shape == (200, 320)  # camera scaled to the frame size
    assert set(np.unique(lab)) <= {BACKGROUND, PAVEMENT, IGNORE}
    assert lab[-1, 160] == PAVEMENT
    assert len(list((labels / "preview" / "kart_t" / "sess").glob("*.jpg"))) == 2


def test_autolabel_with_dummy_backend(tmp_path, oval):
    root, _ = _frames(tmp_path, oval, n=3)
    labels = tmp_path / "labels"
    rc = A.main(
        [
            "--frames-index", str(root / "index.csv"),
            "--labels-root", str(labels),
            "--backend", "dummy",
            "--prompts", str(tmp_path / "missing.yaml"),
            "--limit", "2",
        ]
    )  # fmt: skip
    assert rc == 0
    rows = read_rows(labels / "index.csv")
    assert len(rows) == 2 and all(r["status"] == "auto" and r["method"] == "dummy" for r in rows)
    lab = load_label(labels / rows[0]["label"])
    assert lab[-1, 0] == PAVEMENT and lab[0, 0] == BACKGROUND
    assert lab[int(0.6 * 200), 160] == KART  # kart painted over the pavement
    # A rerun skips what exists.
    rc = A.main(["--frames-index", str(root / "index.csv"), "--labels-root", str(labels), "--backend", "dummy"])
    assert rc == 0 and len(read_rows(labels / "index.csv")) == 3


def test_mask_conversion():
    assert A.to_bool_masks(np.array([[0.2, 0.9]])).tolist() == [[[False, True]]]
    assert A.to_bool_masks(np.array([[[-1.0, 2.0]]])).tolist() == [[[False, True]]]
    assert A.to_bool_masks(np.zeros((2, 1, 3, 3), dtype=bool)).shape == (2, 3, 3)
