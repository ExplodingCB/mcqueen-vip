"""End to end on a synthetic dataset: render labels from the oval, paint images
from them, build the dataset, train the tiny model for a few epochs on the
CPU, export to ONNX with a parity check, and evaluate with the edge metric."""

import json

import numpy as np
import pytest
from PIL import Image

torch = pytest.importorskip("torch")

from mcq_training import dataset as D  # noqa: E402
from mcq_training import evaluate as E  # noqa: E402
from mcq_training import export as X  # noqa: E402
from mcq_training import labels as L  # noqa: E402
from mcq_training import train as T  # noqa: E402
from mcq_training.data import Augment, AugmentParams, SegDataset  # noqa: E402

SIZE = (160, 100)


def _synthetic(tmp_path, oval, cam, videos=4, per_video=4):
    small = cam.scaled(*SIZE)
    frames_root, labels_root = tmp_path / "frames", tmp_path / "labels"
    frames, labels = [], []
    rng = np.random.default_rng(0)
    k = 0
    for v in range(videos):
        for _ in range(per_video):
            x, y, psi = oval.cartesian(3.0 * k, 0.4 * np.sin(k))
            label = L.render_label(oval, (float(x[0]), float(y[0]), float(psi[0])), small, 0.1, 0.0)
            rgb = L.colorize(np.where(label == L.IGNORE, L.PAVEMENT, label))
            rgb = np.clip(rgb.astype(float) + rng.normal(0, 12, rgb.shape), 0, 255).astype(np.uint8)
            rel = f"kart_t/v{v}/{k:012d}.jpg"
            (frames_root / rel).parent.mkdir(parents=True, exist_ok=True)
            (labels_root / rel).parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(rgb).save(frames_root / rel, quality=90)
            L.save_label(labels_root / rel.replace(".jpg", ".png"), label)
            frames.append(
                {
                    "frame": rel,
                    "source": "kart_t",
                    "video_id": f"v{v}",
                    "t_ms": k,
                    "camera": "chassis_front",
                    "license": "own",
                    "track": "oval",
                    "dhash": "",
                }
            )
            labels.append(
                {
                    "frame": rel,
                    "label": rel.replace(".jpg", ".png"),
                    "source": "kart_t",
                    "video_id": f"v{v}",
                    "method": "projection",
                    "status": "auto",
                    "reviewer": "",
                    "date": "",
                    "note": "",
                }
            )
            k += 1
    D.append_rows(frames_root / "index.csv", D.FRAME_COLUMNS, frames)
    D.append_rows(labels_root / "index.csv", D.LABEL_COLUMNS, labels)
    config = {"name": "smoke", "val_fraction": 0.0, "val": ["v2"], "test": ["v3"]}
    D.build(config, tmp_path / "ds", frames_root / "index.csv", labels_root / "index.csv", tmp_path)
    return frames_root, labels_root, tmp_path / "ds"


def test_augment_keeps_shapes_and_ignore(tmp_path, oval, cam):
    frames_root, labels_root, ds = _synthetic(tmp_path, oval, cam, videos=1, per_video=2)
    samples = D.read_split(ds / "train.csv") + D.read_split(ds / "val.csv")
    data = SegDataset(samples, frames_root, labels_root, SIZE, Augment(AugmentParams(scale=(0.6, 0.7)), seed=1))
    x, y, _ = data[0]
    assert x.shape == (3, SIZE[1], SIZE[0]) and y.shape == (SIZE[1], SIZE[0])
    assert (y == L.IGNORE).any()  # padding from the downscale is ignored
    assert set(y.unique().tolist()) <= {0, 1, 2, 255}


def test_train_export_evaluate(tmp_path, oval, cam):
    frames_root, labels_root, ds = _synthetic(tmp_path, oval, cam)
    cam.save(tmp_path / "cam.yaml")
    config = {
        "model": {"name": "tiny_unet"},
        "input": {"width": SIZE[0], "height": SIZE[1]},
        "epochs": 12,
        "batch_size": 4,
        "lr": 0.005,
        "workers": 0,
        "amp": False,
        "augment": {"scale": [0.9, 1.1], "vshift": 0.03, "hshift": 0.03, "blur": 0.0, "grayscale": 0.0},
        "select": "pavement_iou",
        "eval": {"camera": str(tmp_path / "cam.yaml"), "gates": {"pavement_iou": 0.95, "boundary_error_15m": 0.3}},
    }
    out = tmp_path / "run"
    record = T.train(config, ds, out, frames_root, labels_root, torch.device("cpu"))
    assert (out / "best.pt").exists() and (out / "last.pt").exists() and (out / "metrics.json").exists()
    hist = record["history"]
    assert hist[-1]["train_loss"] < hist[0]["train_loss"]
    assert record["best"]["pavement_iou"] > 0.5, record["best"]

    again = T.train(
        {**config, "epochs": 1},
        ds,
        tmp_path / "run2",
        frames_root,
        labels_root,
        torch.device("cpu"),
        init=out / "best.pt",
    )
    assert again["init"] == str(out / "best.pt") and again["best"]["pavement_iou"] > 0.5

    info = X.export_onnx(out / "best.pt", out / "model.onnx", check=True)
    assert (out / "model.onnx").exists() and (out / "model.json").exists()
    assert info["check"]["argmax_agreement"] > 0.99 and info["check"]["max_abs_diff"] < 1e-3

    result = E.evaluate(out / "best.pt", ds / "test.csv", frames_root, labels_root, torch.device("cpu"))
    assert result["frames"] == 4 and "A" in result["pixels"]
    assert result["boundary"]["frames"] == 4
    assert "pavement_iou" in result["gates"] and "boundary_error_15m" in result["gates"]
    assert (
        E.main(
            [
                "--checkpoint",
                str(out / "best.pt"),
                "--split",
                str(ds / "test.csv"),
                "--frames-root",
                str(frames_root),
                "--labels-root",
                str(labels_root),
                "--device",
                "cpu",
            ]
        )
        == 0
    )
    assert json.loads((out / "eval_test.json").read_text())["frames"] == 4
