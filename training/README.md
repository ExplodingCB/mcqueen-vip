# training

Off-kart training for the perception model: label projection, auto-labeling, dataset assembly, training, evaluation and ONNX export. The design and the reasoning are in [docs/09-training-pipeline.md](../docs/09-training-pipeline.md); the source survey and the legal position in [docs/08-training-data.md](../docs/08-training-data.md). Footage ingest (YouTube fetch, frame extraction) is in [data/](data/README.md).

```
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # or the cu130 index on a GPU box
pip install -r training/requirements.txt
python -m pytest -q training/tests
```

Every command below runs from inside `training/` (`cd training`), which puts `mcq_training` on the path and makes `configs/`, `data/` and `runs/` resolve; from the repository root, set `PYTHONPATH=training` and prefix those paths with `training/`.

| Stage | Command | Reads | Writes |
| --- | --- | --- | --- |
| Tier A frames | `python -m mcq_training.extract_mcap <session.mcap> --track purdue --fps 5` | camera and `/ego_state` topics | `data/frames/kart_<session>/…/*.jpg`, `poses.csv`, rows in `data/frames/index.csv` |
| Tier A labels | `python -m mcq_training.project_labels --track tracks/purdue --frames-dir data/frames/kart_<session>/<session> --camera configs/camera_default.yaml --preview 20` | survey, poses, camera model | `data/labels/…/*.png`, rows in `data/labels/index.csv`, previews |
| Tier B and C frames | `python training/data/fetch_youtube.py` then `python training/data/extract_frames.py` | `data/sources.yaml` | `data/raw/`, `data/frames/`, `data/frames/index.csv` |
| Tier B and C labels | `python -m mcq_training.autolabel --sources q_kart_chassis_cam --limit 2000` | frames, `configs/autolabel.yaml`, SAM 3 | `data/labels/…/*.png`, rows with `status=auto` |
| Review | CVAT or Label Studio; the export sets `status` to `reviewed` or `rejected` in `data/labels/index.csv` | | |
| Dataset | `python -m mcq_training.dataset --config configs/dataset_example.yaml` | both index files | `data/datasets/<name>/{train,val,test}.csv`, `manifest.json` |
| Train | `python -m mcq_training.train --config configs/seg_small.yaml --dataset data/datasets/seg_v1 --out runs/seg_v1_a` | splits, frames, labels | `best.pt`, `last.pt`, `metrics.json`, config and manifest copies |
| Fine-tune | `python -m mcq_training.train … --init runs/seg_v1_a/best.pt --dataset data/datasets/seg_v1_ab --out runs/seg_v1_b` | | |
| Evaluate | `python -m mcq_training.evaluate --checkpoint runs/seg_v1_b/best.pt --split data/datasets/seg_v1_ab/test.csv` | held-out tier A | `eval_test.json`: IoU per class and tier, edge error in metres by range, gate pass or fail |
| Export | `python -m mcq_training.export --checkpoint runs/seg_v1_b/best.pt` | | `model.onnx`, `model.json`; then `trtexec --onnx=model.onnx --saveEngine=model.engine --fp16` on the Jetson |

`data/` holds pixels and is git-ignored except for `sources.yaml` and the ingest scripts; `runs/` is git-ignored. What goes into git is under `configs/` (camera, prompts, dataset recipe, training recipe), the metrics and manifests copied into a pull request when a model is promoted, and the code.

| Module (`mcq_training/`) | What it is |
| --- | --- |
| `camera.py` | Pinhole camera between `base_link` and the rectified image; projection and inverse perspective mapping |
| `labels.py` | Class convention (0 background, 1 pavement, 2 kart, 255 ignore), PNG I/O, the survey-to-image projector with the ignore band |
| `extract_mcap.py`, `rosmsg.py` | Frames plus interpolated poses out of a session log without ROS; message definitions assembled from `src/mcq_msgs` |
| `project_labels.py` | Tier A labeling CLI with preview overlays |
| `autolabel.py` | SAM 3 text-prompted labeling for tier B and C, plus a dummy backend for the pipeline test |
| `dataset.py` | Join frames and labels, tiers, weights, leak-free splits by video, manifest |
| `data.py`, `models.py` | PyTorch dataset and augmentation; model registry (tiny U-Net for tests, LR-ASPP MobileNetV3, DeepLabV3 MobileNetV3) |
| `train.py`, `evaluate.py`, `export.py` | Training loop, evaluation with the edge metric and gates, ONNX export with an onnxruntime parity check |
| `boundary.py`, `metrics.py` | Edge extraction and inverse perspective mapping (the reference for the C++ node), IoU and edge error accumulators |
