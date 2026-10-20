# training/perception

The first perception model: RGB in, pavement mask out, trained on synthetic frames from random tracks and scored on tracks it has never seen. It is the segmenter in the chain camera, mask, edges, planner, controllers (docs/02 section 7, [the simulator guide](../../docs/10-simulator.md)). Real footage is what it will finally be judged on and is not here yet; see [what this does and does not show](#what-this-does-and-does-not-show).

```sh
uv pip install --python .venv/bin/python torch          # the only dependency beyond the simulator's
export PYTHONPATH=training/perception:src/mcq_sim
.venv/bin/python training/perception/train.py --run v0  # about 16 minutes on an Apple-silicon laptop
.venv/bin/python training/perception/evaluate.py --weights training/perception/runs/v0/model.pt --out logs/perception-v0.json
.venv/bin/python -m mcq_sim evaluate --policy stack --perception segmenter:create --out logs/stack-v0
```

Weights and run logs go to `runs/`, which is git-ignored. `runs/latest` points at the last run and is what `segmenter:create` loads; set `MCQ_SEGMENTER_WEIGHTS` to use another file.

| File | What it is |
| --- | --- |
| `synth.py` | Random closed tracks, kart poses, and a renderer with randomized surfaces, shadows, edge paint, exposure, white balance, haze, sky, camera geometry, noise and blur. Labels come from the simulator camera's own ground-plane geometry, so they are exact |
| `model.py` | `PavementNet`, a 1.6 M parameter encoder-decoder with a dilated bottleneck. Input 240 x 135, one logit per pixel |
| `train.py` | Generates fresh frames every epoch from new random tracks, trains with BCE plus Dice, validates on tracks it never trained on |
| `segmenter.py` | `create()` returns the trained model behind the simulator's `predict(rgb)` contract: 270 x 480 uint8 in, pavement probability out |
| `evaluate.py` | Scores a segmenter on the Purdue track and the synthetic oval, neither of which is ever trained on |

## What is measured

`evaluate.py` reports, per held-out track and per frame source, mean pavement IoU, the lateral error of the left and right edge about 15 m ahead after ground-plane projection (the docs/08 section 7 acceptance quantity, under 0.3 m), and how often the model recovers both edges where the truth has them. The color-threshold demo segmenter is scored on the same frames as a floor. Frame sources are the randomized appearance the model trains on, rendered through the kart's real 480 x 270 camera geometry, and the simulator's own flat camera, which is the image the closed loop shows the model.

## What this does and does not show

It shows that the pipeline works end to end and that a small network can learn to segment pavement by shape and context under wide appearance variation, on layouts it has not seen. It is also useful pretraining.

It does not show that the model works on a real photograph. The appearance randomization is a guess at what real cameras vary, made by the person who wrote the renderer, and every frame comes from a flat ground plane with no kart, no other vehicles, no curbs with height, no slopes and no lens effects. The acceptance test in docs/08 section 7 is held-out real footage from the Purdue track. Until that exists, a high number here is a statement about this generator.

## Next, in order

1. Tier A and B footage through `training/data`, auto-labeled and reviewed (docs/08 section 5), used to fine-tune this model and to score it.
2. Fine-tune on real frames, keeping the synthetic set as a regularizer.
3. Export to ONNX and TensorRT and measure on the Orin against the 20 fps budget.

## The driving model (2026-10)

The segmenter above grew into a model that outputs a pavement mask, a racing line (12 waypoints, 2 to 24 m ahead, in the kart's frame) and a target speed at each. The stack drives that line through the controllers; the model never outputs steering or throttle. Version-by-version results, what each one measurably did and what none of it shows are in [docs/13](../../docs/13-driving-model-versions.md). The short version: drive-v5, camera only, laps the simulated Purdue track in 64.6 s with 0.39 m to the edge, and its pavement mask scores 0.84 IoU on held-out real kart footage (against auto-labels). Nothing here has seen the kart's own camera or driven a real kart. The trained weights are at https://huggingface.co/ExplodingCB2/mcqueen-driving-model (built and uploaded by `tools/hf/upload_model.py`).

| File | What it is |
| --- | --- |
| `drive_data.py` | Camera frames on the orthophoto-draped track, labeled with the expert's racing line and speed |
| `model.py` | `PavementNet` (mask) and `DriveNet` (mask, line and speed) |
| `train_drive.py` | Trains the driving model: simulator frames, DAgger frames, and real footage for the mask. Prints held-out real-footage IoU each epoch |
| `dagger.py` | Collects frames from the states the model actually reaches, labeled by the expert from where the kart really is |
| `laptime_opt.py` | Searches for a faster expert line and speed profile by simulated lap time (CMA-ES), scored across a normal kart, a sloppy kart and servo, and the real-width pavement |
| `race_eval.py` | Closed-loop laps, per model, speed cap and seed |
| `robustness.py` | The same lap with the pavement, camera mount, image, servo, brakes, grip or GPS made worse |
| `real_eval.py` | Scores a model's pavement mask on real footage from `training/data` against its auto-labels |
| `driver.py` | The trained driver behind the simulator's camera-model contract (`--perception driver:create`) |
| `watch.py` | Local training dashboard: loss and validation curves, the lap-time search, and a gallery of lap plots |

Real footage comes from `training/data` (see [docs/08](../../docs/08-training-data.md)): `fetch_youtube.py`, `extract_frames.py` and `autolabel.py`, which labels pavement with SegFormer. The auto-labels are pseudo-labels with no human review. The 3D demo and the pre-show check are in `tools/sim_video/` and [docs/15](../../docs/15-demo-runbook.md).

