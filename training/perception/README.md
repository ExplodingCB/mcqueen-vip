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
