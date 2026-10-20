---
library_name: pytorch
pipeline_tag: image-segmentation
tags:
  - autonomous-driving
  - go-kart
  - racing-line
  - semantic-segmentation
  - imitation-learning
  - robotics
---

# McQueen driving model (drive-v5)

A small camera-only driving model for an autonomous go-kart on the Purdue Grand Prix track. One RGB frame goes in; three things come out: a **pavement mask**, a **racing line** (12 waypoints, 2 to 24 m ahead, in the kart's frame) and a **target speed** at each waypoint. It does not output steering or throttle. Controllers outside the model turn the line into commands from measured state, and a safety layer keeps the line inside the pavement the camera can see.

> **Status: a simulator result, not a road-tested model.** It has never seen the kart's own camera and has never driven a real kart. See [What this does not show](#what-this-does-not-show).

Code, training scripts, the simulator and the full version history are in the project repository: https://github.com/ExplodingCB/mcqueen-vip (branch `training-perception`, see `docs/13-driving-model-versions.md`).

## Files

| File | What it is |
| --- | --- |
| `drive-v5.pt` | **The recommended model.** Real-footage mask training and a clean simulated lap |
| `checkpoints/drive-v3.pt` | Best simulated lap time (63.8 s); weaker on real footage |
| `checkpoints/drive-v4.pt` | drive-v3 plus real helmet-cam footage for the mask |
| `checkpoints/drive-v6.pt` | Trained on a faster teacher line. Best open-loop numbers, but **does not complete laps**; included as a research result |
| `model.py`, `driver.py`, `example.py` | Everything needed to load a checkpoint and run it on an image |
| `laptime-v2-best.json` | The searched expert line (racing line offsets and speed limits) the faster teacher uses |

{{CHECKSUMS}}

## Use

```python
# pip install torch numpy pillow
import numpy as np
from PIL import Image
from driver import Driver  # driver.py and model.py sit beside it

driver = Driver("drive-v5.pt")  # CPU by default
frame = np.asarray(Image.open("frame.jpg").convert("RGB").resize((480, 270)))  # uint8, 270 x 480 x 3
mask, line, speed = driver.predict_drive(frame)
# mask : (270, 480) pavement probability
# line : (12, 2) metres in the kart frame, x forward and y left, from 2 m to 24 m ahead
# speed: (12,)   metres per second the model wants at each waypoint
```

The model assumes a camera about 0.7 m above the ground, pitched about 8 degrees down, with a 100 degree horizontal field of view. A different mount shifts the line, because it is in metres. `python example.py frame.jpg` prints the same thing.

## Model

`DriveNet`: a 3.9 M parameter encoder-decoder (a dilated bottleneck, all standard convolutions) with a mask head, plus a driving head that reads the bottleneck and the quarter-resolution decoder features (with pixel-coordinate channels) and regresses the line and speeds. Input is the 240 x 135 half-resolution frame. About 7 ms per frame on one CPU thread.

## Training data

* **Simulator frames:** the Purdue track with the 2023 Indiana orthophoto draped over a flat ground plane (Indiana's imagery service, marked CC0), camera mount and image appearance randomized, labeled with an expert's minimum-curvature racing line and the speed along it.
* **DAgger frames:** frames from states the model itself reaches in closed loop (with steering bumps), labeled by the expert from where the kart actually is.
* **Real kart footage, for the mask only:** about 2,300 frames from about 30 YouTube kart onboard videos (helmet cams and nose or bumper cams), pavement labeled automatically by SegFormer (ADE20K classes road, runway, sidewalk, path). **These are pseudo-labels with no human review.** The videos are third-party, so check their terms before redistributing or using this commercially.

## Results (all in the simulator unless stated)

| Measure | drive-v5 |
| --- | --- |
| Closed-loop lap, camera only, default limits, no speed cap, 3 seeds | **64.6 s**, 0.39 m closest to an edge |
| Racing-line error on unseen simulator frames, 2 to 10 m / 12 to 24 m ahead | 0.27 m / 0.54 m |
| Target-speed error | 0.34 m/s |
| Pavement mask IoU on unseen simulator frames | 0.979 |
| **Pavement mask IoU on 575 held-out real kart frames, against auto-labels** | **0.84** |

For reference: a simulator-only model scores 0.39 on the real frames, below a fixed "the lower part of the picture is road" guess (0.47). Models trained on the faster teacher (drive-v6) fail to complete laps.

## What this does not show

* No version has seen the kart's own camera or driven a real kart. The held-out real frames are other people's cameras on other tracks, scored against automatic labels.
* The racing line and speed were trained only on simulator frames. The mask is the only output that has been trained on real images, so how the line and speed behave on real frames is untested.
* The track file is hand-traced from aerial imagery, and the simulated tyres, brakes and steering servo are placeholders. A 64.6 s simulated lap is not a prediction of the real kart.
* The driving path (safety filter, corridor tracker) exists only in the Python simulator and is not yet ported to the kart's software or timed on the Jetson.

The intended use today is research, the simulator, and **shadow mode** (a person drives while the model only watches).
