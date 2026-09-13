# Training footage: where the hours are and how we get them

2026-09-13: the pipeline of section 5 exists in `training/mcq_training/` and is described in [09-training-pipeline.md](09-training-pipeline.md). Two changes to the plan below. Step 2 uses SAM 3 (Meta, November 2025: text-prompted concept segmentation; SAM 3.1 of March 2026 made its multi-object video tracking several times faster) instead of Grounded SAM 2. Step 5, projecting the surveyed edges into our own frames, is implemented and tested against the synthetic oval, so tier A labels cost nothing from the first logged lap with a calibrated camera; the practical consequence is that the camera should be mounted and calibrated in Phase 1 with the other sensors, not in Phase 4.

The perception model in `mcq_perception` needs front-camera footage of kart tracks, labeled for pavement versus not-pavement and for other karts. This document is the result of a source hunt done on 2026-09-11 and the plan that follows from it. The tooling that implements the plan lives in `training/data/`.

## 1. The short version

There is no open dataset of onboard kart footage measured in hours. The kart-specific material that is public and clean amounts to a few thousand images. The hours exist in two places: on YouTube, where onboard karting is a genre with thousands of hours across helmet-cam channels, series broadcasts and individual racers, and on our own kart, where every practice lap at the Purdue track is both legally unambiguous and exactly the camera placement the model will see. The plan is three tiers, in this priority:

| Tier | Source | Target | Camera match | Legal position |
| --- | --- | --- | --- | --- |
| A | Our kart at the Purdue track and any paved loop we can access | 5 h or more across days, lighting and weather, growing every session | Exact | Ours; the AKS rules allow all practice-session data |
| B | Footage we have written permission for, or that is Creative Commons: Purdue Grand Prix teams and the Foundation, AKS teams, Kart Chaser, individual POV channels who say yes | 20 h or more | Close (kart-mounted) to fair (helmet) | Permission on file or CC BY attribution |
| C | Auto-labeled pretraining pool: open racing and road datasets plus YouTube kart footage selected by query | 100 h or more | Mixed | Datasets under their licenses; YouTube under the constraints in section 4 |

Hours are not the metric that matters for a segmentation model. Diversity is. Fifty hours of one track in one light is worth less than five hours across ten tracks, wet and dry, morning and evening. The frame extractor samples at 2 fps and removes near-duplicates for that reason, and the target numbers above are for post-dedupe footage, not raw uploads.

## 2. What exists today

### 2.1 Kart-specific

| Source | Content | Camera | Size | License | Access |
| --- | --- | --- | --- | --- | --- |
| Purdue Grand Prix Foundation, YouTube | Race broadcasts (Race 64 through Race 69), event videos | Trackside, occasional onboard | Tens of hours of broadcast | Standard YouTube | Channel `@purduegrandprixfoundation4523`, playlist `PLZ21C3BEHG5pM46OaDk3fF0ocYgMhuCx1`; ask the Foundation for raw onboard files from teams |
| Purdue Grand Prix onboard laps by student teams | Onboard laps at the exact track | Helmet and kart mounts | Unknown, likely a few hours | Standard YouTube | open-racer.com catalogs onboard videos for the Purdue track; teams post their own; ask them directly |
| Kart Chaser, YouTube | North American series broadcasts and onboard features (KA100, X30, KZ) | Mixed; onboard segments | Hundreds of hours of broadcast | Standard YouTube | `@KartChaser`; a media company, so ask for permission and raw files |
| FIA Karting Championship, YouTube | European and World Championship onboard laps and race coverage | Onboard and trackside | Tens of hours | Standard YouTube | Channel `UCj5zsq1zTNtcA4lDOUz2kAg` |
| RotaxKarting, Karting Live TV, Ultimate Karting Championship, BNL Karting Series, YouTube | Series streams and onboard features | Mixed | Hundreds of hours combined | Standard YouTube | See `training/data/sources.yaml` |
| Individual POV channels, YouTube | Full sessions, helmet cam or chassis cam, KA100, X30, KZ, Rotax, LO206 | Helmet (most), chassis (some) | Thousands of hours in aggregate | Standard YouTube unless the uploader chose CC BY | Discovered by query; see section 3 |
| `spawn99/karting`, Hugging Face | 1,076 stills from race footage with kart bounding boxes (YOLO format, Roboflow export) | Broadcast | Small | Not stated | `huggingface-cli download spawn99/karting --repo-type dataset` |
| AKS `GoKartSampleSet`, GitHub | Trackside photos of AKS teams' karts for detector training | Trackside at kart height | Unknown | GPL-3.0 | Clone |
| `hamidebadi/autonomous_platform_gokartcentrallen_imitation_learning_dataset`, Hugging Face | Color, depth and ORB image pickles from an autonomous platform at an indoor kart center, about 4.4 GB | Onboard | A few hours of frames | Gated | Request access on the dataset page |

### 2.2 Racing, not karts

| Source | Content | Size | License | Why it helps |
| --- | --- | --- | --- | --- |
| RoRaTrack (Purdue, Black and Gold Racing) | 1,398 annotated multi-camera images with track masks from a Dallara AV-21 at Putnam Park, Indiana | Small | Not stated; academic | Same task (track surface segmentation with no lane markings), same state, same university; code at `github.com/ghosh64/RaceGAN`, data on Kaggle `shreya64/roratrack-dataset` |
| RACECAR (Indy Autonomous Challenge) | 6.5 h across 27 sessions, six cameras, LiDAR, radar, RTK; ROS 2 and nuScenes formats | Large | CC BY-NC 4.0 | Only large open racing camera set; ovals, so limited boundary geometry; `aws s3 cp s3://racecar-dataset/ ... --no-sign-request` |
| Roboflow `autonomous-driving-challenge/racetrack` | 4,698 instance-segmentation images of racetracks (cars, pit entry, obstacles) | Medium | Per Roboflow project | Track-domain segmentation pretraining |
| `ghosh64/track-detection`, GitHub | Track boundary masks on frames sampled from four racing YouTube videos | Small | Not stated | Worked example of the YouTube-to-masks pipeline |
| A2RL Vmax (TUM, Yas Marina) | About 30,000 annotated LiDAR and radar frames from the 2024 A2RL | Large | Check site | LiDAR only in the annotated set; relevant if the Mid-360 is fitted |
| FSOCO | 11,572 images, 220,862 cone annotations, boxes and masks | Medium | GPL-3.0; public since 2024 | Cone detection if an event adds cones; the track surface is not labeled |
| EUFS datasets | ROS bags with ZED stereo at 672x376, LiDAR, IMU, from a Formula Student car | Small | Not stated | Format reference for our own MCAP logs; low resolution |

### 2.3 Road driving, for pretraining only

| Source | Content | Size | License |
| --- | --- | --- | --- |
| comma2k19 | 33 h of highway driving, front camera, GNSS, IMU, CAN | 100 GB | MIT |
| OpenDV-YouTube (GenAD) | 1,700 h of YouTube driving videos, distributed as a video ID list plus a download toolkit and language annotations (CC BY-NC-SA 4.0 for the annotations) | Very large | Videos under their YouTube licenses |
| `samuelsze/drivable_area_segmentation`, Hugging Face | 26,138 image and mask pairs for drivable area | 41 MB download | Not stated |
| `urjc-deepracer/carla-expert-racing`, Hugging Face | CARLA front camera with semantic masks and telemetry on custom racing maps | 10k to 100k samples | Apache-2.0 |

Road datasets teach a model what asphalt, grass, curbs and shadows look like from a low camera; they do not teach kart-track geometry. Use them for the encoder, then fine-tune on tiers A and B.

## 3. YouTube: finding the hours

`training/data/fetch_youtube.py` reads `sources.yaml`, expands each entry (a channel, a playlist, or a search query) into a candidate list with `yt-dlp`, filters by duration and title, fetches metadata including the license field, and downloads what passes into `raw/youtube/<id>/` with a manifest row. Run it with `--dry-run` first: it prints candidate counts and hours per source without downloading anything, which is how the team measures what each channel is actually worth before committing disk.

The query set in `sources.yaml` covers the classes and camera types that produce onboard footage: `kart onboard`, `karting onboard`, `go kart POV`, `kart helmet cam`, `kart chassis cam`, `KA100 onboard`, `X30 onboard`, `KZ onboard`, `Rotax onboard`, `LO206 onboard`, `kart rain onboard`, `kart practice onboard full session`, plus the French, Italian, Spanish and German equivalents, and the Purdue-specific `Purdue Grand Prix onboard` and `evGrandPrix onboard`. Each query is capped at a few hundred results; the default order is YouTube relevance, and `--search-sort date` reorders by upload date so reruns pick up new uploads. `--search-cc` asks YouTube for Creative Commons results only, which is the cheap way to build the tier C pool.

Camera placement is recorded per source as a hint (`helmet`, `chassis_front`, `chassis_rear`, `trackside`, `mixed`) and carried into the frame index. For our model the chassis-front class is the one to prefer, because the camera on the kart will sit about 0.6 to 0.9 m above the pavement, forward-facing and rigid. Helmet footage sits higher and moves with the driver's head; it still trains a segmentation model well when the horizon is randomized in augmentation. Broadcast footage is used only for kart detection, never for boundaries.

Expect the raw take to be dominated by helmet cams and to include a lot of pit lane, grid and post-race walking. The extractor's skip-intro and skip-outro parameters and the per-source crop (to remove GoPro overlays and helmet chin bars) handle most of it; the rest is removed at labeling time.

## 4. The legal position, stated once

YouTube's terms of service prohibit downloading content except through features YouTube provides, and since December 2024 creators control a per-channel setting that allows third-party AI training, which is off by default. Creative Commons Attribution is the only CC license YouTube offers, and it licenses reuse of the work, not the act of scraping. The academic norm (OpenDV-YouTube, the track-detection repository above) is to publish a video ID list and let each group download for itself; that is a convention, not a permission.

So the toolkit does three things by design. It runs with `--cc-only` as the default download mode, which keeps only videos whose metadata carries the Creative Commons Attribution license and records the attribution in the manifest. It records every video's uploader and URL so that permission requests can be sent, and `sources.yaml` carries a `permission` field per source that must be set to `granted` before non-CC downloads from that source are allowed. And it never redistributes video: the repository holds `sources.yaml` and manifests, not footage.

The most valuable footage is the easiest to clear. The Purdue Grand Prix Foundation and the fraternity and club teams that race there are on campus; Kart Chaser and the AKS teams answer email. A shared drive of raw GoPro files from three teams' 2026 qualifying sessions beats a scraped 1080p re-encode of the same laps in every way, including resolution and the absence of overlays.

## 5. From video to labels

1. `extract_frames.py` samples every video at 2 fps, skips the first and last 20 s, applies the per-source crop, resizes to a 1280 px long edge, and drops frames whose perceptual hash is within a Hamming distance of 6 of a kept frame. Output is `frames/<source>/<video>/<t_ms>.jpg` plus `frames/index.csv` with source, video, timestamp, camera hint and license.
2. Auto-label with Grounded SAM 2 using the prompts `asphalt track`, `grass`, `curb`, `gravel`, `barrier`, `go-kart`, producing a pavement mask and kart boxes per frame. Grounded SAM 2 tracks masks across a clip, so labeling at 2 fps costs one prompt per shot, not per frame.
3. Review in CVAT or Label Studio. The reviewer's job is to reject bad masks, not to draw; a few thousand reviewed frames across tiers A and B is the first training set.
4. Store masks as PNG next to the frame, in the same directory layout, with a `labels.csv` recording who reviewed what. Version the manifest and label indices in git; keep the pixels on the team's shared storage.
5. For the Purdue track, project the surveyed edges from `tracks/` into our own camera frames using the calibrated extrinsics and the logged pose. That gives labels for tier A at zero human cost and is the reason `mcq_localization` and the survey tool come before perception on the roadmap.

## 6. Synthetic data

Assetto Corsa with the Modular Kart mod and community kart circuits renders convincing POV footage but has no segmentation output without shader-patch hacks. CARLA gives pixel-perfect masks and is what the URJC-DeepRacer dataset used, but has no kart and no kart tracks. The route that fits this project is to build the Purdue track in Isaac Sim or Unity from the high-resolution scan the team is obtaining, drive the bicycle-model simulator through it, and render camera plus mask pairs with randomized lighting, weather and camera height. That is a Phase 4 task and depends on the scan.

## 7. Acceptance for the first perception model

The first model is a small segmentation network (pavement, not-pavement, kart) exported to TensorRT. It is trained on tier C, fine-tuned on tiers A and B, and evaluated only on held-out tier A laps at the Purdue track: mean IoU on the pavement class above 0.95 in daylight, boundary polyline error under 0.3 m at 15 m range after inverse perspective mapping, at 20 fps or better on the Orin. Until tier A exists, evaluation uses a held-out slice of tier B with the caveat that the camera does not match.
