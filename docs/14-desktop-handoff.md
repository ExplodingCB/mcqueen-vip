# Moving the heavy runs to the desktop (RTX 5080, WSL2)

The MacBook Air is fine for editing and short checks but slow for the long jobs. The neural-net training is GPU-bound (an epoch takes about 9 minutes on the Mac's GPU), and the simulator work (frame generation, DAgger, the lap-time search) is CPU-bound and throttles on a fanless laptop. This note is for the agent that sets up the desktop and runs those jobs. Nothing here changes what the code does: `train_drive.py` already picks CUDA when it is there, and the C control libraries build themselves on Linux with cmake the first time they are imported.

## Who does what

* **Mac (this repo, branch `training-perception`)**: code changes, short simulator checks, the dashboard (`training/perception/watch.py`), the demo (`tools/sim_video/run_demo.sh`), docs.
* **Desktop (WSL2 Ubuntu)**: long training, DAgger collection, the lap-time search, the robustness suite.
* **Between them**: code moves by git. Data and weights move by `rsync` over Tailscale. Results come back as small files (a 16 MB `model.pt`, the `metrics.jsonl`, a JSON report). Nothing large goes through git; `runs/`, `raw/`, `frames/` and `masks/` are git-ignored.

## 1. Set up the desktop (once)

In WSL2 Ubuntu, clone the repo, check out the branch and run the setup script (it installs the build tools, `uv`, a Python 3.12 environment, PyTorch for CUDA 12.8 or newer because the RTX 5080 needs a recent build, and the other packages, then prints whether the GPU is seen):

```sh
git clone https://github.com/ExplodingCB/mcqueen-vip.git && cd mcqueen-vip
git checkout training-perception          # the branch with the driving-model work (it must be pushed first)
bash tools/desktop/setup_wsl.sh
```

Then run the test suite (the first run builds the C control libraries with cmake). All of it should pass:

```sh
PATH="$PWD/.venv/bin:$PATH" PYTHONPATH=training/perception:src/mcq_sim .venv/bin/python -m pytest -q src/mcq_sim/test training/perception/tests training/data/tests
```

SegFormer labeling (`training/data/autolabel.py`) additionally needs `transformers` and `torchvision`; labeling is already done on the Mac, so the desktop does not need them.

## 2. Data to copy from the Mac

The Mac does not accept SSH (Remote Login is off and is not being turned on), so `rsync` is not available. Instead the Mac packs one file and serves it over Tailscale only while the desktop downloads it.

On the Mac (one person at a keyboard; it serves to every device on the tailnet until stopped with Ctrl-C):

```sh
bash tools/desktop/make_bundle.sh serve
```

On the desktop:

```sh
bash tools/desktop/pull_bundle.sh      # downloads, verifies the checksum, unpacks into the repo
```

The bundle holds about 1.3 GB: the helmet-cam and nose-cam frames and their labels (`training/data/frames`, `masks`, `test_frames`, `test_masks`), the searched expert line (`runs/laptime-v2/best.json`) and the latest models (`drive-v5`, and `drive-v6` once it exists). The 3.5 GB of source videos and the 870 MB recovery-frames file are left out: the desktop regenerates recovery frames itself, faster. If a direct connection is not established, Tailscale relays the transfer and it will be slower; a second attempt resumes where it stopped.

## 3. Smoke test, and measure the speed

```sh
bash tools/desktop/speedtest.sh
```

It prints the CPU model and core count, how long the data generation took, the time per epoch and the held-out numbers. Report them: the point is to replace my estimate with a measurement. On the Mac a full-size epoch (28,000 frames plus recovery frames) takes about 9 to 10 minutes and the data generation about 8 to 10 minutes.

## 4. The jobs to run

All use `PYTHONPATH=training/perception:src/mcq_sim` and the two thread variables above. Use `--workers` equal to the core count minus two.

* **Train a driving model** (`train_drive.py`): the commands used so far are in `training/perception/runs/drive-v*.log` (the first line of each `metrics.jsonl` has the full arguments). The latest is drive-v6: `--init runs/drive-v5/model.pt --fine --corner-weight 2 --frames 28000 --epochs 8 --expert-json runs/laptime-v2/best.json --dagger runs/dagger-r2.npz --dagger-repeat 3 --real <both labels files> --real-hold t13IbWQsvsE xH4hDjq_n1w fjPo6uzeCyc`.
* **DAgger collection** (`dagger.py`): `--weights runs/<model>/model.pt --rollouts 160 --caps 6 8 10 --race-limits --sloppy-kart --expert-json runs/laptime-v2/best.json --out runs/dagger-rN.npz`.
* **Lap-time search** (`laptime_opt.py`): `--run laptime-v3 --generations 120 --population 16 --workers <cores-2> --resume runs/laptime-v2/best.json`. It is CPU-bound, so more cores help directly.
* **Closed-loop race** (`race_eval.py --weights runs/drive-v5/model.pt runs/drive-v6/model.pt --caps 4 6 8 10 --seeds 0 1 2`) and **robustness** (`robustness.py --weights runs/<model>/model.pt --cap 8`).
* **Real-footage score** (`real_eval.py --weights ... --frames training/data/test_frames --masks training/data/test_masks --tag nose-cam`).

Possible speedups worth trying on the 5080 that the Mac cannot use: bfloat16 mixed precision in `train_drive.py`, a larger `--batch` (the whole training set already lives in GPU memory as uint8), and `torch.compile`. Measure before and after; keep the held-out numbers the same.

## 5. What to send back

Small files only, to the Mac over the tailnet (`tailscale file cp <files> chases-macbook-air:` on the desktop, then `tailscale file get <folder>` on the Mac; Taildrop needs no SSH): `runs/<name>/model.pt`, `runs/<name>/metrics.jsonl`, and any JSON report under `logs/`. State the exact command and commit so a result can be reproduced. The dashboard on the Mac reads `runs/*/metrics.jsonl`, so copying a finished run's folder over is enough for it to show up.

## What this does not change

Nothing in this move makes the models any more real. The open questions are the same as in [the version log](13-driving-model-versions.md): no version has seen the kart's own camera or driven a real kart, the track file and the tyres, brakes and servo are placeholders, and the driving path exists only in the Python simulator.
