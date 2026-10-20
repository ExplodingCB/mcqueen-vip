# Demo runbook: showing the model to the leaders

A five-minute demo of drive-v5, a camera-only model, driving the simulated Purdue Grand Prix track, ending on what we need to make it run on the real kart. Everything on screen is the simulator; say so, because it is what keeps the claims honest.

## Before you leave (the night before or morning of)

1. **Quit everything heavy**: training jobs, the lap-time search, DAgger collection, video exports. The simulator runs about 1.6 times faster than real time on a quiet machine and slower than real time on a busy one (a training job slowed it to about half), so there is little headroom: a stuttering demo reads as a failing one. Leave the simulation speed at 1x. Check with `uptime`: the first load average should be under 3.
2. **Pre-show check** (1 to 3 minutes): `tools/sim_video/demo_check.sh`. It starts the demo, drives one full lap with drive-v5 (the model the demo opens with) and prints either `READY` or `NOT READY` and why. Do not present on a `NOT READY`.
3. **Start the demo**: `tools/sim_video/run_demo.sh`, then open http://127.0.0.1:8766/demo in Chrome, full screen (press F).
4. **Press P** for presentation mode. Press **space** once now to run a lap and see that the closing slide appears, then press space again to reset. The slide's asks live at the top of the script in `demo.html` (the `ASKS` list) and can be edited.
5. **Backup**: `logs/videos/demo-v5-lap.mp4` is a recorded lap of the same model (drive-v5, no speed cap) in case the live demo fails. Test that it plays.
6. The 3D view loads three.js from a CDN, so the studio needs a network connection. The map and the model's view work without one.

## The five minutes

| Time | What you do | What to say |
| --- | --- | --- |
| 0:00 | Presentation mode, chase camera, press space | "This is a kart driven by a camera. One image goes in; the model decides where to drive and how fast." |
| 0:30 | Point at the model's view (right side) | "The yellow dots are its own line, with the speed it wants at each point. Nobody gave it the track." |
| 1:30 | Point at the top-left HUD | "The safety layer keeps it on the pavement the camera can see. This counter is how many times it had to step in." |
| 2:30 | Press **1** to **4** to change cameras (cockpit, above, overview) | Let the picture talk. |
| 4:00 | The lap finishes and the closing slide appears | Read the lap time and the closest-to-an-edge number. |
| 4:30 | Stay on the slide | "This is the simulator. What we need to take it to the real kart is this list." |

If the room asks for a comparison, leave presentation mode (P), pick `expert (reads the map)` or an older driver from the menu, run it, and the session table at the right keeps every run side by side.

## If someone asks

* **"Is this the real kart?"** No. It is a simulated kart on a hand-traced track file. The camera view is the real 2023 aerial photo of the track draped over the ground. The next step is the real camera and the real track.
* **"Why is it capped?"** In presentation mode it is not: the model decides its own speed, still inside the stack's other limits (it only goes as fast as it can stop within the pavement it can see). On the real kart a human sets a maximum and raises it in steps (docs/04).
* **"Is this your best model?"** It is the best one that laps cleanly. A newer model trained on a faster line (drive-v6) has better numbers on paper but does not yet complete laps; we know why and are fixing it.
* **Leave "faster limits" off.** It is marked experimental because the newer models do not complete laps with it.
* **"Does it work on real video?"** Its pavement detection has been trained on real kart footage and scores about 0.84 against automatic labels on clips it never saw. Its racing line and speed have only been trained in the simulator. Real footage from our camera on the track is the test that settles that, and it is the first ask on the slide.
* **"How fast is it?"** 64.6 seconds around the track in the simulator, with about 0.4 m to spare from the edge. The simulator's tyres and brakes are placeholders until we measure the real kart.

## If something goes wrong

* The kart stops early: press **space** (it resets and runs again). The slide says why it stopped in plain words.
* The page is blank or the 3D view is empty: reload, and if the network is the problem, press **2** (cockpit) or use the map and model view, which do not need it.
* Everything is down: play the backup video.
