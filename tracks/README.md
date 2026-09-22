# tracks

One directory per track. `track.csv` is the centerline with half-widths in the
TUM optimizer format (`# x_m, y_m, w_tr_right_m, w_tr_left_m`, map frame, about
1 m spacing, closed loop). `track.yaml` carries the datum, the start/finish
line, the id and the survey date. Generated racelines go next to them as
`raceline_<date>.csv` in the optimizer's output format.

| Directory | Source | Notes |
| --- | --- | --- |
| `synthetic_oval` | `python -m mcq_sim make-track` | 60 m straights, 15 m radius, 5 m wide, 214 m. Phase 0 test track. |
| `purdue_gp` | Indiana orthophoto trace and user KML anchors | Image-derived geometry and widths, no measured elevation. See its README. |

Replace the provisional Purdue geometry using `tools/survey` and an RTK log of
both pavement edges. Keep the measured track in a separate directory until its
frame, direction and boundary checks pass. Coordinates live in `tracks/`.
