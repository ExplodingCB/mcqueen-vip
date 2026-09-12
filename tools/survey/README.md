# tools/survey

Turns two edge drives (left edge, then right edge, both in the direction of travel, RTK fixed) into `tracks/<id>/track.csv` and `track.yaml`. Input is csv with `x,y` in the map frame or `lat,lon[,height]`, or an MCAP log with an `EgoState` or `NavSatFix` topic once `mcap` and `mcap-ros2-support` are installed.

```
python tools/survey/survey.py --left left.csv --right right.csv --out tracks/purdue --datum 40.4237 -86.9212 190
python tools/survey/survey.py --left session.mcap --right session2.mcap --topic /ego_state --out tracks/purdue
python -m pytest -q tools/survey
```

The pipeline drops standstill samples, cuts each drive where it returns to its start, resamples by arc length, smooths with a 3 m window, pairs the edges, takes the midpoint as the centerline and the edge distances as half-widths. On a synthetic oval with 2 cm noise and uneven sampling it recovers the centerline to within 8 cm.
