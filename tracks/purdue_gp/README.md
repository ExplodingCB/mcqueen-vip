# Purdue Grand Prix centerline

The current centerline follows the user's refined KML. Pavement edges remain based on a georeferenced 2023 Indiana orthophoto. It replaces an OSM path that incorrectly followed the pit lane along the bottom. This remains an image reconstruction, not a survey. Build date: 2026-09-22.

`region.kml` preserves the user's original area of interest. `user_track.kml` preserves the latest 63 circuit points, plus the closing point. This revision moves 19 points from the initial KML, archived as `user_track_initial.kml`. Although exported as a polygon, the coordinates define a closed centerline. The user explicitly confirmed counterclockwise travel on 2026-09-22. The builder enforces positive signed area in ENU and starts southeast along the main straight. The zero KML altitudes are not ground elevation.

`aerial.png` is a pinned, georeferenced export of Indiana raster `in2023_29901890_06`, ID 270339, native resolution 6 inches. `aerial_source.json` records the service, explicit raster selection, extent, dimensions and request parameters. [Indiana's imagery service](https://www.in.gov/gis/geoinsights/posts/indianas-imagery-services/) describes the source. The service states CC0 access. The UI can display this image under the track for direct alignment checks.

`centerline_pixels.csv` retains the earlier image trace for pavement-edge estimates. Paint-line contrast estimates the edges; unclear sections use a 5.2 m total-width fallback around that image trace. The revised KML is converted from WGS84 to local ENU and interpolated with a periodic spline, keeping every user waypoint. Intermediate collinear knots constrain the long straights. New centerline normals intersect the existing estimated pavement boundaries to obtain asymmetric widths; the road is not moved or widened to center the supplied line. Widths, elevation, barriers and curb heights still need measurement. The simulated start is at the visible checkered line.

The resulting centerline is approximately 435.07 m long. This is a calculation from the supplied line. Widths are measured along its normals, so they can increase where it crosses the pavement obliquely. Datum: 40.43775 N, -86.94475 E, 0 m. The user's coordinates are included in `track.yaml` and can be overlaid; interpolation agreement is not survey validation. The reference controller centers its target between the pavement edges, accounting for asymmetric widths.

Rebuild offline:

```sh
.venv/bin/python tools/track/build_purdue.py
```

`track.yaml` records image/trace hashes and uncertainty. `track.csv` uses the existing TUM centerline/width format. Replace it with RTK pavement-edge surveys or a registered scan before claiming measured real-world clearance.

The unused `source.osm` is retained to document the rejected first source. It is copyright OpenStreetMap contributors, [ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/). The current centerline comes from the user KML; pavement-edge estimates come from the Indiana CC0 orthophoto, not that OSM path.
