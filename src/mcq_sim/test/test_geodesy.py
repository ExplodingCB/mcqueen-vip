"""The map frame conversions, in both directions.

The survey tool reads recorded fixes into the map frame and the simulator's
GNSS model writes the map frame back out as latitude and longitude. If the two
directions disagree, a surveyed track and a simulated drive of the same track
do not land in the same place, which is the kind of error that looks like a
localization bug for a week.
"""

import numpy as np
import pytest

from mcq_sim.geodesy import enu_to_geodetic, geodetic_to_enu

# West Lafayette, near the Grand Prix track. No real survey coordinates live in
# source (docs/06-dev-setup.md section 5); this is a round-number datum.
DATUM = (40.4237, -86.9212, 190.0)


def test_datum_maps_to_the_origin():
    lat, lon, h = enu_to_geodetic(0.0, 0.0, 0.0, DATUM)
    assert (float(lat), float(lon), float(h)) == pytest.approx(DATUM)
    e, n, u = geodetic_to_enu(*DATUM, DATUM)
    assert (float(e), float(n), float(u)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_round_trip_over_a_track_sized_area():
    rng = np.random.default_rng(0)
    east = rng.uniform(-500.0, 500.0, 500)
    north = rng.uniform(-500.0, 500.0, 500)
    up = rng.uniform(-20.0, 20.0, 500)
    lat, lon, h = enu_to_geodetic(east, north, up, DATUM)
    e2, n2, u2 = geodetic_to_enu(lat, lon, h, DATUM)
    # Both directions go through ECEF, so the round trip is numerical noise.
    assert np.abs(e2 - east).max() < 1e-6
    assert np.abs(n2 - north).max() < 1e-6
    assert np.abs(u2 - up).max() < 1e-6


def test_axis_directions():
    # North increases latitude and leaves longitude exactly alone; a degree of
    # latitude is about 111 km.
    lat, lon, _ = enu_to_geodetic(0.0, 1000.0, 0.0, DATUM)
    assert lat > DATUM[0]
    assert float(lon) == pytest.approx(DATUM[1], abs=1e-12)
    assert (lat - DATUM[0]) * 111_000 == pytest.approx(1000.0, rel=0.01)

    lat, lon, _ = enu_to_geodetic(1000.0, 0.0, 0.0, DATUM)
    assert lon > DATUM[1]
    # Latitude does not stay put: the map frame is a plane tangent at the datum,
    # so a straight line east leaves the curving ellipsoid behind. At 1 km that
    # is 7 cm of latitude, which is geometry rather than error.
    assert abs(float(lat) - DATUM[0]) < 1e-6


def test_the_tangent_plane_is_flat_enough_for_a_track():
    # Going straight on the plane means rising above the ellipsoid by roughly
    # d^2 / 2R. Worth pinning, because it is the reason the map frame is
    # anchored per track and not per region: at 200 m, our scale, it is 3 mm;
    # at 20 km it would be 31 m.
    earth_radius = 6.371e6
    for distance, tolerance in ((200.0, 1e-3), (1000.0, 2e-3)):
        _, _, h = enu_to_geodetic(distance, 0.0, 0.0, DATUM)
        assert float(h) - DATUM[2] == pytest.approx(distance**2 / (2 * earth_radius), abs=tolerance)
    _, _, h = enu_to_geodetic(200.0, 0.0, 0.0, DATUM)
    assert float(h) - DATUM[2] < 0.004


def test_a_simulated_drive_survives_the_survey_tool():
    """The loop that matters: the simulator's GNSS model writes map coordinates
    out as latitude and longitude, and the survey tool reads recorded fixes back
    into map coordinates. If these two disagree, a track surveyed from a
    simulated drive lands somewhere other than where the drive went. What this
    catches is a gross mistake in the plumbing: swapped axes, a sign, a datum
    read from the wrong place. It would not catch the tangent-plane warp, which
    needs hundreds of metres to show up and is pinned by the round-trip test
    above; the oval sits 30 m from the datum, where that error is a micron."""
    from mcq_sim.sensors import SensorSuite
    from mcq_sim.track import Track

    oval = Track.synthetic_oval()
    suite = SensorSuite.from_params(seed=4)

    def truth_at(t: float) -> tuple[float, float]:
        x, y, _ = oval.cartesian(np.array([(6.0 * t) % oval.length]), 0.0)
        return float(x[0]), float(y[0])

    expected, published = [], []
    for k in range(2000):  # 20 s at 100 Hz
        t = k * 0.01
        x, y = truth_at(t)
        for sample in suite.gnss.step(t, x, y, 0.0, 0.0):
            # Exactly what sim_node does before publishing a NavSatFix. The
            # sample carries the time it was measured, so that is the truth to
            # compare against, not the truth at the time it arrived.
            lat, lon, height = enu_to_geodetic(sample.x, sample.y, 0.0, DATUM)
            published.append((float(lat), float(lon), float(height)))
            expected.append(truth_at(sample.t))

    assert len(published) > 300
    lat, lon, height = np.array(published).T
    east, north, _ = geodetic_to_enu(lat, lon, height, DATUM)
    recovered = np.column_stack([east, north])

    # All that is left is the 2 cm of GNSS noise the model put there on purpose.
    error = np.hypot(*(recovered - np.array(expected)).T)
    sigma = suite.gnss.params.sigma_fixed
    assert np.median(error) < 2 * sigma
    assert error.max() < 6 * sigma
