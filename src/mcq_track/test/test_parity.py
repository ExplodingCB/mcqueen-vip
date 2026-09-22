"""Compile the C++ core and compare the unchanged Python reference on identical CSVs."""

import subprocess
from pathlib import Path

import numpy as np
import pytest

from mcq_sim.track import Track

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    build = tmp_path_factory.mktemp("track_core")
    subprocess.run(["cmake", "-S", str(ROOT / "src/mcq_track/core"), "-B", str(build)], check=True, capture_output=True)
    subprocess.run(["cmake", "--build", str(build)], check=True, capture_output=True)
    subprocess.run(["ctest", "--test-dir", str(build), "--output-on-failure"], check=True, capture_output=True)
    return build / "track_probe"


@pytest.mark.parametrize("kind", ["oval", "open", "nonuniform", "duplicates", "triangle"])
def test_cpp_matches_python_on_same_csv(probe, tmp_path, kind):
    path = ROOT / "tracks/synthetic_oval/track.csv"
    closed = kind != "open"
    if kind != "oval":
        path = tmp_path / "track.csv"
        if kind == "open":
            data = [[0, 0, 1, 2], [1, 0.5, 1.2, 2.2], [3, 1.1, 0.9, 1.8], [5, -0.2, 1.4, 2.1]]
        elif kind == "triangle":
            data = [[0, 0, 1, 2], [3, 0, 2, 1], [1, 3, 1.5, 1.2]]
        else:
            data = [[0, 0, 1, 2], [3, 0.5, 1.2, 2.2], [6, 4, 0.9, 1.8], [1, 7, 1.4, 2.1], [-2, 3, 2, 1]]
            if kind == "duplicates":
                data.insert(1, data[0])
                data.append(data[0])
        np.savetxt(path, data, delimiter=",")
    track = Track.from_csv(path, closed=closed)
    rng = np.random.default_rng(12)
    s = np.concatenate(
        [track.s, [0, track.length, -track.length, track.length * 2], rng.uniform(-10, track.length + 10, 2000)]
    )
    d = rng.uniform(-4, 4, len(s))
    x, y, psi = track.cartesian(s, d)
    # Independent Cartesian query positions exercise nearest-neighbor and seam selection.
    qx, qy = x + rng.normal(0, 0.03, len(x)), y + rng.normal(0, 0.03, len(y))
    if kind != "oval":
        # At a coarse polyline vertex, two clamped segments can tie while
        # reporting different lateral offsets. Query segment interiors instead.
        a = rng.integers(0, len(track.x) if closed else len(track.x) - 1, len(s))
        b = (a + 1) % len(track.x)
        t = rng.uniform(0.2, 0.8, len(s))
        qx = track.x[a] + t * (track.x[b] - track.x[a]) + rng.normal(0, 0.01, len(s))
        qy = track.y[a] + t * (track.y[b] - track.y[a]) + rng.normal(0, 0.01, len(s))
    stdin = "\n".join(" ".join(format(v, ".17g") for v in row) for row in zip(s, d, qx, qy, strict=True))
    result = subprocess.run(
        [str(probe), str(path), str(int(closed))], input=stdin, text=True, capture_output=True, check=True
    )
    actual = np.loadtxt(result.stdout.splitlines())
    wl, wr = track.width_at(s)
    sf, df = track.frenet(qx, qy)
    expected = np.column_stack([np.full(len(s), track.length), x, y, psi, track.curvature_at(s), wl, wr, sf, df])
    error = actual - expected
    error[:, 3] = (error[:, 3] + np.pi) % (2 * np.pi) - np.pi
    if closed:
        error[:, 7] = (error[:, 7] + track.length / 2) % track.length - track.length / 2
    assert np.max(np.abs(error)) < 1e-6, np.max(np.abs(error), axis=0)


@pytest.mark.parametrize(
    "csv", ["0,0,1\n1,0,1\n2,0,1\n", "0,0,1,1\n1,0,-1,1\n2,0,1,1\n", "0,0,1,1\n1,nan,1,1\n2,0,1,1\n"]
)
def test_invalid_track_is_rejected(probe, tmp_path, csv):
    path = tmp_path / "track.csv"
    path.write_text(csv)
    result = subprocess.run([str(probe), str(path), "1"], input="", text=True, capture_output=True)
    assert result.returncode != 0
