import csv
from dataclasses import asdict

import pytest

from mcq_sim.dynamics import DynamicKart
from mcq_sim.environment import load_kart
from mcq_sim.validation import validate_recording


def write_recording(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def synthetic_recording():
    sim = DynamicKart(load_kart()[0])
    rows = [{**asdict(sim.state), "steer_cmd": 0, "throttle_cmd": 0, "brake_cmd": 0}]
    for i in range(250):
        command = (0.12 if i > 150 else 0, 0.7, 0)
        state = sim.step(*command)
        rows.append({**asdict(state), "steer_cmd": command[0], "throttle_cmd": command[1], "brake_cmd": command[2]})
    return rows


def test_replay_matches_independently_generated_commands_without_accuracy_claim(tmp_path):
    path = tmp_path / "synthetic.csv"
    write_recording(path, synthetic_recording())
    report = validate_recording(path)
    assert report["position_rmse_m"] < 1e-8
    assert report["speed_rmse_mps"] < 1e-8
    assert report["accuracy_validated"] is False


def test_replay_detects_observed_position_bias(tmp_path):
    rows = synthetic_recording()
    for row in rows[1:]:
        row["x"] += 1.0
    path = tmp_path / "biased.csv"
    write_recording(path, rows)
    assert validate_recording(path)["position_rmse_m"] == pytest.approx(1, abs=1e-6)


@pytest.mark.parametrize("defect", ["nan", "backward", "gap", "missing"])
def test_bad_recordings_rejected(tmp_path, defect):
    rows = synthetic_recording()[:4]
    if defect == "nan":
        rows[2]["v"] = float("nan")
    elif defect == "backward":
        rows[2]["t"] = rows[1]["t"]
    elif defect == "gap":
        rows[3]["t"] = 4
    else:
        for row in rows:
            row.pop("yaw_rate")
    path = tmp_path / "bad.csv"
    write_recording(path, rows)
    with pytest.raises(ValueError):
        validate_recording(path)
