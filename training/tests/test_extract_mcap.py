"""Write a session with images and EgoState to MCAP the way rosbag2 would,
then pull frames with interpolated poses back out."""

import io
import math

import numpy as np
import pytest
from PIL import Image

from mcq_training import extract_mcap as X
from mcq_training import rosmsg
from mcq_training.dataset import read_rows

mcap_writer = pytest.importorskip("mcap_ros2.writer")


def test_definitions_include_dependencies():
    ego = rosmsg.definition("mcq_msgs/EgoState")
    assert ego.startswith("# Fused kart state") or "geometry_msgs/Pose pose" in ego
    assert "MSG: std_msgs/Header" in ego and "MSG: geometry_msgs/Pose" in ego
    assert "MSG: builtin_interfaces/Time" in ego and "MSG: geometry_msgs/Quaternion" in ego
    det = rosmsg.definition("mcq_msgs/Detections")
    assert "MSG: mcq_msgs/Detection" in det and "MSG: geometry_msgs/Vector3" in det
    assert rosmsg.definition("sensor_msgs/CompressedImage").count("MSG:") == 2


def _write_bag(path, oval, seconds=3.0, ego_hz=100, cam_hz=20):
    with open(path, "wb") as f:
        w = mcap_writer.Writer(f)
        ego_schema = w.register_msgdef("mcq_msgs/EgoState", rosmsg.definition("mcq_msgs/EgoState"))
        img_schema = w.register_msgdef("sensor_msgs/CompressedImage", rosmsg.definition("sensor_msgs/CompressedImage"))
        v = 4.0
        n_ego = int(seconds * ego_hz)
        for i in range(n_ego):
            t = i / ego_hz
            s = 5.0 + v * t
            x, y, psi = oval.cartesian(s, 0.0)
            ns = int(t * 1e9)
            msg = {
                "header": {"stamp": {"sec": ns // 10**9, "nanosec": ns % 10**9}, "frame_id": "map"},
                "pose": {
                    "position": {"x": float(x[0]), "y": float(y[0]), "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(psi[0] / 2), "w": math.cos(psi[0] / 2)},
                },
                "v": v if t < 2.5 else 0.0,  # the kart stops for the last half second
                "v_lat": 0.0,
                "yaw_rate": 0.0,
                "a_long": 0.0,
                "a_lat": 0.0,
                "covariance": [0.0] * 36,
                "gnss_status": 2 if t < 2.0 else 1,  # RTK drops to float after 2 s
                "s": float(s),
                "d": 0.0,
            }
            w.write_message("/ego_state", ego_schema, msg, log_time=ns, publish_time=ns, sequence=i)
        n_img = int(seconds * cam_hz)
        for i in range(n_img):
            t = i / cam_hz + 0.007
            ns = int(t * 1e9)
            im = Image.fromarray(np.full((40, 64, 3), 100 + i, dtype=np.uint8))
            buf = io.BytesIO()
            im.save(buf, format="JPEG")
            msg = {
                "header": {"stamp": {"sec": ns // 10**9, "nanosec": ns % 10**9}, "frame_id": "camera_front"},
                "format": "jpeg",
                "data": buf.getvalue(),
            }
            w.write_message("/camera/front/image/compressed", img_schema, msg, log_time=ns, publish_time=ns, sequence=i)
        w.finish()


def test_extract_frames_with_interpolated_poses(tmp_path, oval):
    bag = tmp_path / "2026-11-02_1410_oval_kartA_practice.mcap"
    _write_bag(bag, oval)
    out = tmp_path / "frames"
    stats = X.extract(bag, "/camera/front/image/compressed", "/ego_state", out, "kart_test", "sess1", fps=5.0)
    # 3 s at 5 fps is 15 candidates; the last second has no RTK fix or is standing still.
    assert stats["images"] == 60
    assert 9 <= stats["kept"] <= 11, stats
    assert stats["no_fix"] >= 2
    poses = read_rows(out / "kart_test" / "sess1" / "poses.csv")
    index = read_rows(out / "index.csv")
    assert len(poses) == len(index) == stats["kept"]
    assert all(r["license"] == "own" and r["source"] == "kart_test" for r in index)
    for r in poses:
        t = int(r["t_ns"]) / 1e9
        x, y, _ = oval.cartesian(5.0 + 4.0 * t, 0.0)
        assert abs(float(r["x"]) - x[0]) < 0.02 and abs(float(r["y"]) - y[0]) < 0.02
        assert (out / r["frame"]).exists()
    with Image.open(out / poses[0]["frame"]) as im:
        assert im.size == (64, 40)


def test_cli_and_allow_float(tmp_path, oval):
    bag = tmp_path / "s.mcap"
    _write_bag(bag, oval)
    out = tmp_path / "frames"
    assert X.main([str(bag), "--out", str(out), "--fps", "2", "--allow-float", "--min-speed", "0"]) == 0
    index = read_rows(out / "index.csv")
    assert len(index) == 6 and index[0]["source"] == "kart_s" and index[0]["video_id"] == "s"
