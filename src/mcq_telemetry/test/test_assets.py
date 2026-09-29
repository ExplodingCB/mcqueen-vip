import importlib.util
import json
import re
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def test_layout_topics_and_fields_are_available_through_bridge():
    config = yaml.safe_load((PACKAGE / "config/bridge.yaml").read_text())["foxglove_bridge"]["ros__parameters"]
    assert config["best_effort_qos_topic_whitelist"] == [".*"]
    assert config["min_qos_depth"] == config["max_qos_depth"] == 1
    types = {
        "telemetry/summary": "Telemetry",
        "ego_state": "EgoState",
        "vehicle_command": "VehicleCommand",
        "vehicle_state": "VehicleState",
        "trajectory": "Trajectory",
        "gateway_status": "GatewayStatus",
    }
    for name in ("pit", "localization-debug", "control-debug"):
        layout = json.loads((PACKAGE / "layouts" / (name + ".json")).read_text())

        def check_tree(tree, configs=layout["configById"]):
            if isinstance(tree, str):
                assert tree in configs
            else:
                check_tree(tree["first"])
                check_tree(tree["second"])

        check_tree(layout["layout"])
        for topic, field in re.findall(r'"(/[^".]+)(?:\.([^".\[]+))?[^\"]*"', json.dumps(layout["configById"])):
            assert any(re.fullmatch(pattern, topic) for pattern in config["topic_whitelist"])
            definition = (PACKAGE.parent / "mcq_msgs/msg" / (types[topic[1:]] + ".msg")).read_text()
            if field:
                assert re.search(r"^\S+\s+" + re.escape(field) + r"\b", definition, re.MULTILINE)


def test_comparison_rejects_rate_regression_and_stops():
    spec = importlib.util.spec_from_file_location("compare_metrics", PACKAGE / "tools/compare_metrics.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    baseline = {
        "exit_code": 0,
        "urgent_stops": 0,
        "progress_m": 120,
        "rates": {"ego_state": 100, "vehicle_command": 100, "trajectory": 20},
    }
    assert module.compare(baseline, baseline, 0.05) == []
    regression = {**baseline, "rates": {**baseline["rates"], "trajectory": 15}}
    assert module.compare(baseline, regression, 0.05)
    assert module.compare(baseline, {**baseline, "urgent_stops": 1}, 0.05)
