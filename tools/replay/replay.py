#!/usr/bin/env python3
"""Replay MCAP through a real ROS node using acknowledged inputs and explicit ticks."""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from collections import Counter, defaultdict, deque
from pathlib import Path

import yaml
from compare import ReplayMismatch, compare


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def load_bag(path, topics):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    metadata = {item.name: item for item in reader.get_all_topics_and_types() if item.name in topics}
    missing = topics - metadata.keys()
    if missing:
        raise ValueError(f"bag is missing mapped topics: {sorted(missing)}")
    records = []
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic in topics:
            records.append((topic, payload, timestamp))
    # Stable for tied timestamps: preserve the recorder's stored order.
    records.sort(key=lambda record: record[2])
    counts = Counter(record[0] for record in records)
    empty = topics - counts.keys()
    if empty:
        raise ValueError(f"bag has no messages on mapped topics: {sorted(empty)}")
    return metadata, records


def run(args):
    import rclpy
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Header

    config = yaml.safe_load(args.topic_map.read_text())
    inputs, outputs = config["inputs"], config["outputs"]
    if not inputs or not outputs or inputs.keys() & outputs.keys():
        raise ValueError("topic map needs nonempty, disjoint input and output maps")
    metadata, records = load_bag(args.bag, inputs.keys() | outputs.keys())
    types = {topic: get_message(item.type) for topic, item in metadata.items()}
    namespace = f"/mcq_replay_{uuid.uuid4().hex}"
    qos = QoSProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE, depth=100)
    rclpy.init()
    node = Node("replay_driver", namespace=namespace)
    pending = defaultdict(deque)
    receipts = deque()
    publishers = {topic: node.create_publisher(types[topic], target, qos) for topic, target in inputs.items()}

    def output_callback(topic):
        def receive(message):
            pending[topic].append(message)

        return receive

    subscriptions = {
        topic: node.create_subscription(types[topic], spec["topic"], output_callback(topic), qos)
        for topic, spec in outputs.items()
    }
    ticks = node.create_publisher(Header, "replay/tick", qos)
    receipt_subscription = node.create_subscription(Header, "replay/received", lambda msg: receipts.append(msg), qos)
    parameters = args.parameters or (args.topic_map.parent / config["parameters"]).resolve()
    node_name = config["node_name"]
    # A bare `controller:` YAML key does not match a namespaced node. Preserve
    # every source value while scoping it to this isolated replay instance.
    parameter_values = yaml.safe_load(parameters.read_text())[node_name]["ros__parameters"]
    scoped_parameters = tempfile.TemporaryDirectory(prefix="mcq-replay-params-")
    scoped_path = Path(scoped_parameters.name) / "parameters.yaml"
    scoped_path.write_text(yaml.safe_dump({f"{namespace}/{node_name}": {"ros__parameters": parameter_values}}))
    command = [
        *config["command"],
        "--ros-args",
        "--params-file",
        str(scoped_path),
        "-r",
        f"__ns:={namespace}",
        "-r",
        f"__node:={node_name}",
        "-p",
        "replay_mode:=true",
    ]
    log_path = args.node_log
    log_path.parent.mkdir(parents=True, exist_ok=True)
    process = None
    writer = None
    checked = Counter()
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def wait_for(condition, label, timeout=None):
            deadline = time.monotonic() + (timeout if timeout is not None else args.timeout)
            while not condition():
                if process.poll() is not None:
                    raise RuntimeError(f"node exited {process.returncode} during {label}; see {log_path}")
                if time.monotonic() >= deadline:
                    raise ReplayMismatch(f"timeout waiting for {label}; see {log_path}")
                rclpy.spin_once(node, timeout_sec=0.02)

        wait_for(
            lambda: (
                all(pub.get_subscription_count() for pub in [*publishers.values(), ticks])
                and all(
                    node.count_publishers(sub.topic_name) for sub in [*subscriptions.values(), receipt_subscription]
                )
            ),
            "DDS discovery",
            30.0,
        )
        if args.capture:
            if args.capture.exists():
                raise ValueError(f"capture destination already exists: {args.capture}")
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(args.capture), storage_id="mcap"), rosbag2_py.ConverterOptions("", "")
            )
            for item in metadata.values():
                writer.create_topic(item)
        previous_tick = None
        for topic, payload, timestamp in records:
            expected = deserialize_message(payload, types[topic])
            header_stamp = stamp_ns(expected.header)
            context = f"{topic} at timestamp={timestamp} ns (header={header_stamp} ns)"
            if topic in inputs:
                publishers[topic].publish(expected)
                wait_for(lambda: receipts, f"input acknowledgement {context}")
                receipt = receipts.popleft()
                if receipt.frame_id != inputs[topic] or stamp_ns(receipt) != header_stamp:
                    raise ReplayMismatch(f"unexpected input acknowledgement during {context}: {receipt}")
                if writer:
                    writer.write(topic, payload, timestamp)
                continue
            if previous_tick != header_stamp:
                if previous_tick is not None and header_stamp < previous_tick:
                    raise ValueError(
                        f"output header timestamps move backwards at {context}; split clock-reset sessions"
                    )
                leftovers = {key: len(value) for key, value in pending.items() if value}
                if leftovers:
                    raise ReplayMismatch(f"unexpected outputs before {context}: {leftovers}")
                tick = Header()
                tick.stamp = expected.header.stamp
                ticks.publish(tick)
                previous_tick = header_stamp
            wait_for(lambda t=topic: pending[t], f"output {context}")
            actual = pending[topic].popleft()
            if writer:
                if stamp_ns(actual.header) != header_stamp:
                    raise ReplayMismatch(f"node did not honor replay tick during {context}")
                writer.write(topic, serialize_message(actual), timestamp)
            else:
                compare(
                    expected,
                    actual,
                    topic=topic,
                    timestamp_ns=header_stamp,
                    tolerances=outputs[topic].get("tolerances", {}),
                )
            checked[topic] += 1
        # Catch an extra output even if it arrives after the expected final one.
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        if receipts or any(pending.values()):
            raise ReplayMismatch("unexpected extra output or acknowledgement after final recorded event")
        print(json.dumps({"result": "captured" if writer else "passed", "outputs": dict(checked)}, sort_keys=True))
    finally:
        # Release the writer before process exit so metadata and chunks are finalized.
        if writer:
            writer.close()
        if process:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
        scoped_parameters.cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--map", dest="topic_map", type=Path, default=Path(__file__).with_name("controller.yaml"))
    parser.add_argument("--parameters", type=Path, help="override the mapped YAML parameter file")
    parser.add_argument("--capture", type=Path, help="record node outputs as a new reference instead of comparing")
    parser.add_argument("--node-log", type=Path, default=Path("replay-node.log"))
    parser.add_argument("--timeout", type=float, default=3.0, help="maximum wall seconds per receipt/output")
    args = parser.parse_args()
    try:
        run(args)
    except (AssertionError, RuntimeError, ValueError) as error:
        print(f"REPLAY FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
