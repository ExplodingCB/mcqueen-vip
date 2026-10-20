#!/usr/bin/env python3
"""Exercise every layout's WebSocket subscriptions throughout a graph run.

Start before graph_smoke.sh. Requires websocket-client and a sourced ROS build.
This process never creates a DDS participant. SIGTERM writes the final report.
"""

import argparse
import json
import signal
import struct
import time
from pathlib import Path

import websocket
from rclpy.serialization import deserialize_message

from mcq_msgs.msg import Telemetry


def layout_topics(directory):
    def strings(value):
        if isinstance(value, str):
            yield value
        elif isinstance(value, dict):
            for item in value.values():
                yield from strings(item)
        elif isinstance(value, list):
            for item in value:
                yield from strings(item)

    return {
        value.split(".")[0]
        for path in directory.glob("*.json")
        for value in strings(json.loads(path.read_text()))
        if value.startswith("/")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--layouts", type=Path, default=Path(__file__).resolve().parents[1] / "layouts")
    parser.add_argument("--report", type=Path, default=Path("dashboard_websocket.json"))
    parser.add_argument("--graph-report", type=Path, default=Path("metrics_telemetry_dashboard.json"))
    args = parser.parse_args()
    started_wall = time.time()
    required = layout_topics(args.layouts)
    if not required or "/telemetry/summary" not in required:
        parser.error("layouts must include telemetry/summary")
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    deadline = time.monotonic() + 40
    connection = None
    while running and time.monotonic() < deadline:
        try:
            connection = websocket.create_connection(
                args.url, subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"], timeout=1
            )
            break
        except (OSError, websocket.WebSocketException):
            time.sleep(0.2)
    if connection is None:
        raise RuntimeError("bridge did not become available before the startup deadline")
    subscriptions, counts = {}, dict.fromkeys(required, 0)
    saw_rc = False
    first_auto = None
    last_auto = None
    errors = []
    summary_stamps = []
    try:
        while running:
            try:
                data = connection.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except websocket.WebSocketConnectionClosedException:
                data = None
            if not data:
                # graph_smoke tears down the bridge after the checker writes
                # its result. Only accept a fresh, successful run report.
                report = json.loads(args.graph_report.read_text()) if args.graph_report.exists() else {}
                if (
                    not report
                    or args.graph_report.stat().st_mtime < started_wall
                    or report.get("exit_code") != 0
                    or report.get("progress_m", 0) < 120
                ):
                    errors.append("bridge disconnected before successful graph completion")
                break
            if isinstance(data, str):
                event = json.loads(data)
                if event.get("op") == "status" and event.get("level", 0) >= 2:
                    errors.append(event.get("message", "bridge error"))
                if event.get("op") != "advertise":
                    continue
                for channel in event["channels"]:
                    topic = channel["topic"]
                    if topic in required and channel["id"] not in subscriptions:
                        sub_id = channel["id"]
                        subscriptions[sub_id] = topic
                        connection.send(
                            json.dumps(
                                {"op": "subscribe", "subscriptions": [{"id": sub_id, "channelId": channel["id"]}]}
                            )
                        )
            elif data[0] == 1:
                topic = subscriptions[struct.unpack_from("<I", data, 1)[0]]
                counts[topic] += 1
                if topic == "/telemetry/summary":
                    msg = deserialize_message(data[13:], Telemetry)
                    saw_rc |= msg.gateway_valid and msg.gateway_mode == "RC"
                    if msg.gateway_valid and msg.gateway_mode == "AUTO":
                        summary_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                        if first_auto is None:
                            first_auto = time.monotonic()
                            if not saw_rc or not all(counts.values()):
                                errors.append("layout streams were not established before AUTO")
                        last_auto = time.monotonic()
    finally:
        connection.close()
    if first_auto is None or last_auto - first_auto < 20:
        errors.append("did not observe a CI-length driving window (at least 20 seconds in AUTO)")
    if not all(counts.values()):
        errors.append("one or more layout topics delivered no messages")
    summary_rate = 0.0
    if len(summary_stamps) > 1 and summary_stamps[-1] > summary_stamps[0]:
        summary_rate = (len(summary_stamps) - 1) / (summary_stamps[-1] - summary_stamps[0])
    if not 9 <= summary_rate <= 11:
        errors.append(f"summary stream rate {summary_rate:.2f} Hz is outside 10 Hz +/-10%")
    args.report.write_text(
        json.dumps({"counts": counts, "errors": errors, "saw_rc": saw_rc, "summary_rate_hz": summary_rate}, indent=2)
        + "\n"
    )
    print(args.report.read_text())
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
