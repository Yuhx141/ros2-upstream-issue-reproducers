#!/usr/bin/env python3
"""Small source-guided ros_gz_bridge image-stride regression."""

import argparse
import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from map_semantic_regression import atomic_json, terminate


def wait_for(node, predicate, timeout=8.0):
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def pixels(sample):
    if sample is None:
        return None
    width, height, step, data = (
        sample["width"], sample["height"], sample["step"], sample["data"])
    if step < width or len(data) < step * height:
        return None
    return [value for row in range(height)
            for value in data[row * step:row * step + width]]


def run_ros_to_gz(directory, repeat):
    import rclpy
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    topic = f"/p3_image_r2g_{repeat}_{os.getpid()}"
    log = (directory / "ros_to_gz.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         f"{topic}@sensor_msgs/msg/Image]gz.msgs.Image"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = Node(f"p3_image_r2g_{repeat}_{os.getpid()}")
    received = []
    gz_node = GzNode()
    gz_node.subscribe(
        GzImage, topic,
        lambda msg: received.append({"width": msg.width, "height": msg.height,
                                     "step": msg.step, "data": list(msg.data)}))
    publisher = node.create_publisher(Image, topic, 10)

    def send(step, data, marker):
        message = Image()
        message.width = 2
        message.height = 2
        message.encoding = "mono8"
        message.step = step
        message.data = data
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            for sample in received:
                if sample["data"][:2] == marker:
                    return sample
        return None

    try:
        if not wait_for(node, lambda: publisher.get_subscription_count() > 0):
            raise RuntimeError("ROS-to-Gazebo bridge subscription unavailable")
        baseline = send(2, [11, 12, 13, 14], [11, 12])
        padded = send(4, [21, 22, 201, 202, 23, 24, 203, 204], [21, 22])
        return {"baseline": baseline, "padded": padded,
                "baseline_pixels": pixels(baseline), "padded_pixels": pixels(padded)}
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_gz_to_ros(directory, repeat):
    import rclpy
    from gz.msgs10.image_pb2 import Image as GzImage, L_INT8
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    topic = f"/p3_image_g2r_{repeat}_{os.getpid()}"
    log = (directory / "gz_to_ros.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         f"{topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = Node(f"p3_image_g2r_{repeat}_{os.getpid()}")
    received = []
    node.create_subscription(
        Image, topic,
        lambda msg: received.append({"width": msg.width, "height": msg.height,
                                     "step": msg.step, "data": list(msg.data)}), 10)
    gz_node = GzNode()
    publisher = gz_node.advertise(topic, GzImage)

    def send(step, data, marker):
        message = GzImage()
        message.width = 2
        message.height = 2
        message.step = step
        message.pixel_format_type = L_INT8
        message.data = bytes(data)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and process.poll() is None:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            for sample in received:
                if sample["data"][:2] == marker:
                    return sample
        return None

    try:
        if not wait_for(node, publisher.has_connections):
            raise RuntimeError("Gazebo-to-ROS bridge subscription unavailable")
        baseline = send(2, [31, 32, 33, 34], [31, 32])
        padded = send(4, [41, 42, 211, 212, 43, 44, 213, 214], [41, 42])
        time.sleep(0.1)
        return {"baseline": baseline, "padded": padded,
                "baseline_pixels": pixels(baseline), "padded_pixels": pixels(padded),
                "bridge_exit_after_padded": process.poll()}
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_once(directory, repeat):
    ros_to_gz = run_ros_to_gz(directory, repeat)
    gz_to_ros = run_gz_to_ros(directory, repeat)
    checks = {
        "ros_to_gz_tight_control": ros_to_gz["baseline_pixels"] == [11, 12, 13, 14],
        "ros_to_gz_padded_pixels_preserved": ros_to_gz["padded_pixels"] == [21, 22, 23, 24],
        "gz_to_ros_tight_control": gz_to_ros["baseline_pixels"] == [31, 32, 33, 34],
        "gz_to_ros_padded_pixels_preserved": gz_to_ros["padded_pixels"] == [41, 42, 43, 44],
    }
    return {"ros_to_gz": ros_to_gz, "gz_to_ros": gz_to_ros, "checks": checks,
            "controls_pass": checks["ros_to_gz_tight_control"] and
            checks["gz_to_ros_tight_control"],
            "violations": [name for name, passed in checks.items()
                           if not passed and "padded" in name]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("repetitions must be positive")
    root = args.out_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).resolve()
    source_value = os.environ.get("ROS_GZ_SOURCE_ROOT")
    source = (Path(source_value) / "ros_gz_bridge/src/convert/sensor_msgs.cpp" if source_value else None)
    manifest = {
        "schema": 1,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "repetitions": args.repetitions,
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "source_sha256": (hashlib.sha256(source.read_bytes()).hexdigest() if source and source.exists() else None),
        "package": subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Package} ${Version}",
             "ros-jazzy-ros-gz-bridge"], text=True),
        "obligations": ["ros_to_gz_tight_control", "ros_to_gz_padded_pixels",
                        "gz_to_ros_tight_control", "gz_to_ros_padded_pixels"],
    }
    atomic_json(root / "manifest.json", manifest)
    os.environ["GZ_PARTITION"] = f"p3-image-semantic-{os.getpid()}"

    import rclpy
    rclpy.init()
    results = []
    try:
        for repeat in range(args.repetitions):
            directory = root / f"repeat-{repeat:02d}"
            directory.mkdir()
            try:
                result = {"repeat": repeat, "status": "observed",
                          "observation": run_once(directory, repeat)}
            except Exception as error:
                result = {"repeat": repeat, "status": "execution_error", "error": repr(error)}
            results.append(result)
            atomic_json(directory / "result.json", result)
            atomic_json(root / "progress.json", {"manifest": manifest, "results": results})
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()

    observed = [item["observation"] for item in results if item["status"] == "observed"]
    summary = {
        "manifest": manifest,
        "completed": len(results),
        "execution_errors": sum(item["status"] != "observed" for item in results),
        "control_failures": sum(not item["controls_pass"] for item in observed),
        "ros_to_gz_padded_violations": sum(
            "ros_to_gz_padded_pixels_preserved" in item["violations"] for item in observed),
        "gz_to_ros_padded_violations": sum(
            "gz_to_ros_padded_pixels_preserved" in item["violations"] for item in observed),
        "results": results,
    }
    atomic_json(root / "summary.json", summary)
    print(json.dumps({key: summary[key] for key in (
        "completed", "execution_errors", "control_failures",
        "ros_to_gz_padded_violations", "gz_to_ros_padded_violations")}), flush=True)


if __name__ == "__main__":
    main()
