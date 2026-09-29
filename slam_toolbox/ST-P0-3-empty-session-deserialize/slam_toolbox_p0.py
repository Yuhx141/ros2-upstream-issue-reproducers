#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, Reset, SerializePoseGraph
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


CASES = (
    "scan_control",
    "pause_before_first",
    "reset_stale_map",
    "empty_roundtrip",
    "missing_deserialize",
)


def wait_future(node, future, process, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return "process_exit", None
        node._p3_executor.spin_once(timeout_sec=0.05)
        if future.done():
            try:
                return "done", future.result()
            except Exception as exc:  # transport failure is an observation
                return "exception", repr(exc)
    return "timeout", None


def call(node, process, srv_type, name, request, timeout=8.0):
    client = node.create_client(srv_type, name)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and process.poll() is None:
        if client.wait_for_service(timeout_sec=0.1):
            return wait_future(node, client.call_async(request), process, timeout)
    return ("process_exit" if process.poll() is not None else "unavailable"), None


def transition(node, process, transition_id):
    req = ChangeState.Request()
    req.transition.id = transition_id
    status, response = call(node, process, ChangeState, "/slam_toolbox/change_state", req, 15.0)
    return status == "done" and bool(response.success)


def health(node, process):
    status, response = call(node, process, GetState, "/slam_toolbox/get_state", GetState.Request(), 2.0)
    return {
        "ok": status == "done" and process.poll() is None,
        "call": status,
        "state": int(response.current_state.id) if status == "done" else None,
        "returncode": process.poll(),
    }


def paused_parameter(node, process):
    req = GetParameters.Request()
    req.names = ["paused_new_measurements"]
    status, response = call(node, process, GetParameters, "/slam_toolbox/get_parameters", req, 3.0)
    if status != "done" or not response.values:
        return None
    return bool(response.values[0].bool_value)


def map_semantic(message):
    if message is None:
        return None
    info = message.info
    payload = bytes((value + 256) % 256 for value in message.data)
    return {
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin": [
            float(info.origin.position.x),
            float(info.origin.position.y),
            float(info.origin.position.z),
            float(info.origin.orientation.z),
            float(info.origin.orientation.w),
        ],
        "data_sha256": hashlib.sha256(payload).hexdigest(),
    }


def install_tf(node):
    broadcaster = StaticTransformBroadcaster(node)
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in (("odom", "base_footprint"), ("base_footprint", "laser")):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.rotation.w = 1.0
        transforms.append(msg)
    broadcaster.sendTransform(transforms)
    return broadcaster


def publish_scans(node, publisher, process, count=6):
    for _ in range(count):
        if process.poll() is not None:
            break
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -3.141592653589793
        msg.angle_max = 3.141592653589793
        msg.angle_increment = 2.0 * 3.141592653589793 / 359.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [2.0] * 360
        publisher.publish(msg)
        node._p3_executor.spin_once(timeout_sec=0.15)


def wait_for_map(node, process, maps, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and process.poll() is None:
        node._p3_executor.spin_once(timeout_sec=0.1)
        for message in reversed(maps):
            if message.info.width and message.info.height:
                return message
    return None


def make_map_fixture(node, process, maps):
    broadcaster = install_tf(node)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(LaserScan, "/scan", qos)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and node.count_subscribers("/scan") == 0:
        node._p3_executor.spin_once(timeout_sec=0.1)
    time.sleep(0.3)
    publish_scans(node, publisher, process)
    result = wait_for_map(node, process, maps)
    return result, broadcaster, publisher


def run_case(case, repeat, domain, output):
    case_dir = output / f"{case}-{repeat}"
    case_dir.mkdir(parents=True, exist_ok=False)
    log_path = case_dir / "node.log"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    command = [
        "/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node",
        "--ros-args", "-r", "__node:=slam_toolbox",
        "-p", "use_sim_time:=false",
        "-p", "use_map_saver:=false",
        "-p", "enable_interactive_mode:=false",
        "-p", "transform_publish_period:=0.0",
        "-p", "map_update_interval:=0.2",
        "-p", "minimum_time_interval:=0.0",
        "-p", "minimum_travel_distance:=0.0",
        "-p", "minimum_travel_heading:=0.0",
        "-p", "check_min_dist_and_heading_precisely:=true",
        "-p", "scan_queue_size:=10",
    ]

    log_file = log_path.open("w")
    process = subprocess.Popen(
        command, stdout=log_file, stderr=subprocess.STDOUT, env=env, start_new_session=True
    )
    context = Context()
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    rclpy.init(context=context)
    node = Node(f"p3_slam_probe_{domain}", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    node._p3_executor = executor
    maps = []
    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    map_subscription = node.create_subscription(OccupancyGrid, "/map", maps.append, map_qos)
    observation = {"case": case, "repeat": repeat, "domain": domain}
    status = "execution_error"
    checks = {}

    try:
        configured = transition(node, process, Transition.TRANSITION_CONFIGURE)
        activated = configured and transition(node, process, Transition.TRANSITION_ACTIVATE)
        observation["lifecycle"] = {"configured": configured, "activated": activated}
        if not activated:
            observation["error"] = "lifecycle transition failed"
        elif case == "scan_control":
            mapped, broadcaster, publisher = make_map_fixture(node, process, maps)
            observed = map_semantic(mapped)
            live = health(node, process)
            observation.update({"map": observed, "health": live})
            checks = {"map_created": observed is not None, "server_healthy": live["ok"]}
            status = "pass" if all(checks.values()) else "execution_error"

        elif case == "pause_before_first":
            pause_status, pause_response = call(
                node, process, Pause, "/slam_toolbox/pause_new_measurements", Pause.Request(), 4.0
            )
            paused = paused_parameter(node, process)
            mapped, broadcaster, publisher = make_map_fixture(node, process, maps)
            observed = map_semantic(mapped)
            live = health(node, process)
            observation.update({
                "pause_call": pause_status,
                "pause_status": bool(pause_response.status) if pause_status == "done" else None,
                "paused_parameter": paused,
                "map": observed,
                "health": live,
            })
            apparatus = pause_status == "done" and bool(pause_response.status) and paused is True and live["ok"]
            checks = {"pause_established": apparatus, "no_map_after_pause": observed is None, "server_healthy": live["ok"]}
            status = "pass" if apparatus and observed is None else ("violation" if apparatus and observed else "execution_error")

        elif case == "reset_stale_map":
            mapped, broadcaster, publisher = make_map_fixture(node, process, maps)
            before = map_semantic(mapped)
            reset_request = Reset.Request()
            reset_request.pause_new_measurements = False
            reset_status, reset_response = call(
                node, process, Reset, "/slam_toolbox/reset", reset_request, 5.0
            )
            time.sleep(0.7)
            get_status, get_response = call(
                node, process, GetMap, "/slam_toolbox/dynamic_map", GetMap.Request(), 4.0
            )
            after = map_semantic(get_response.map) if get_status == "done" else None
            live = health(node, process)
            observation.update({
                "before": before,
                "reset_call": reset_status,
                "reset_result": int(reset_response.result) if reset_status == "done" else None,
                "dynamic_map_call": get_status,
                "after": after,
                "health": live,
            })
            apparatus = before is not None and reset_status == "done" and int(reset_response.result) == 0 and live["ok"]
            stale = apparatus and get_status == "done" and after == before
            checks = {"fixture_created": before is not None, "reset_succeeded": apparatus, "old_map_not_returned": not stale, "server_healthy": live["ok"]}
            status = "violation" if stale else ("pass" if apparatus and get_status == "done" and after is None else "execution_error")

        elif case == "empty_roundtrip":
            base = case_dir / "empty"
            request = SerializePoseGraph.Request()
            request.filename = str(base)
            write_status, write_response = call(
                node, process, SerializePoseGraph, "/slam_toolbox/serialize_map", request, 8.0
            )
            files = {
                "posegraph": (base.with_suffix(".posegraph")).exists(),
                "data": (base.with_suffix(".data")).exists(),
            }
            serialize_success = write_status == "done" and int(write_response.result) == 0 and all(files.values())
            read_status = None
            if serialize_success:
                read_request = DeserializePoseGraph.Request()
                read_request.filename = str(base)
                read_request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
                read_status, _ = call(
                    node, process, DeserializePoseGraph, "/slam_toolbox/deserialize_map", read_request, 8.0
                )
                time.sleep(0.5)
            live = health(node, process) if process.poll() is None else {
                "ok": False, "call": "process_exit", "state": None, "returncode": process.poll()
            }
            observation.update({
                "serialize_call": write_status,
                "serialize_result": int(write_response.result) if write_status == "done" else None,
                "files": files,
                "deserialize_call": read_status,
                "health": live,
            })
            crashed = serialize_success and process.poll() is not None
            checks = {"serialize_success": serialize_success, "deserializer_survived": not crashed}
            status = "violation" if crashed else ("pass" if serialize_success and live["ok"] else "execution_error")

        elif case == "missing_deserialize":
            request = DeserializePoseGraph.Request()
            request.filename = str(case_dir / "does-not-exist")
            request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
            read_status, _ = call(
                node, process, DeserializePoseGraph, "/slam_toolbox/deserialize_map", request, 5.0
            )
            live = health(node, process)
            observation.update({"deserialize_call": read_status, "health": live})
            checks = {"request_completed": read_status == "done", "server_healthy": live["ok"]}
            status = "pass" if all(checks.values()) else "execution_error"

        observation["checks"] = checks
        observation["status"] = status
        return observation
    except Exception as exc:
        observation["error"] = repr(exc)
        observation["checks"] = checks
        observation["status"] = "execution_error"
        return observation
    finally:
        node.destroy_subscription(map_subscription)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        if old_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = old_domain
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
        observation["process_returncode"] = process.returncode
        log_file.close()
        (case_dir / "result.json").write_text(json.dumps(observation, indent=2, sort_keys=True) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--domain-base", type=int, default=170)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    records = []
    domain = args.domain_base
    for case in args.cases:
        for repeat in range(1, args.repetitions + 1):
            record = run_case(case, repeat, domain, args.output)
            records.append(record)
            print(json.dumps({"case": case, "repeat": repeat, "status": record["status"]}), flush=True)
            domain += 1

    counts = {key: sum(record["status"] == key for record in records) for key in ("pass", "violation", "execution_error")}
    summary = {"schema": "p3-slam-toolbox-contract-v1", "counts": counts, "records": records}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    with (args.output / "results.jsonl").open("w") as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
    print(json.dumps(counts, sort_keys=True))


if __name__ == "__main__":
    main()
