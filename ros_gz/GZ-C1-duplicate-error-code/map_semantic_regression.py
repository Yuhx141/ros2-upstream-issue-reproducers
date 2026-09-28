#!/usr/bin/env python3
"""Deterministic Nav2 MapIO and Map Server semantic regressions."""

import argparse
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


def atomic_json(path, value):
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def terminate(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def write_map(directory, name, pixels, width, height, mode="trinary", free=0.196,
              occupied=0.65, negate=0, origin=(1.25, -2.5, 0.0)):
    image = directory / f"{name}.pgm"
    yaml = directory / f"{name}.yaml"
    image.write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes(pixels))
    yaml.write_text(
        f"image: {image}\nresolution: 0.05\norigin: {list(origin)}\n"
        f"negate: {negate}\noccupied_thresh: {occupied}\nfree_thresh: {free}\nmode: {mode}\n",
        encoding="utf-8",
    )
    return yaml


def write_rgba_map(directory, name, gray, alpha, mode="scale", free=0.2,
                   occupied=0.8, negate=0, origin=(1.25, -2.5, 0.7)):
    from PIL import Image

    image = directory / f"{name}.png"
    yaml = directory / f"{name}.yaml"
    rgba = Image.new("RGBA", (len(gray), 1))
    rgba.putdata([(value, value, value, opacity) for value, opacity in zip(gray, alpha)])
    rgba.save(image)
    yaml.write_text(
        f"image: {image}\nresolution: 0.05\norigin: {list(origin)}\n"
        f"negate: {negate}\noccupied_thresh: {occupied}\nfree_thresh: {free}\nmode: {mode}\n",
        encoding="utf-8",
    )
    return yaml


def expected_pixels(pixels, mode, free=0.2, occupied=0.8, negate=0, alpha=None):
    expected = []
    for index, pixel in enumerate(pixels):
        if mode == "raw":
            value = pixel if pixel <= 100 else -1
        else:
            probability = pixel / 255.0 if negate else 1.0 - pixel / 255.0
            if probability >= occupied:
                value = 100
            elif probability <= free:
                value = 0
            elif mode == "trinary":
                value = -1
            else:
                value = math.floor(
                    (probability - free) / (occupied - free) * 100.0 + 0.5)
        if alpha is not None and alpha[index] < 255 and mode != "raw":
            value = -1
        expected.append(value)
    return expected


def semantic(grid):
    return {
        "width": grid.info.width,
        "height": grid.info.height,
        "resolution": grid.info.resolution,
        "origin": [
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            grid.info.origin.position.z,
            grid.info.origin.orientation.x,
            grid.info.origin.orientation.y,
            grid.info.origin.orientation.z,
            grid.info.origin.orientation.w,
        ],
        "frame_id": grid.header.frame_id,
        "data": list(grid.data),
        "stamp": [grid.header.stamp.sec, grid.header.stamp.nanosec],
        "map_load_time": [grid.info.map_load_time.sec, grid.info.map_load_time.nanosec],
    }


def content(value):
    return {key: item for key, item in value.items() if key not in ("stamp", "map_load_time")}


def call(node, client, request, timeout=10):
    import rclpy

    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"service call failed: {client.srv_name}")
    return future.result()


def wait_sample(node, samples, predicate, timeout=5):
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for sample in reversed(samples):
            if predicate(sample):
                return sample
    raise RuntimeError("map topic sample unavailable")


def save_roundtrip(node, directory, index, source_data, mode):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    topic = f"/p3_map_semantic_{index}/{mode}_source"
    output = directory / mode
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(OccupancyGrid, topic, qos)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 0.05
    message.info.width = len(source_data)
    message.info.height = 1
    message.info.origin.position.x = 1.25
    message.info.origin.position.y = -2.5
    message.info.origin.orientation.w = 1.0
    message.data = source_data
    log = (directory / "map-saver.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-t", topic, "-f", str(output), "--occ", "0.65", "--free", "0.25",
            "--fmt", "png", "--mode", mode,
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 15
        while process.poll() is None and time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
        if process.poll() is None:
            raise RuntimeError("map_saver_cli timeout")
        if process.returncode != 0:
            raise RuntimeError(f"map_saver_cli exited {process.returncode}")
        if not output.with_suffix(".yaml").is_file() or not output.with_suffix(".png").is_file():
            raise RuntimeError(f"map_saver_cli did not create {mode} files")
        return output.with_suffix(".yaml"), process.pid
    finally:
        terminate(process)
        log.close()
        node.destroy_publisher(publisher)


def run_once(root, index):
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetMap
    from nav2_msgs.srv import LoadMap
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    directory = root / f"repeat-{index:02d}"
    directory.mkdir()
    source_data = [-1, 0, 1, 25, 42, 50, 65, 99, 100]
    map_a = write_map(directory, "a", [0, 255, 255, 0], 2, 2)
    map_b = write_map(directory, "b", [255, 0, 0, 255], 2, 2)
    boundary = write_map(directory, "boundary", [51, 204, 50, 205], 4, 1,
                         free=0.2, occupied=0.8)
    gray = [0, 1, 50, 51, 52, 127, 203, 204, 205, 254, 255]
    semantic_maps = {
        "trinary_gray": write_map(directory, "trinary-gray", gray, len(gray), 1,
                                   free=0.2, occupied=0.8, origin=(1.25, -2.5, 0.7)),
        "scale_gray": write_map(directory, "scale-gray", gray, len(gray), 1, mode="scale",
                                free=0.2, occupied=0.8, origin=(1.25, -2.5, 0.7)),
        "scale_negated": write_map(directory, "scale-negated", gray, len(gray), 1,
                                   mode="scale", free=0.2, occupied=0.8, negate=1,
                                   origin=(1.25, -2.5, 0.7)),
        "raw_gray": write_map(directory, "raw-gray", gray, len(gray), 1, mode="raw",
                              free=0.2, occupied=0.8, origin=(1.25, -2.5, 0.7)),
    }
    alpha_gray = [0, 255, 127, 50]
    alpha = [255, 255, 254, 0]
    semantic_maps["scale_alpha"] = write_rgba_map(
        directory, "scale-alpha", alpha_gray, alpha)
    node = Node(f"p3_map_semantic_observer_{index}_{os.getpid()}")
    server = None
    server_log = None
    started = time.monotonic()
    try:
        scale_yaml, scale_saver_pid = save_roundtrip(
            node, directory, index, source_data, "scale")
        raw_yaml, raw_saver_pid = save_roundtrip(
            node, directory, index, source_data, "raw")
        server_name = f"p3_map_semantic_server_{index}_{os.getpid()}"
        topic = f"/p3_map_semantic_{index}/map"
        server_log = (directory / "map-server.log").open("w", encoding="utf-8")
        server = subprocess.Popen(
            [
                "ros2", "run", "nav2_map_server", "map_server", "--ros-args",
                "-r", f"__node:={server_name}", "-r", f"map:={topic}",
                "-p", f"yaml_filename:={map_a}",
            ],
            stdout=server_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        prefix = f"/{server_name}/"
        change = node.create_client(ChangeState, prefix + "change_state")
        get_map = node.create_client(GetMap, prefix + "map")
        load_map = node.create_client(LoadMap, prefix + "load_map")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        samples = []
        subscription = node.create_subscription(
            OccupancyGrid, topic, lambda message: samples.append(semantic(message)), qos)

        def transition(code):
            request = ChangeState.Request()
            request.transition.id = code
            if not call(node, change, request).success:
                raise RuntimeError(f"lifecycle transition {code} rejected")

        def load(path):
            request = LoadMap.Request()
            request.map_url = str(path)
            return call(node, load_map, request)

        transition(Transition.TRANSITION_CONFIGURE)
        transition(Transition.TRANSITION_ACTIVATE)
        initial = semantic(call(node, get_map, GetMap.Request()).map)
        expected_a = [0, 100, 100, 0]

        samples.clear()
        response_b = load(map_b)
        returned_b = semantic(response_b.map)
        expected_b = [100, 0, 0, 100]
        topic_b = wait_sample(node, samples, lambda value: value["data"] == expected_b)
        get_b = semantic(call(node, get_map, GetMap.Request()).map)
        late = []
        late_subscription = node.create_subscription(
            OccupancyGrid, topic, lambda message: late.append(semantic(message)), qos)
        late_b = wait_sample(node, late, lambda value: value["data"] == expected_b)

        samples.clear()
        missing_response = load(directory / "missing.yaml")
        after_missing = semantic(call(node, get_map, GetMap.Request()).map)
        deadline = time.monotonic() + 0.75
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        post_failure_samples = list(samples)

        boundary_response = load(boundary)
        boundary_map = semantic(boundary_response.map)
        scale_response = load(scale_yaml)
        scale_map = semantic(scale_response.map)
        raw_response = load(raw_yaml)
        raw_map = semantic(raw_response.map)
        semantic_results = {}
        semantic_expected = {
            "trinary_gray": expected_pixels(gray, "trinary"),
            "scale_gray": expected_pixels(gray, "scale"),
            "scale_negated": expected_pixels(gray, "scale", negate=1),
            "raw_gray": expected_pixels(gray, "raw"),
            "scale_alpha": expected_pixels(alpha_gray, "scale", alpha=alpha),
        }
        expected_origin = [1.25, -2.5, 0, 0, 0, math.sin(0.35), math.cos(0.35)]
        for name, path in semantic_maps.items():
            response = load(path)
            observed = semantic(response.map)
            semantic_results[name] = {
                "success": response.result == LoadMap.Response.RESULT_SUCCESS,
                "loaded": observed["data"],
                "expected": semantic_expected[name],
                "data_match": observed["data"] == semantic_expected[name],
                "geometry_match": (
                    observed["width"] == len(semantic_expected[name]) and
                    observed["height"] == 1 and
                    abs(observed["resolution"] - 0.05) < 1e-6 and
                    max(abs(a - b) for a, b in zip(observed["origin"], expected_origin)) < 1e-6
                ),
            }

        scale_errors = []
        for position, (before, after) in enumerate(zip(source_data, scale_map["data"])):
            if (before == -1 and after != -1) or (before >= 0 and abs(before - after) > 1):
                scale_errors.append({"index": position, "before": before, "after": after})
        geometry_ok = (
            scale_map["width"] == len(source_data) and scale_map["height"] == 1 and
            abs(scale_map["resolution"] - 0.05) < 1e-6 and
            max(abs(a - b) for a, b in zip(scale_map["origin"], [1.25, -2.5, 0, 0, 0, 0, 1])) < 1e-6
        )
        interface_checks = {
            "initial_matches_input_a": initial["data"] == expected_a,
            "load_b_success": response_b.result == LoadMap.Response.RESULT_SUCCESS,
            "load_response_equals_get_map": content(returned_b) == content(get_b),
            "load_response_equals_topic": content(returned_b) == content(topic_b),
            "late_subscriber_gets_b": content(returned_b) == content(late_b),
            "failed_load_rejected": missing_response.result != LoadMap.Response.RESULT_SUCCESS,
            "failed_load_preserves_b": content(after_missing) == content(get_b),
            "failed_load_does_not_publish": len(post_failure_samples) == 0,
        }
        boundary_data = boundary_map["data"]
        result = {
            "status": "observed",
            "repeat": index,
            "duration_s": time.monotonic() - started,
            "pids": {"scale_map_saver": scale_saver_pid,
                     "raw_map_saver": raw_saver_pid, "map_server": server.pid},
            "scale_roundtrip": {
                "source": source_data,
                "loaded": scale_map["data"],
                "errors": scale_errors,
                "geometry_ok": geometry_ok,
                "violation": bool(scale_errors) or not geometry_ok,
                "yaml": scale_yaml.read_text(encoding="utf-8"),
                "png_sha256": hashlib.sha256(scale_yaml.with_suffix(".png").read_bytes()).hexdigest(),
            },
            "raw_roundtrip": {
                "source": source_data,
                "loaded": raw_map["data"],
                "exact": raw_map["data"] == source_data,
            },
            "load_semantics": {
                "cases": semantic_results,
                "pass": all(item["success"] and item["data_match"] and item["geometry_match"]
                            for item in semantic_results.values()),
            },
            "threshold_boundary": {
                "pixels": [51, 204, 50, 205],
                "loaded": boundary_data,
                "inclusive_expected": [100, 0, 100, 0],
                "strict_expected": [-1, -1, 100, 0],
                "matches_inclusive": boundary_data == [100, 0, 100, 0],
                "matches_strict": boundary_data == [-1, -1, 100, 0],
            },
            "interface_state": {
                "checks": interface_checks,
                "violation": not all(interface_checks.values()),
                "initial": initial,
                "load_b_response": returned_b,
                "get_b": get_b,
                "topic_b": topic_b,
                "late_b": late_b,
                "missing_result": missing_response.result,
                "missing_result_is_does_not_exist": (
                    missing_response.result == LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
                ),
                "after_missing": after_missing,
                "post_failure_topic_samples": post_failure_samples,
            },
        }
        atomic_json(directory / "result.json", result)
        node.destroy_subscription(late_subscription)
        node.destroy_subscription(subscription)
        try:
            transition(Transition.TRANSITION_DEACTIVATE)
            transition(Transition.TRANSITION_CLEANUP)
        except Exception:
            pass
        return result
    finally:
        terminate(server)
        if server_log:
            server_log.close()
        node.destroy_node()


def write_summary(root, manifest, results, started):
    complete = [result for result in results if result["status"] == "observed"]
    summary = {
        "manifest": manifest,
        "started_at": started,
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "completed": len(results),
        "observed": len(complete),
        "execution_errors": sum(result["status"] == "execution_error" for result in results),
        "scale_violations": sum(result.get("scale_roundtrip", {}).get("violation", False) for result in complete),
        "raw_roundtrip_failures": sum(
            not result.get("raw_roundtrip", {}).get("exact", False) for result in complete),
        "load_semantic_failures": sum(
            not result.get("load_semantics", {}).get("pass", False) for result in complete),
        "interface_violations": sum(result.get("interface_state", {}).get("violation", False) for result in complete),
        "boundary_inclusive": sum(result.get("threshold_boundary", {}).get("matches_inclusive", False) for result in complete),
        "boundary_strict": sum(result.get("threshold_boundary", {}).get("matches_strict", False) for result in complete),
        "results": results,
    }
    atomic_json(root / "progress.json", summary)
    lines = [
        "# Map semantic directed regression", "",
        f"- completed: {summary['completed']} / {manifest['repetitions']}",
        f"- observed: {summary['observed']}",
        f"- execution errors: {summary['execution_errors']}",
        f"- Scale round-trip violations: {summary['scale_violations']}",
        f"- Raw round-trip failures: {summary['raw_roundtrip_failures']}",
        f"- load-semantic failures: {summary['load_semantic_failures']}",
        f"- interface/state violations: {summary['interface_violations']}",
        f"- boundary matches inclusive semantics: {summary['boundary_inclusive']}",
        f"- boundary matches pre-refactor strict semantics: {summary['boundary_strict']}", "",
    ]
    for result in results:
        if result["status"] == "observed":
            lines.append(
                f"- repeat {result['repeat']:02d}: scale={result['scale_roundtrip']['loaded']}; "
                f"raw_exact={result['raw_roundtrip']['exact']}; "
                f"load_semantics={result['load_semantics']['pass']}; "
                f"boundary={result['threshold_boundary']['loaded']}; "
                f"interface_violation={result['interface_state']['violation']}"
            )
        else:
            lines.append(f"- repeat {result['repeat']:02d}: execution_error={result['error']}")
    (root / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("repetitions must be positive")
    args.out_dir = args.out_dir.resolve()
    script = Path(__file__).resolve()
    script_sha = hashlib.sha256(script.read_bytes()).hexdigest()
    map_io_sha = hashlib.sha256(Path("/opt/ros/jazzy/lib/libmap_io.so").read_bytes()).hexdigest()
    package = subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Package} ${Version}", "ros-jazzy-nav2-map-server"],
        text=True,
    )
    if args.out_dir.exists():
        if not args.resume:
            parser.error("out-dir exists; pass --resume to continue it")
        manifest = json.loads((args.out_dir / "manifest.json").read_text(encoding="utf-8"))
        if (manifest["repetitions"] != args.repetitions or
                manifest["script_sha256"] != script_sha or
                manifest["map_io_sha256"] != map_io_sha or manifest["package"] != package):
            parser.error("resume manifest does not match repetitions, script, or installed MapIO")
        started = manifest["started_at"]
    else:
        args.out_dir.mkdir(parents=True)
        started = datetime.now(timezone.utc).isoformat()
        manifest = {
        "schema": 1,
        "name": "map-semantic-directed-v2",
        "started_at": started,
        "repetitions": args.repetitions,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "script": str(script),
        "script_sha256": script_sha,
        "map_io_sha256": map_io_sha,
        "package": package,
        "cases": ["scale_roundtrip", "raw_roundtrip", "threshold_boundary",
                  "gray_scale_raw_negate_alpha_origin", "interface_state"],
        }
        atomic_json(args.out_dir / "manifest.json", manifest)
    results = {}
    for index in range(args.repetitions):
        path = args.out_dir / f"repeat-{index:02d}" / "result.json"
        if path.is_file():
            results[index] = json.loads(path.read_text(encoding="utf-8"))
        elif path.parent.exists():
            shutil.rmtree(path.parent)
    import rclpy

    rclpy.init()
    try:
        for index in range(args.repetitions):
            if index in results:
                continue
            try:
                result = run_once(args.out_dir, index)
            except Exception as error:
                result = {"status": "execution_error", "repeat": index, "error": repr(error)}
                directory = args.out_dir / f"repeat-{index:02d}"
                directory.mkdir(exist_ok=True)
                atomic_json(directory / "result.json", result)
            results[index] = result
            ordered = [results[key] for key in sorted(results)]
            write_summary(args.out_dir, manifest, ordered, started)
            print(json.dumps({
                "repeat": index,
                "status": result["status"],
                "scale_violation": result.get("scale_roundtrip", {}).get("violation"),
                "interface_violation": result.get("interface_state", {}).get("violation"),
            }), flush=True)
    finally:
        rclpy.shutdown()
    ordered = [results[key] for key in sorted(results)]
    summary = write_summary(args.out_dir, manifest, ordered, started)
    atomic_json(args.out_dir / "summary.json", summary)


if __name__ == "__main__":
    main()
