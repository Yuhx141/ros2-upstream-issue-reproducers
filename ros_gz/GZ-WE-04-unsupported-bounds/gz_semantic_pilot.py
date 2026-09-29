#!/usr/bin/env python3
"""Small source-guided ResetSimulation and StepSimulation regression."""

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

from map_semantic_regression import atomic_json, terminate


def call(node, client, request, timeout=15.0):
    import rclpy

    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"service call failed: {client.srv_name}")
    return future.result()


def snapshot(response):
    state = response.state
    values = [state.pose.position.x, state.pose.position.y, state.pose.position.z,
              state.twist.linear.x, state.twist.linear.y, state.twist.linear.z]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("non-finite entity state")
    return {"time": state.header.stamp.sec + state.header.stamp.nanosec / 1e9,
            "position": values[:3], "velocity": values[3:]}


def state_delta(left, right):
    return max(abs(a - b) for key in ("position", "velocity")
               for a, b in zip(left[key], right[key]))


def duplicate_time_delta(samples):
    return max((state_delta(left, right)
                for index, left in enumerate(samples)
                for right in samples[index + 1:]
                if abs(left["time"] - right["time"]) <= 1e-12), default=0.0)


def wait_state(node, client, expected, timeout=8.0):
    from simulation_interfaces.srv import GetSimulationState

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        actual = call(node, client, GetSimulationState.Request()).state.state
        if actual == expected:
            return actual
        time.sleep(0.01)
    raise RuntimeError(f"simulation state {expected} unavailable")


def set_state(node, client, get_client, value):
    from simulation_interfaces.srv import SetSimulationState

    request = SetSimulationState.Request()
    request.state.state = value
    response = call(node, client, request)
    wait_state(node, get_client, value)
    return response.result.result


def server(world, log_path, partition):
    environment = os.environ.copy()
    environment["GZ_PARTITION"] = partition
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "ros_gz_sim", "gzserver", "--ros-args", "-p",
         f"world_sdf_file:={world}"], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, env=environment)
    return process, log


def run_step(directory, repeat, world, step_size):
    from rclpy.node import Node
    from simulation_interfaces.msg import Result, SimulationState
    from simulation_interfaces.srv import (
        GetEntityState, GetSimulationState, SetSimulationState, StepSimulation)

    process, log = server(world, directory / "step.log", f"p3-step-semantic-{os.getpid()}-{repeat}")
    node = Node(f"p3_step_semantic_{repeat}_{os.getpid()}")
    try:
        get_state = node.create_client(GetSimulationState, "/gzserver/get_simulation_state")
        set_state_client = node.create_client(SetSimulationState, "/gzserver/set_simulation_state")
        entity = node.create_client(GetEntityState, "/gzserver/get_entity_state")
        step = node.create_client(StepSimulation, "/gzserver/step_simulation")
        entity_request = GetEntityState.Request()
        entity_request.entity = "sphere"
        wait_state(node, get_state, SimulationState.STATE_PLAYING)

        playing_request = StepSimulation.Request()
        playing_request.steps = 1
        started = time.monotonic()
        playing_response = call(node, step, playing_request)
        playing_wall = time.monotonic() - started
        wait_state(node, get_state, SimulationState.STATE_PAUSED)

        overflow_request = StepSimulation.Request()
        overflow_request.steps = 2 ** 32
        started = time.monotonic()
        overflow_response = call(node, step, overflow_request)
        overflow_wall = time.monotonic() - started

        before = snapshot(call(node, entity, entity_request))
        complete_request = StepSimulation.Request()
        complete_request.steps = 100
        started = time.monotonic()
        complete_response = call(node, step, complete_request)
        response_wall = time.monotonic() - started
        first = snapshot(call(node, entity, entity_request))
        target = before["time"] + complete_request.steps * step_size
        final = first
        deadline = time.monotonic() + 3
        while final["time"] < target - 1e-9 and time.monotonic() < deadline:
            time.sleep(0.02)
            final = snapshot(call(node, entity, entity_request))
        state_after = call(node, get_state, GetSimulationState.Request()).state.state
        checks = {
            "playing_request_rejected":
            playing_response.result.result == Result.RESULT_OPERATION_FAILED,
            "uint32_overflow_rejected":
            overflow_response.result.result == Result.RESULT_OPERATION_FAILED,
            "paused_request_accepted": complete_response.result.result == Result.RESULT_OK,
            "steps_complete_before_response": first["time"] >= target - 1e-9,
            "steps_eventually_complete": final["time"] >= target - 1e-9,
            "paused_after_steps": state_after == SimulationState.STATE_PAUSED,
        }
        return {"playing": {"code": playing_response.result.result, "wall_s": playing_wall},
                "overflow": {"code": overflow_response.result.result, "wall_s": overflow_wall},
                "completion": {"before": before, "first": first, "final": final,
                               "target": target, "response_code": complete_response.result.result,
                               "response_wall_s": response_wall, "state_after": state_after},
                "checks": checks}
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_reset(directory, repeat, world):
    from rclpy.node import Node
    from simulation_interfaces.msg import Result, SimulationState
    from simulation_interfaces.srv import (
        GetEntityState, GetSimulationState, ResetSimulation, SetSimulationState)

    process, log = server(world, directory / "reset.log", f"p3-reset-semantic-{os.getpid()}-{repeat}")
    node = Node(f"p3_reset_semantic_{repeat}_{os.getpid()}")
    try:
        get_state = node.create_client(GetSimulationState, "/gzserver/get_simulation_state")
        set_state_client = node.create_client(SetSimulationState, "/gzserver/set_simulation_state")
        entity = node.create_client(GetEntityState, "/gzserver/get_entity_state")
        reset = node.create_client(ResetSimulation, "/gzserver/reset_simulation")
        entity_request = GetEntityState.Request()
        entity_request.entity = "sphere"
        wait_state(node, get_state, SimulationState.STATE_PLAYING)
        deadline = time.monotonic() + 8
        while True:
            current = snapshot(call(node, entity, entity_request))
            if current["time"] >= 0.1:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("simulation advance timeout")
        set_state(node, set_state_client, get_state, SimulationState.STATE_PAUSED)
        unsupported_before = snapshot(call(node, entity, entity_request))
        unsupported_request = ResetSimulation.Request()
        unsupported_request.scope = ResetSimulation.Request.SCOPE_TIME
        started = time.monotonic()
        unsupported_response = call(node, reset, unsupported_request)
        unsupported_wall = time.monotonic() - started
        unsupported_after = snapshot(call(node, entity, entity_request))

        baseline = [snapshot(call(node, entity, entity_request)) for _ in range(3)]
        reset_request = ResetSimulation.Request()
        reset_request.scope = ResetSimulation.Request.SCOPE_ALL
        started = time.monotonic()
        reset_response = call(node, reset, reset_request)
        reset_wall = time.monotonic() - started
        state_after = call(node, get_state, GetSimulationState.Request()).state.state
        post = [snapshot(call(node, entity, entity_request)) for _ in range(3)]
        duplicate_delta = duplicate_time_delta(post)
        checks = {
            "unsupported_scope_reports_feature_unsupported":
            unsupported_response.result.result == Result.RESULT_FEATURE_UNSUPPORTED,
            "unsupported_scope_preserves_state":
            abs(unsupported_after["time"] - unsupported_before["time"]) <= 1e-12 and
            state_delta(unsupported_before, unsupported_after) <= 1e-12,
            "unsupported_scope_returns_promptly": unsupported_wall < 0.5,
            "reset_all_accepted": reset_response.result.result == Result.RESULT_OK,
            "paused_after_reset": state_after == SimulationState.STATE_PAUSED,
            "paused_baseline_stable": max(state_delta(baseline[0], item) for item in baseline[1:]) <= 1e-12,
            "equal_timestamps_have_equal_state": duplicate_delta <= 1e-12,
        }
        return {"unsupported": {"code": unsupported_response.result.result,
                                "message": unsupported_response.result.error_message,
                                "wall_s": unsupported_wall,
                                "before": unsupported_before, "after": unsupported_after},
                "reset_all": {"code": reset_response.result.result, "wall_s": reset_wall,
                              "state_after": state_after, "baseline": baseline, "post": post,
                              "duplicate_timestamp_max_state_delta": duplicate_delta},
                "checks": checks,
                "unsupported_latency_source_risk":
                checks["unsupported_scope_reports_feature_unsupported"] and
                not checks["unsupported_scope_returns_promptly"]}
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


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
    repo = Path.cwd()
    worktree = Path.cwd()
    world = worktree / "experiments/p1-gz-state/world.sdf"
    step_size = float(ET.parse(world).findtext(".//physics/max_step_size"))
    source_paths = [
        "ros_gz_sim/src/gz_simulation_interfaces/services/reset_simulation.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/services/step_simulation.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/actions/simulate_steps.cpp"]
    source_hashes = {}
    for path in source_paths:
        content = subprocess.check_output(["git", "-C", str(repo), "show", f"HEAD:{path}"])
        source_hashes[path] = hashlib.sha256(content).hexdigest()
    manifest = {
        "schema": 1, "started_at": datetime.now(timezone.utc).isoformat(),
        "repetitions": args.repetitions,
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "ros_gz_source_head": subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
        "source_sha256": source_hashes,
        "package": subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Package} ${Version}",
             "ros-jazzy-ros-gz-sim"], text=True),
        "step_size_s": step_size,
    }
    atomic_json(root / "manifest.json", manifest)

    import rclpy
    rclpy.init()
    results = []
    try:
        for repeat in range(args.repetitions):
            directory = root / f"repeat-{repeat:02d}"
            directory.mkdir()
            try:
                result = {"repeat": repeat, "status": "observed",
                          "step": run_step(directory, repeat, world, step_size),
                          "reset": run_reset(directory, repeat, world)}
            except Exception as error:
                result = {"repeat": repeat, "status": "execution_error", "error": repr(error)}
            results.append(result)
            atomic_json(directory / "result.json", result)
            atomic_json(root / "progress.json", {"manifest": manifest, "results": results})
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    observed = [item for item in results if item["status"] == "observed"]
    summary = {
        "manifest": manifest, "completed": len(results),
        "execution_errors": sum(item["status"] != "observed" for item in results),
        "step_playing_violations": sum(
            not item["step"]["checks"]["playing_request_rejected"] for item in observed),
        "step_completion_violations": sum(
            not item["step"]["checks"]["steps_complete_before_response"] for item in observed),
        "step_control_failures": sum(not all(
            item["step"]["checks"][key] for key in
            ("uint32_overflow_rejected", "paused_request_accepted",
             "steps_eventually_complete", "paused_after_steps")) for item in observed),
        "reset_atomicity_violations": sum(
            not item["reset"]["checks"]["equal_timestamps_have_equal_state"]
            for item in observed),
        "reset_control_failures": sum(not all(
            item["reset"]["checks"][key] for key in
            ("unsupported_scope_reports_feature_unsupported",
             "unsupported_scope_preserves_state", "reset_all_accepted",
             "paused_after_reset", "paused_baseline_stable")) for item in observed),
        "reset_unsupported_latency_risks": sum(
            item["reset"]["unsupported_latency_source_risk"] for item in observed),
        "results": results,
    }
    atomic_json(root / "summary.json", summary)
    print(json.dumps({key: summary[key] for key in summary if key not in ("manifest", "results")}),
          flush=True)


if __name__ == "__main__":
    main()
