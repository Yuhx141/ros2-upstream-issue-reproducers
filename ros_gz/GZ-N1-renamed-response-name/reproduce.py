#!/usr/bin/env python3
"""Small source-guided checks for Gazebo entity and world services (TODO 21-23)."""

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import time
import traceback
from pathlib import Path

from gz_semantic_pilot import call, server, set_state, snapshot, wait_state
from map_semantic_regression import atomic_json


PARENT_SDF = """<sdf version='1.9'><model name='carrier'><static>true</static>
  <link name='body'/><model name='child'><pose>0.5 0 0 0 0 0</pose>
  <static>true</static><link name='body'/></model></model></sdf>"""


def result(item, title, checks, observations, known=()):
    ok = all(checks.values())
    return {"item": item, "title": title, "status": "observed", "pass": ok,
            "classification": "pass" if ok else "violation", "checks": checks,
            "observations": observations, "known_families": list(known)}


def names(node, client):
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import GetEntitiesStates
    response = call(node, client, GetEntitiesStates.Request())
    if response.result.result != Result.RESULT_OK:
        raise RuntimeError("GetEntitiesStates failed")
    return response


def get(node, client, entity):
    from simulation_interfaces.srv import GetEntityState
    request = GetEntityState.Request()
    request.entity = entity
    return call(node, client, request)


def wait_entity(node, client, entity, present, timeout=5.0):
    from simulation_interfaces.msg import Result
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = get(node, client, entity)
        if (last.result.result == Result.RESULT_OK) == present:
            return last
        time.sleep(0.02)
    return last


def spawn(node, client, name, x, rename=False, frame="world"):
    from simulation_interfaces.srv import SpawnEntity
    request = SpawnEntity.Request()
    request.name = name
    request.allow_renaming = rename
    request.resource_string = PARENT_SDF
    request.initial_pose.header.frame_id = frame
    request.initial_pose.pose.position.x = x
    request.initial_pose.pose.orientation.w = 1.0
    return call(node, client, request)


def delete(node, client, entity):
    from simulation_interfaces.srv import DeleteEntity
    request = DeleteEntity.Request()
    request.entity = entity
    return call(node, client, request)


def set_entity(node, client, entity, x, y=0.0, z=2.0, vx=0.0, frame="world"):
    from simulation_interfaces.srv import SetEntityState
    request = SetEntityState.Request()
    request.entity = entity
    request.state.header.frame_id = frame
    request.state.pose.position.x = x
    request.state.pose.position.y = y
    request.state.pose.position.z = z
    request.state.pose.orientation.w = 1.0
    request.state.twist.linear.x = vx
    return call(node, client, request)


def wait_position(node, client, entity, expected, timeout=5.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        response = get(node, client, entity)
        if response.result.result == 1:
            last = snapshot(response)
            if max(abs(a - b) for a, b in zip(last["position"], expected)) < 0.02:
                return last
        time.sleep(0.02)
    return None


def item21(node, clients):
    from simulation_interfaces.msg import Result
    base = set(names(node, clients["many"]).entities)
    first = spawn(node, clients["spawn"], "p3_parent", 1.0)
    first_state = wait_entity(node, clients["get"], "p3_parent", True)
    duplicate = spawn(node, clients["spawn"], "p3_parent", 4.0)
    renamed = spawn(node, clients["spawn"], "p3_parent", 4.0, True)
    after = set(names(node, clients["many"]).entities)
    added = sorted(after - base - {"p3_parent"})
    actual = added[0] if len(added) == 1 else ""
    child_before = wait_entity(node, clients["get"], "child", True, 1.0)
    removed = delete(node, clients["delete"], actual) if actual else None
    gone = wait_entity(node, clients["get"], actual, False) if actual else None
    repeated = delete(node, clients["delete"], actual) if actual else None
    respawned = spawn(node, clients["spawn"], actual, 7.0) if actual else None
    clean = wait_position(node, clients["get"], actual, [7.0, 0.0, 0.0]) if actual else None
    cleanup_actual = delete(node, clients["delete"], actual) if actual else None
    if actual:
        wait_entity(node, clients["get"], actual, False)
    parent_removed = delete(node, clients["delete"], "p3_parent")
    child_after = wait_entity(node, clients["get"], "child", False, 1.0)
    checks = {
        "unique_spawn_visible": first.result.result == Result.RESULT_OK and first_state.result.result == Result.RESULT_OK,
        "duplicate_reports_name_not_unique": duplicate.result.result == 101,
        "renamed_response_is_actual_unique_name": renamed.result.result == Result.RESULT_OK and renamed.entity_name == actual and bool(actual),
        "delete_removes_actual_entity": removed is not None and removed.result.result == Result.RESULT_OK and gone.result.result != Result.RESULT_OK,
        "repeat_delete_reports_not_found": repeated is not None and repeated.result.result == Result.RESULT_NOT_FOUND,
        "same_name_respawn_has_clean_requested_state": respawned is not None and respawned.result.result == Result.RESULT_OK and clean is not None,
        "parent_delete_removes_nested_child": parent_removed.result.result == Result.RESULT_OK and child_before.result.result == Result.RESULT_OK and child_after.result.result != Result.RESULT_OK,
    }
    obs = {"baseline": sorted(base), "after_spawn": sorted(after), "actual_renamed": actual,
           "codes": {"first": first.result.result, "duplicate": duplicate.result.result,
                     "renamed": renamed.result.result, "delete": getattr(getattr(removed, "result", None), "result", None),
                     "repeat_delete": getattr(getattr(repeated, "result", None), "result", None)},
           "returned_name": renamed.entity_name, "clean_state": clean,
           "cleanup_actual_code": getattr(getattr(cleanup_actual, "result", None), "result", None),
           "nested_child_query_before": child_before.result.result,
           "nested_child_query_after_parent_delete": child_after.result.result}
    return result(21, "Spawn/Delete semantics", checks, obs, ("GZ-N1",))


def item22(node, clients):
    from simulation_interfaces.msg import Result, SimulationState
    from simulation_interfaces.srv import ResetSimulation
    set_state(node, clients["set_sim"], clients["get_sim"], SimulationState.STATE_PAUSED)
    normal = set_entity(node, clients["set"], "sphere", 2.0, 1.0, 3.0, 0.25)
    one = wait_position(node, clients["get"], "sphere", [2.0, 1.0, 3.0])
    batch = names(node, clients["many"])
    index = list(batch.entities).index("sphere")
    many = batch.states[index]
    batch_state = {"time": many.header.stamp.sec + many.header.stamp.nanosec / 1e9,
                   "position": [many.pose.position.x, many.pose.position.y, many.pose.position.z],
                   "velocity": [many.twist.linear.x, many.twist.linear.y, many.twist.linear.z]}
    anchor = spawn(node, clients["spawn"], "p3_frame_anchor", 10.0)
    relative = set_entity(node, clients["set"], "sphere", 2.0, 0.0, 2.0, frame="p3_frame_anchor")
    relative_state = wait_position(node, clients["get"], "sphere", [12.0, 0.0, 2.0], 1.0)
    actual_relative = snapshot(get(node, clients["get"], "sphere"))
    missing_frame = set_entity(node, clients["set"], "sphere", 3.0, 0.0, 2.0, frame="no_such_frame")
    before_reset = snapshot(get(node, clients["get"], "sphere"))
    request = ResetSimulation.Request(); request.scope = ResetSimulation.Request.SCOPE_ALL
    reset = call(node, clients["reset"], request)
    after_reset = snapshot(wait_entity(node, clients["get"], "sphere", True))
    spawned_after = wait_entity(node, clients["get"], "p3_frame_anchor", False)
    checks = {
        "normal_set_round_trip": normal.result.result == Result.RESULT_OK and one is not None and abs(one["velocity"][0] - .25) < .02,
        "individual_batch_consistent": max(abs(a - b) for key in ("position", "velocity") for a, b in zip(one[key], batch_state[key])) < .02 and abs(one["time"] - batch_state["time"]) < .01,
        "known_frame_is_applied": anchor.result.result == Result.RESULT_OK and relative.result.result == Result.RESULT_OK and relative_state is not None,
        "missing_frame_is_rejected": missing_frame.result.result != Result.RESULT_OK,
        "reset_returns_new_epoch_state": reset.result.result == Result.RESULT_OK and after_reset["time"] < .05 and abs(after_reset["position"][2] - 2.0) < .05,
        "reset_removes_spawned_entity": spawned_after.result.result != Result.RESULT_OK,
    }
    obs = {"normal": one, "batch": batch_state, "relative_code": relative.result.result,
           "relative_expected_world_x": 12.0, "relative_actual": actual_relative,
           "missing_frame_code": missing_frame.result.result, "before_reset": before_reset,
           "reset_code": reset.result.result, "after_reset": after_reset,
           "spawned_after_reset_code": spawned_after.result.result}
    return result(22, "Get/Set Entity State semantics", checks, obs, ("GZ-F1", "GZ-R1"))


def item23(node, clients):
    from simulation_interfaces.msg import Result, SimulationState
    from simulation_interfaces.srv import ResetSimulation, StepSimulation
    set_state(node, clients["set_sim"], clients["get_sim"], SimulationState.STATE_PAUSED)
    set_entity(node, clients["set"], "sphere", 0.0, 0.0, 2.0)
    paused = []
    for _ in range(3):
        paused.append(snapshot(get(node, clients["get"], "sphere"))); time.sleep(.03)
    set_state(node, clients["set_sim"], clients["get_sim"], SimulationState.STATE_PLAYING)
    time.sleep(.15)
    moving = [snapshot(get(node, clients["get"], "sphere"))]
    time.sleep(.05); moving.append(snapshot(get(node, clients["get"], "sphere")))
    time.sleep(.05)
    set_state(node, clients["set_sim"], clients["get_sim"], SimulationState.STATE_PAUSED)
    settled = []
    for _ in range(3):
        settled.append(snapshot(get(node, clients["get"], "sphere"))); time.sleep(.03)
    step_request = StepSimulation.Request(); step_request.steps = 2000
    step_started = time.monotonic(); step_response = call(node, clients["step"], step_request)
    step_wall = time.monotonic() - step_started
    first_after_step = snapshot(get(node, clients["get"], "sphere"))
    reset_request = ResetSimulation.Request(); reset_request.scope = ResetSimulation.Request.SCOPE_ALL
    reset_response = call(node, clients["reset2"], reset_request)
    reset_samples = []
    for _ in range(4):
        reset_samples.append(snapshot(get(node, clients["get"], "sphere"))); time.sleep(.05)
    paused_spread = max(s["time"] for s in paused) - min(s["time"] for s in paused)
    settled_spread = max(s["time"] for s in settled) - min(s["time"] for s in settled)
    reset_spread = max(s["time"] for s in reset_samples) - min(s["time"] for s in reset_samples)
    checks = {
        "three_paused_baseline_samples_stable": paused_spread < 1e-9,
        "play_has_two_aligned_motion_observations": moving[1]["time"] > moving[0]["time"] and moving[1]["position"][2] < moving[0]["position"][2],
        "pause_after_play_is_stable": settled_spread < 1e-9,
        "step_completes_before_response": first_after_step["time"] >= settled[-1]["time"] + 2.0 - 1e-6,
        "step_reset_competition_leaves_reset_epoch_stable": reset_response.result.result == Result.RESULT_OK and reset_samples[0]["time"] < first_after_step["time"] and reset_spread < 1e-9 and all(abs(x["position"][2] - 2.0) < .05 for x in reset_samples),
    }
    obs = {"paused": paused, "moving": moving, "settled": settled,
           "step_code": step_response.result.result, "step_wall_s": step_wall,
           "first_after_step": first_after_step, "reset_code": reset_response.result.result,
           "reset_samples": reset_samples}
    return result(23, "World control semantics", checks, obs, ("GZ-S1", "GZ-R1"))


def run_once(root, repeat, world):
    import rclpy
    from rclpy.node import Node
    from simulation_interfaces.srv import (DeleteEntity, GetEntitiesStates, GetEntityState,
        GetSimulationState, ResetSimulation, SetEntityState, SetSimulationState, SpawnEntity,
        StepSimulation)
    process, log = server(world, root / "gzserver.log", f"p3-gz-entity-{os.getpid()}-{repeat}")
    node = Node(f"p3_gz_entity_{repeat}_{os.getpid()}")
    types = {"spawn": SpawnEntity, "delete": DeleteEntity, "get": GetEntityState,
             "many": GetEntitiesStates, "set": SetEntityState, "get_sim": GetSimulationState,
             "set_sim": SetSimulationState, "reset": ResetSimulation, "reset2": ResetSimulation,
             "step": StepSimulation}
    services = {"spawn": "spawn_entity", "delete": "delete_entity", "get": "get_entity_state",
                "many": "get_entities_states", "set": "set_entity_state",
                "get_sim": "get_simulation_state", "set_sim": "set_simulation_state",
                "reset": "reset_simulation", "reset2": "reset_simulation", "step": "step_simulation"}
    clients = {name: node.create_client(kind, f"/gzserver/{services[name]}") for name, kind in types.items()}
    try:
        for client in clients.values():
            if not client.wait_for_service(timeout_sec=20):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        return [item21(node, clients), item22(node, clients), item23(node, clients)]
    finally:
        node.destroy_node()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--repetitions", type=int, default=3); args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).resolve(); world = script.with_name("world.sdf")
    manifest = {"schema": "p3-gazebo-entity-world-semantic-v1", "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "repetitions": args.repetitions, "script": str(script), "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(), "world": str(world), "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest()}
    atomic_json(args.output / "manifest.json", manifest)
    import rclpy; rclpy.init(); records = []
    try:
        for repeat in range(args.repetitions):
            root = args.output / f"repeat-{repeat:02d}"; root.mkdir()
            try: current = run_once(root, repeat, world)
            except Exception as exc:
                current = [{"item": item, "title": "Gazebo entity/world semantics", "status": "error", "pass": False, "classification": "execution_error", "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()} for item in (21, 22, 23)]
            for record in current: record["repetition"] = repeat; records.append(record); atomic_json(root / f"item-{record['item']}.json", record)
            atomic_json(args.output / "progress.json", {"completed": len(records), "total": args.repetitions * 3, "pass": sum(x["pass"] for x in records), "errors": sum(x["classification"] == "execution_error" for x in records)})
            print(json.dumps({"repetition": repeat, "results": [[x["item"], x["classification"]] for x in current]}), flush=True)
    finally: rclpy.shutdown()
    counts = {key: sum(x["classification"] == key for x in records) for key in ("pass", "violation", "execution_error")}; counts["total"] = len(records)
    atomic_json(args.output / "summary.json", {"schema": manifest["schema"], "counts": counts, "records": records, "completed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
    print(json.dumps(counts)); return 1 if counts["execution_error"] else 0


if __name__ == "__main__": raise SystemExit(main())
