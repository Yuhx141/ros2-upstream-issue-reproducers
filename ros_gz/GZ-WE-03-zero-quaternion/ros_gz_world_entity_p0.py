#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
import subprocess
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

from gz_semantic_pilot import call, server, set_state, snapshot, wait_state
from map_semantic_regression import atomic_json, terminate

CASES = (
    "valid_dynamic",
    "missing_set",
    "static_twist",
    "invalid_pose",
    "unsupported_bounds",
    "missing_read",
)
STATIC_SDF = """<sdf version='1.9'><model name='static_probe'><static>true</static>
<link name='body'/></model></sdf>"""


def set_request(entity, x=0.0, z=2.0, vx=0.0, valid_orientation=True):
    from simulation_interfaces.srv import SetEntityState
    request = SetEntityState.Request()
    request.entity = entity
    request.state.header.frame_id = "world"
    request.state.pose.position.x = x
    request.state.pose.position.z = z
    if valid_orientation:
        request.state.pose.orientation.w = 1.0
    request.state.twist.linear.x = vx
    return request


def get_entity(node, client, entity):
    from simulation_interfaces.srv import GetEntityState
    request = GetEntityState.Request()
    request.entity = entity
    return call(node, client, request, timeout=5.0)


def health(node, client):
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import GetSimulationState
    try:
        response = call(node, client, GetSimulationState.Request(), timeout=3.0)
        return {"ok": response.result.result == Result.RESULT_OK,
                "code": response.result.result, "state": response.state.state}
    except Exception as error:
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}


def wait_position(node, client, entity, x, timeout=3.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        response = get_entity(node, client, entity)
        if response.result.result == 1:
            last = snapshot(response)
            if abs(last["position"][0] - x) < 0.02:
                return last
        time.sleep(0.02)
    return last


def run_case(case, node, clients, process):
    from geometry_msgs.msg import Vector3
    from simulation_interfaces.msg import Bounds, Result, SimulationState, SimulatorFeatures
    from simulation_interfaces.srv import (
        GetEntities, GetEntitiesStates, GetEntityInfo, GetSimulatorFeatures,
        SetEntityState, SpawnEntity,
    )

    set_state(node, clients["set_sim"], clients["get_sim"], SimulationState.STATE_PAUSED)

    if case == "valid_dynamic":
        response = call(node, clients["set"], set_request("sphere", x=1.0, vx=0.25))
        state = wait_position(node, clients["get"], "sphere", 1.0)
        checks = {
            "result_ok": response.result.result == Result.RESULT_OK,
            "pose_round_trip": state is not None and abs(state["position"][0] - 1.0) < 0.02,
            "twist_round_trip": state is not None and abs(state["velocity"][0] - 0.25) < 0.02,
            "server_healthy": health(node, clients["get_sim"])["ok"],
        }
        observed = {"code": response.result.result, "state": state}

    elif case == "missing_set":
        service_error = ""
        response = None
        try:
            response = call(node, clients["set"], set_request("no_such_entity"), timeout=5.0)
        except Exception as error:
            service_error = f"{type(error).__name__}: {error}"
        status = health(node, clients["get_sim"])
        checks = {
            "reports_not_found": response is not None and response.result.result == Result.RESULT_NOT_FOUND,
            "server_healthy": status["ok"],
        }
        observed = {"code": None if response is None else response.result.result,
                    "service_error": service_error, "health": status,
                    "server_returncode": process.poll()}

    elif case == "static_twist":
        spawn = SpawnEntity.Request()
        spawn.name = "p3_static"
        spawn.resource_string = STATIC_SDF
        spawn.initial_pose.header.frame_id = "world"
        spawn.initial_pose.pose.position.x = 5.0
        spawn.initial_pose.pose.orientation.w = 1.0
        spawned = call(node, clients["spawn"], spawn)
        wait_position(node, clients["get"], "p3_static", 5.0)
        response = call(node, clients["set"], set_request("p3_static", x=7.0, z=0.0, vx=1.0))
        time.sleep(0.05)
        after = get_entity(node, clients["get"], "p3_static")
        checks = {
            "fixture_spawned": spawned.result.result == Result.RESULT_OK,
            "static_nonzero_twist_rejected": response.result.result == Result.RESULT_OPERATION_FAILED,
            "server_healthy": health(node, clients["get_sim"])["ok"],
        }
        observed = {"spawn_code": spawned.result.result, "set_code": response.result.result,
                    "after_code": after.result.result,
                    "after_position_x": after.state.pose.position.x,
                    "after_velocity_x": after.state.twist.linear.x}

    elif case == "invalid_pose":
        response = call(node, clients["set"], set_request("sphere", x=3.0, valid_orientation=False))
        status = health(node, clients["get_sim"])
        checks = {
            "invalid_quaternion_reports_101": response.result.result == SetEntityState.Response.INVALID_POSE,
            "server_healthy": status["ok"],
        }
        observed = {"code": response.result.result, "message": response.result.error_message,
                    "health": status}

    elif case == "unsupported_bounds":
        features = call(node, clients["features"], GetSimulatorFeatures.Request())
        advertised = SimulatorFeatures.ENTITY_BOUNDS in features.features.features
        request = GetEntities.Request()
        request.filters.bounds.type = Bounds.TYPE_SPHERE
        center = Vector3()
        radius = Vector3()
        radius.x = 1.0
        request.filters.bounds.points = [center, radius]
        entities = call(node, clients["entities"], request)
        states_request = GetEntitiesStates.Request()
        states_request.filters = request.filters
        states = call(node, clients["many"], states_request)
        checks = {
            "bounds_not_advertised": not advertised,
            "get_entities_reports_unsupported": entities.result.result == Result.RESULT_FEATURE_UNSUPPORTED,
            "get_entities_states_reports_unsupported": states.result.result == Result.RESULT_FEATURE_UNSUPPORTED,
            "server_healthy": health(node, clients["get_sim"])["ok"],
        }
        observed = {"advertised": advertised, "get_entities_code": entities.result.result,
                    "get_entities_names": list(entities.entities),
                    "get_states_code": states.result.result,
                    "get_states_names": list(states.entities)}

    else:
        state = get_entity(node, clients["get"], "no_such_entity")
        info_request = GetEntityInfo.Request()
        info_request.entity = "no_such_entity"
        info = call(node, clients["info"], info_request)
        checks = {
            "get_state_reports_not_found": state.result.result == Result.RESULT_NOT_FOUND,
            "get_info_reports_not_found": info.result.result == Result.RESULT_NOT_FOUND,
            "server_healthy": health(node, clients["get_sim"])["ok"],
        }
        observed = {"get_state_code": state.result.result,
                    "get_state_message": state.result.error_message,
                    "get_info_code": info.result.result,
                    "get_info_message": info.result.error_message}

    return {
        "case": case,
        "status": "pass" if all(checks.values()) else "violation",
        "checks": checks,
        "observed": observed,
    }


def run_one(root, case, repeat, world):
    import rclpy
    from rclpy.node import Node
    from simulation_interfaces.msg import SimulationState
    from simulation_interfaces.srv import (
        GetEntities, GetEntitiesStates, GetEntityInfo, GetEntityState,
        GetSimulationState, GetSimulatorFeatures, SetEntityState,
        SetSimulationState, SpawnEntity,
    )

    case_dir = root / f"{case}-{repeat + 1}"
    case_dir.mkdir()
    process, log = server(world, case_dir / "gzserver.log",
                          f"p3-gz-world-{os.getpid()}-{case}-{repeat}")
    node = Node(f"p3_gz_world_{case}_{repeat}_{os.getpid()}")
    types = {
        "set": SetEntityState,
        "get": GetEntityState,
        "spawn": SpawnEntity,
        "get_sim": GetSimulationState,
        "set_sim": SetSimulationState,
        "features": GetSimulatorFeatures,
        "entities": GetEntities,
        "many": GetEntitiesStates,
        "info": GetEntityInfo,
    }
    clients = {name: node.create_client(kind, f"/gzserver/{name_map(name)}")
               for name, kind in types.items()}
    setup_complete = False
    try:
        for client in clients.values():
            if not client.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        wait_state(node, clients["get_sim"], SimulationState.STATE_PLAYING)
        setup_complete = True
        record = run_case(case, node, clients, process)
    except Exception as error:
        record = {
            "case": case,
            "status": "violation" if setup_complete else "execution_error",
            "checks": {"completed": False},
            "observed": {
                "error": f"{type(error).__name__}: {error}",
                "traceback": traceback.format_exc(),
                "server_returncode": process.poll(),
            },
        }
    finally:
        terminate(process)
        log.close()
        node.destroy_node()
    record["repeat"] = repeat + 1
    atomic_json(case_dir / "result.json", record)
    return record


def name_map(name):
    return {
        "set": "set_entity_state",
        "get": "get_entity_state",
        "spawn": "spawn_entity",
        "get_sim": "get_simulation_state",
        "set_sim": "set_simulation_state",
        "features": "get_simulator_features",
        "entities": "get_entities",
        "many": "get_entities_states",
        "info": "get_entity_info",
    }[name]


def source_hash(root, path):
    content = subprocess.run(
        ["git", "-C", str(root), "show", f"HEAD:{path}"],
        check=True, capture_output=True).stdout
    return hashlib.sha256(content).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repetitions", default=3, type=int)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    import rclpy
    rclpy.init()
    script = Path(__file__).resolve()
    worktree = script.parents[2]
    world = worktree / "experiments/p1-gz-state/world.sdf"
    source_root = worktree.parents[1] / "ros_gz-src-1.0.24"
    paths = [
        "ros_gz_sim/src/gz_simulation_interfaces/services/set_entity_state.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/services/get_entity_state.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/services/get_entity_info.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/gz_entity_filters.cpp",
        "ros_gz_sim/src/gz_simulation_interfaces/services/get_simulator_features.cpp",
    ]
    manifest = {
        "schema": "p3-ros-gz-world-entity-contract-v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "package": "ros_gz_sim 1.0.24",
        "source_head": subprocess.run(
            ["git", "-C", str(source_root), "rev-parse", "HEAD"],
            check=True, text=True, capture_output=True).stdout.strip(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
        "source_sha256": {path: source_hash(source_root, path) for path in paths},
        "repetitions": args.repetitions,
        "cases": list(CASES),
    }
    atomic_json(args.output / "manifest.json", manifest)
    records = []
    try:
        for case in CASES:
            for repeat in range(args.repetitions):
                record = run_one(args.output, case, repeat, world)
                records.append(record)
                atomic_json(args.output / "progress.json", {
                    "completed": len(records),
                    "total": len(CASES) * args.repetitions,
                    "counts": {key: sum(item["status"] == key for item in records)
                               for key in ("pass", "violation", "execution_error")},
                })
                print(json.dumps({"case": case, "repeat": repeat + 1,
                                  "status": record["status"]}), flush=True)
    finally:
        rclpy.shutdown()
    counts = {key: sum(item["status"] == key for item in records)
              for key in ("pass", "violation", "execution_error")}
    summary = {"schema": manifest["schema"], "counts": counts, "records": records}
    atomic_json(args.output / "summary.json", summary)
    print(json.dumps(counts, sort_keys=True))
    return 1 if counts["execution_error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
