#!/usr/bin/env python3
"""Test SetEntityState on an absent entity after a successful valid update."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import GetEntities, GetEntityState, SetEntityState, SpawnEntity

SDF = ("<sdf version='1.9'><model name='payload'><static>true</static>"
       "<link name='body'><collision name='box'><geometry><box>"
       "<size>0.1 0.1 0.1</size></box></geometry></collision></link>"
       "</model></sdf>")
VALID = "p3_set_valid"
MISSING = "p3_set_absent"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--expected-library", type=Path)
    parser.add_argument("--scenario", choices=("missing", "frame", "static-twist"), default="missing")
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be 0..232")
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(out / "ros-logs")
    os.environ["GZ_PARTITION"] = f"p3-set-missing-{os.getpid()}-{args.ros_domain_id}"
    world = Path(__file__).resolve().parent / "world.sdf"
    trace = (out / "trace.jsonl").open("x", encoding="utf-8")
    log = (out / "gzserver.log").open("x", encoding="utf-8")
    started = time.monotonic_ns()
    process = node = None
    values, errors, preconditions = {}, [], []
    loaded_libraries = []

    def emit(event, **fields):
        trace.write(json.dumps({"event": event, "elapsed_ns": time.monotonic_ns() - started,
                                **fields}) + "\n")
        trace.flush()

    try:
        process = subprocess.Popen(
            ["ros2", "run", "ros_gz_sim", "gzserver", "--ros-args", "-p",
             f"world_sdf_file:={world}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        emit("server_start", pid=process.pid, domain=args.ros_domain_id, partition=os.environ["GZ_PARTITION"])
        rclpy.init()
        node = Node(f"p3_set_missing_{os.getpid()}")
        spawn = node.create_client(SpawnEntity, "/gzserver/spawn_entity")
        get_entities = node.create_client(GetEntities, "/gzserver/get_entities")
        get_state = node.create_client(GetEntityState, "/gzserver/get_entity_state")
        set_state = node.create_client(SetEntityState, "/gzserver/set_entity_state")
        for client in (spawn, get_entities, get_state, set_state):
            if not client.wait_for_service(timeout_sec=20):
                raise TimeoutError(f"service not discovered: {client.srv_name}")
        if args.expected_library is not None:
            expected_library = args.expected_library.resolve(strict=True)
            for proc in Path("/proc").iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    if os.getpgid(int(proc.name)) != process.pid:
                        continue
                    for line in (proc / "maps").read_text().splitlines():
                        if "libgzserver_component.so" in line:
                            loaded_libraries.append(line.split(maxsplit=5)[-1])
                except (FileNotFoundError, ProcessLookupError, PermissionError):
                    continue
            loaded_libraries = sorted(set(loaded_libraries))
            emit("loaded_libraries", paths=loaded_libraries)
            if loaded_libraries != [str(expected_library)]:
                raise RuntimeError(f"loaded library mismatch: {loaded_libraries}")

        def call(label, client, request):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or future.exception() is not None:
                raise TimeoutError(f"{label} did not return")
            response = future.result()
            emit("service_response", operation=label, result=response.result.result)
            return response

        def names():
            response = call("get_entities", get_entities, GetEntities.Request())
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError("GetEntities failed")
            return set(response.entities)

        def position(entity):
            request = GetEntityState.Request()
            request.entity = entity
            response = call("get_entity_state", get_state, request)
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError(f"GetEntityState({entity}) failed")
            return response.state.pose.position.x

        def set_request(entity, x):
            request = SetEntityState.Request()
            request.entity = entity
            request.state.pose.position.x = x
            request.state.pose.orientation.w = 1.0
            return request

        before = names()
        if VALID in before or MISSING in before:
            raise RuntimeError("test name already occupied")
        request = SpawnEntity.Request()
        request.name = VALID
        request.resource_string = SDF
        request.initial_pose.header.frame_id = "world"
        request.initial_pose.pose.position.x = 1.0
        request.initial_pose.pose.orientation.w = 1.0
        spawned = call("spawn", spawn, request)
        deadline = time.monotonic() + 5
        while VALID not in names() and time.monotonic() < deadline:
            time.sleep(0.05)
        initial_x = position(VALID)
        valid = call("set_valid", set_state, set_request(VALID, 2.0))
        deadline = time.monotonic() + 5
        valid_x = position(VALID)
        while abs(valid_x - 2.0) > 0.01 and time.monotonic() < deadline:
            time.sleep(0.05)
            valid_x = position(VALID)
        if (spawned.result.result != Result.RESULT_OK or abs(initial_x - 1.0) > 0.01
                or valid.result.result != Result.RESULT_OK or abs(valid_x - 2.0) > 0.01):
            preconditions.append("valid spawn/set/readback did not reach x=2")
        elif args.scenario == "static-twist":
            twist_request = set_request(VALID, 3.0)
            twist_request.state.header.frame_id = "world"
            twist_request.state.twist.linear.x = 1.0
            twist_reply = call("set_static_nonzero_twist", set_state, twist_request)
            deadline = time.monotonic() + 5
            twist_x = position(VALID)
            while (twist_reply.result.result == Result.RESULT_OK and
                   abs(twist_x - 2.0) < 0.01 and time.monotonic() < deadline):
                time.sleep(0.05)
                twist_x = position(VALID)
            values = {"spawn_result": spawned.result.result, "initial_x": initial_x,
                      "valid_set_result": valid.result.result, "valid_x": valid_x,
                      "twist_result": twist_reply.result.result,
                      "twist_message": twist_reply.result.error_message,
                      "requested_twist_x": 1.0, "requested_pose_x": 3.0,
                      "twist_x": twist_x, "server_exit_after": process.poll()}
            emit("static_twist_witness", **values)
        elif args.scenario == "frame":
            anchor = "p3_set_anchor"
            if anchor in names():
                raise RuntimeError("anchor name already occupied")
            anchor_request = SpawnEntity.Request()
            anchor_request.name = anchor
            anchor_request.resource_string = SDF
            anchor_request.initial_pose.header.frame_id = "world"
            anchor_request.initial_pose.pose.position.x = 10.0
            anchor_request.initial_pose.pose.orientation.w = 1.0
            anchor_spawn = call("spawn_anchor", spawn, anchor_request)
            deadline = time.monotonic() + 5
            while anchor not in names() and time.monotonic() < deadline:
                time.sleep(0.05)
            anchor_x = position(anchor)
            if anchor_spawn.result.result != Result.RESULT_OK or abs(anchor_x - 10.0) > 0.01:
                preconditions.append("anchor spawn/readback did not reach x=10")
            else:
                known_request = set_request(VALID, 3.0)
                known_request.state.header.frame_id = anchor
                known = call("set_known_frame", set_state, known_request)
                deadline = time.monotonic() + 5
                known_x = position(VALID)
                while (known.result.result == Result.RESULT_OK and
                       abs(known_x - 2.0) < 0.01 and time.monotonic() < deadline):
                    time.sleep(0.05)
                    known_x = position(VALID)
                unknown_request = set_request(VALID, 4.0)
                unknown_request.state.header.frame_id = "p3_no_such_frame"
                unknown = call("set_unknown_frame", set_state, unknown_request)
                deadline = time.monotonic() + 5
                unknown_x = position(VALID)
                while (unknown.result.result == Result.RESULT_OK and
                       abs(unknown_x - known_x) < 0.01 and time.monotonic() < deadline):
                    time.sleep(0.05)
                    unknown_x = position(VALID)
                values = {"spawn_result": spawned.result.result, "initial_x": initial_x,
                          "valid_set_result": valid.result.result, "valid_x": valid_x,
                          "anchor_result": anchor_spawn.result.result, "anchor_x": anchor_x,
                          "known_frame": anchor, "known_result": known.result.result,
                          "known_message": known.result.error_message, "known_x": known_x,
                          "unknown_result": unknown.result.result,
                          "unknown_message": unknown.result.error_message,
                          "unknown_x": unknown_x, "server_exit_after": process.poll()}
                emit("frame_witness", **values)
        else:
            names_before_missing = names()
            future = set_state.call_async(set_request(MISSING, 9.0))
            deadline = time.monotonic() + 8
            while not future.done() and process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            exited = process.poll()
            missing_result = None
            missing_error_message = None
            missing_exception = None
            if future.done():
                if future.exception() is None:
                    missing_reply = future.result().result
                    missing_result = missing_reply.result
                    missing_error_message = missing_reply.error_message
                else:
                    missing_exception = str(future.exception())
            emit("missing_outcome", result=missing_result, exception=missing_exception,
                 server_exit=exited, response_done=future.done())
            names_after_missing = names() if exited is None and future.done() else None
            valid_x_after = position(VALID) if names_after_missing is not None else None
            missing_get_result = None
            missing_get_message = None
            if names_after_missing is not None:
                get_request = GetEntityState.Request()
                get_request.entity = MISSING
                missing_get_reply = call("get_missing_state", get_state, get_request).result
                missing_get_result = missing_get_reply.result
                missing_get_message = missing_get_reply.error_message
                if (missing_get_result != Result.RESULT_OPERATION_FAILED or
                        missing_get_message != "Requested entity not found"):
                    preconditions.append("GetEntityState did not confirm target absence")
            values = {"spawn_result": spawned.result.result, "initial_x": initial_x,
                      "valid_set_result": valid.result.result, "valid_x": valid_x,
                      "names_before_missing": sorted(names_before_missing),
                      "missing_result": missing_result, "missing_error_message": missing_error_message,
                      "missing_get_result": missing_get_result,
                      "missing_get_message": missing_get_message,
                      "missing_exception": missing_exception,
                      "missing_response_done": future.done(), "server_exit_after_missing": exited,
                      "names_after_missing": sorted(names_after_missing) if names_after_missing is not None else None,
                      "valid_x_after": valid_x_after}
            emit("state_witness", **values)
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
        emit("execution_error", error=errors[-1])
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        emit("cleanup_complete", returncode=process.returncode if process else None)
        trace.close()
        log.close()
    if errors:
        status = "execution_error"
    elif preconditions:
        status = "invalid_precondition"
    elif args.scenario == "static-twist":
        status = ("violation" if (
            values["server_exit_after"] is not None or
            values["twist_result"] != Result.RESULT_OPERATION_FAILED or
            abs(values["twist_x"] - 2.0) > 0.01
        ) else "pass")
    elif args.scenario == "frame":
        status = ("violation" if (
            values["server_exit_after"] is not None or
            (values["known_result"] == Result.RESULT_OK and
             abs(values["known_x"] - 13.0) > 0.01) or
            (values["known_result"] != Result.RESULT_OK and
             abs(values["known_x"] - 2.0) > 0.01) or
            values["unknown_result"] == Result.RESULT_OK or
            abs(values["unknown_x"] - values["known_x"]) > 0.01
        ) else "pass")
    elif (values["server_exit_after_missing"] is not None or
          values["missing_result"] == Result.RESULT_OK or
          (values["valid_x_after"] is not None and abs(values["valid_x_after"] - 2.0) > 0.01) or
          (values["names_after_missing"] is not None and
           values["names_after_missing"] != values["names_before_missing"])):
        status = "violation"
    elif values["missing_result"] is None:
        status = "uncertain"
    else:
        status = "pass"
    summary = {"schema": 1, "status": status, "domain": args.ros_domain_id,
               "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
               "server_returncode": process.returncode if process else None,
               "loaded_libraries": loaded_libraries,
               "loaded_library_sha256": (hashlib.sha256(args.expected_library.read_bytes()).hexdigest()
                                         if args.expected_library is not None else None),
               "values": values, "preconditions": preconditions, "errors": errors}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"status": status, "values": values,
                      "preconditions": preconditions, "errors": errors}))
    raise SystemExit({"pass": 0, "violation": 0, "uncertain": 3,
                      "invalid_precondition": 3, "execution_error": 2}[status])


if __name__ == "__main__":
    main()
