#!/usr/bin/env python3
"""MoveIt planning-scene health gate; no bug verdicts."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetStateValidity
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

URDF = '''<robot name="p3_slider">
<link name="base"/>
<link name="slider"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/>
<origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
<limit lower="0" upper="1" effort="1" velocity="1"/></joint>
</robot>'''
SRDF = '''<robot name="p3_slider"><virtual_joint name="world_joint" type="fixed"
parent_frame="world" child_link="base"/><group name="slider_group"><joint name="slide"/></group></robot>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--scenario", choices=("health", "invalid-frame", "replace", "attach-transfer"), default="health")
    parser.add_argument("--explicit-touch-link", action="store_true")
    parser.add_argument("--planning-scene-lib-dir", type=Path)
    parser.add_argument("--expect-default-fixed", action="store_true")
    parser.add_argument("--plan-after-attach", action="store_true")
    args = parser.parse_args()
    if (args.explicit_touch_link or args.expect_default_fixed or args.plan_after_attach) and args.scenario != "attach-transfer":
        parser.error("touch-link controls only apply to attach-transfer")
    if args.expect_default_fixed and args.planning_scene_lib_dir is None:
        parser.error("expect-default-fixed requires an isolated library")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be 0..232")
    override = None
    if args.planning_scene_lib_dir is not None:
        override = (args.planning_scene_lib_dir.resolve() /
                    "libmoveit_planning_scene.so.2.12.4")
        if not override.is_file():
            parser.error(f"planning scene override missing: {override}")
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(out / "ros-logs")
    params = out / "move_group.yaml"
    params.write_text(yaml.safe_dump({"move_group": {"ros__parameters": {
        "robot_description": URDF,
        "robot_description_semantic": SRDF,
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {"planning_plugins": ["ompl_interface/OMPLPlanner"]},
        "allow_trajectory_execution": False,
    }}}, sort_keys=False))
    trace = (out / "trace.jsonl").open("x")
    log = (out / "move_group.log").open("x")
    started = time.monotonic_ns()
    calls, errors = [], []
    boundary, status, planning = None, "execution_error", None
    process = node = None
    loaded_planning_scene = []

    def emit(event, **fields):
        trace.write(json.dumps({"event": event, "elapsed_ns": time.monotonic_ns() - started,
                                **fields}) + "\n")
        trace.flush()

    try:
        rclpy.init()
        node = Node(f"p3_moveit_admission_{os.getpid()}")
        server_env = os.environ.copy()
        if override is not None:
            server_env["LD_LIBRARY_PATH"] = (
                f"{override.parent}:{server_env.get('LD_LIBRARY_PATH', '')}")
        process = subprocess.Popen([
            "/opt/ros/jazzy/lib/moveit_ros_move_group/move_group", "--ros-args",
            "--params-file", str(params)], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, env=server_env)
        emit("server_start", pid=process.pid, domain=args.ros_domain_id)
        clients = {
            "apply": node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
            "scene": node.create_client(GetPlanningScene, "/get_planning_scene"),
            "valid": node.create_client(GetStateValidity, "/check_state_validity"),
            **({"plan": node.create_client(GetMotionPlan, "/plan_kinematic_path")}
               if args.plan_after_attach else {}),
        }
        deadline = time.monotonic() + 25
        for name, client in clients.items():
            while not client.wait_for_service(timeout_sec=0.1):
                if process.poll() is not None:
                    raise RuntimeError(f"move_group exited during {name} discovery: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{name} service discovery timeout")
        emit("services_ready", names=[client.srv_name for client in clients.values()])
        loaded_planning_scene = sorted({line.split()[-1] for line in
                                        Path(f"/proc/{process.pid}/maps").read_text().splitlines()
                                        if "libmoveit_planning_scene.so.2.12.4" in line})
        emit("planning_scene_library", paths=loaded_planning_scene)
        if override is not None and loaded_planning_scene != [str(override)]:
            raise RuntimeError(f"wrong planning scene library: {loaded_planning_scene}")

        def call(name, request):
            future = clients[name].call_async(request)
            sent_ns = time.monotonic_ns()
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done() or future.exception() is not None:
                raise TimeoutError(f"{name} response timeout or error")
            result = future.result()
            calls.append({"name": name, "sent_ns": sent_ns,
                          "received_ns": time.monotonic_ns()})
            emit("service_response", **calls[-1])
            return result

        def valid(q):
            request = GetStateValidity.Request()
            request.group_name = "slider_group"
            request.robot_state.joint_state.name = ["slide"]
            request.robot_state.joint_state.position = [q]
            request.robot_state.is_diff = True
            response = call("valid", request)
            emit("state_validity", position=q, valid=response.valid,
                 contacts=[(c.contact_body_1, c.contact_body_2,
                            c.body_type_1, c.body_type_2) for c in response.contacts])
            return response.valid

        def scene_ids():
            response = call("scene", GetPlanningScene.Request())
            ids = sorted(obj.id for obj in response.scene.world.collision_objects)
            emit("scene_objects", ids=ids)
            return ids

        before = [valid(0.0), valid(1.0)]
        if before != [True, True] or scene_ids():
            raise RuntimeError(f"empty-scene health precondition failed: {before}")
        obj = CollisionObject()
        obj.id, obj.header.frame_id, obj.operation = "p3_obstacle", "base", CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type, shape.dimensions = SolidPrimitive.BOX, [0.4, 0.4, 0.4]
        obj.primitives = [shape]
        obj.primitive_poses = [Pose()]
        obj.primitive_poses[0].orientation.w = 1.0
        obj.pose.orientation.w = 1.0
        add = ApplyPlanningScene.Request()
        add.scene = PlanningScene()
        add.scene.is_diff = True
        add.scene.world.collision_objects = [obj]
        accepted_add = call("apply", add).success
        after_add = {"accepted": accepted_add, "ids": scene_ids(),
                     "near": valid(0.0), "far": valid(1.0)}
        emit("after_add", **after_add)
        remove = ApplyPlanningScene.Request()
        remove.scene.is_diff = True
        removed = CollisionObject()
        removed.id, removed.header.frame_id, removed.operation = (
            "p3_obstacle", "base", CollisionObject.REMOVE)
        remove.scene.world.collision_objects = [removed]
        accepted_remove = call("apply", remove).success
        after_remove = {"accepted": accepted_remove, "ids": scene_ids(),
                        "near": valid(0.0), "far": valid(1.0)}
        emit("after_remove", **after_remove)
        healthy = (accepted_add and after_add["ids"] == ["p3_obstacle"] and
                   not after_add["near"] and after_add["far"] and
                   accepted_remove and not after_remove["ids"] and
                   after_remove["near"] and after_remove["far"])
        status = "pass" if healthy else "invalid_precondition"
        if healthy and args.scenario == "invalid-frame":
            bad = CollisionObject()
            bad.id, bad.header.frame_id, bad.operation = (
                "p3_bad_frame", "p3_missing_frame", CollisionObject.ADD)
            bad.primitives = [shape]
            bad.primitive_poses = [Pose()]
            bad.primitive_poses[0].orientation.w = 1.0
            bad.pose.orientation.w = 1.0
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.world.collision_objects = [bad]
            boundary = {"accepted": call("apply", request).success,
                        "ids": scene_ids(), "near": valid(0.0), "far": valid(1.0)}
            emit("invalid_frame_witness", **boundary)
            status = ("pass" if boundary == {"accepted": False, "ids": [],
                                            "near": True, "far": True} else "candidate")
        elif healthy and args.scenario == "replace":
            first = copy.deepcopy(obj)
            first.id = "p3_replace"
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.world.collision_objects = [first]
            accepted_first = call("apply", request).success
            first_state = {"accepted": accepted_first, "ids": scene_ids(),
                           "near": valid(0.0), "far": valid(1.0)}
            emit("replace_first", **first_state)
            second = copy.deepcopy(first)
            second.pose.position.x = 1.0
            request.scene.world.collision_objects = [second]
            accepted_second = call("apply", request).success
            scene = call("scene", GetPlanningScene.Request()).scene
            objects = [(item.id, item.pose.position.x) for item in scene.world.collision_objects]
            second_state = {"accepted": accepted_second, "objects": objects,
                            "near": valid(0.0), "far": valid(1.0)}
            emit("replace_second", **second_state)
            replace_remove = copy.deepcopy(removed)
            replace_remove.id = "p3_replace"
            request.scene.world.collision_objects = [replace_remove]
            accepted_remove = call("apply", request).success
            final = {"accepted": accepted_remove, "ids": scene_ids(),
                     "near": valid(0.0), "far": valid(1.0)}
            emit("replace_removed", **final)
            boundary = {"first": first_state, "second": second_state, "removed": final}
            status = "pass" if (
                first_state == {"accepted": True, "ids": ["p3_replace"],
                                "near": False, "far": True} and
                second_state == {"accepted": True, "objects": [("p3_replace", 1.0)],
                                 "near": True, "far": False} and
                final == {"accepted": True, "ids": [], "near": True, "far": True}
            ) else "candidate"
        elif healthy and args.scenario == "attach-transfer":
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            transfer_obj = copy.deepcopy(obj)
            transfer_obj.id = "p3_transfer"
            request.scene.world.collision_objects = [transfer_obj]
            added = call("apply", request).success
            before = {"accepted": added, "ids": scene_ids(),
                      "near": valid(0.0), "far": valid(1.0)}
            emit("transfer_world", **before)

            attached = AttachedCollisionObject()
            attached.link_name = "slider"
            attached.touch_links = ["slider"] if args.explicit_touch_link else []
            attached.object.id = "p3_transfer"
            attached.object.operation = CollisionObject.ADD
            request.scene.world.collision_objects = []
            request.scene.robot_state.is_diff = True
            request.scene.robot_state.attached_collision_objects = [attached]
            accepted_attach = call("apply", request).success

            def transfer_scene():
                query = GetPlanningScene.Request()
                query.components.components = (
                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
                scene = call("scene", query).scene
                return {"world": sorted(o.id for o in scene.world.collision_objects),
                        "attached": sorted(o.object.id for o in
                                           scene.robot_state.attached_collision_objects),
                        "touch_links": sorted({link for o in
                                               scene.robot_state.attached_collision_objects
                                               for link in o.touch_links})}

            after_attach = {"accepted": accepted_attach, **transfer_scene(),
                            "near": valid(0.0), "far": valid(1.0)}
            emit("transfer_attached", **after_attach)
            if args.plan_after_attach:
                plan_request = GetMotionPlan.Request()
                plan = plan_request.motion_plan_request
                plan.group_name = "slider_group"
                plan.start_state.joint_state.name = ["slide"]
                plan.start_state.joint_state.position = [0.0]
                plan.start_state.is_diff = True
                goal = Constraints()
                goal.joint_constraints = [JointConstraint(joint_name="slide", position=1.0,
                                                          tolerance_above=0.001,
                                                          tolerance_below=0.001, weight=1.0)]
                plan.goal_constraints = [goal]
                plan.num_planning_attempts = 1
                plan.allowed_planning_time = 3.0
                plan.max_velocity_scaling_factor = 1.0
                plan.max_acceleration_scaling_factor = 1.0
                response = call("plan", plan_request).motion_plan_response
                planning = {"code": response.error_code.val,
                            "message": response.error_code.message,
                            "points": len(response.trajectory.joint_trajectory.points),
                            "joint_names": list(response.trajectory.joint_trajectory.joint_names)}
                emit("motion_plan", **planning)
            attached.object.operation = CollisionObject.REMOVE
            request.scene.robot_state.attached_collision_objects = [attached]
            accepted_detach = call("apply", request).success
            after_detach = {"accepted": accepted_detach, **transfer_scene(),
                            "near": valid(0.0), "far": valid(1.0)}
            emit("transfer_detached", **after_detach)
            boundary = {"world": before, "attached": after_attach,
                        "detached": after_detach}
            status = "pass" if (
                before == {"accepted": True, "ids": ["p3_transfer"],
                           "near": False, "far": True} and
                after_attach == {"accepted": True, "world": [],
                                 "attached": ["p3_transfer"],
                                 "touch_links": ["slider"] if (
                                     args.explicit_touch_link or args.expect_default_fixed) else [],
                                 "near": True, "far": True} and
                after_detach == {"accepted": True, "world": ["p3_transfer"],
                                 "attached": [], "touch_links": [],
                                 "near": False, "far": True}
            ) else "candidate"
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
        emit("execution_error", error=errors[-1])
        healthy = False
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        server_alive_before_cleanup = process is not None and process.poll() is None
        if server_alive_before_cleanup:
            # End the isolated fixture; graceful shutdown is outside this relation.
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        emit("cleanup", returncode=process.returncode if process else None)
        log.close()
        trace.close()
    summary = {"stage": "admission", "scenario": args.scenario, "status": status,
               "healthy": healthy, "boundary": boundary, "errors": errors,
               "explicit_touch_link": args.explicit_touch_link,
               "expect_default_fixed": args.expect_default_fixed,
               "planning_scene_lib": str(override) if override else None,
               "planning": planning,
               "loaded_planning_scene": loaded_planning_scene,
               "calls": calls, "domain": args.ros_domain_id,
               "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "server_alive_before_cleanup": server_alive_before_cleanup,
               "server_returncode": process.returncode if process else None}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in ("scenario", "status", "healthy", "boundary", "errors", "server_returncode")}))
    raise SystemExit(0 if status in ("pass", "candidate") else 2)


if __name__ == "__main__":
    assert '<joint name="slide"' in URDF and 'slider_group' in SRDF
    main()
