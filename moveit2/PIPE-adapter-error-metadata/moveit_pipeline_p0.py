#!/usr/bin/env python3
"""Deterministic MoveIt planning-pipeline error and diagnostic checks."""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
import traceback
from pathlib import Path

import yaml


URDF = """<robot name='p3_slider'>
<link name='base'/><link name='slider'><collision><geometry><sphere radius='0.08'/></geometry></collision></link>
<joint name='slide' type='prismatic'><parent link='base'/><child link='slider'/><axis xyz='1 0 0'/><limit lower='0' upper='1' effort='1' velocity='1'/></joint>
</robot>"""
SRDF = """<robot name='p3_slider'><virtual_joint name='world_joint' type='fixed' parent_frame='world' child_link='base'/><group name='slider_group'><joint name='slide'/></group></robot>"""


def atomic_json(path, value):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(tmp, path)


def terminate(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def run_once(root, repeat):
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import (
        CollisionObject, Constraints, JointConstraint, PipelineState,
        PlanningScene, RobotState,
    )
    from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
    from rclpy.node import Node
    from shape_msgs.msg import SolidPrimitive

    params = root / "move_group.yaml"
    params.write_text(yaml.safe_dump({"move_group": {"ros__parameters": {
        "robot_description": URDF,
        "robot_description_semantic": SRDF,
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
        },
        "allow_trajectory_execution": False,
    }}}, sort_keys=False))
    log = (root / "move_group.log").open("w")
    process = subprocess.Popen(
        ["ros2", "run", "moveit_ros_move_group", "move_group", "--ros-args", "--params-file", str(params)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    node = Node(f"p3_moveit_pipeline_{repeat}_{os.getpid()}")
    clients = {
        "apply": node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
        "valid": node.create_client(GetStateValidity, "/check_state_validity"),
        "plan": node.create_client(GetMotionPlan, "/plan_kinematic_path"),
    }
    stages = []

    def on_stage(msg):
        stages.append({
            "stage": msg.pipeline_stage,
            "code": msg.response.error_code.val,
            "message": msg.response.error_code.message,
            "source": msg.response.error_code.source,
        })

    stage_sub = node.create_subscription(PipelineState, "/pipeline_state", on_stage, 20)

    def call(name, request, timeout=12):
        future = clients[name].call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            raise RuntimeError(f"{name} failed")
        return future.result()

    def state(value):
        result = RobotState()
        result.is_diff = True
        result.joint_state.name = ["slide"]
        result.joint_state.position = [value]
        return result

    def valid(value):
        request = GetStateValidity.Request()
        request.group_name = "slider_group"
        request.robot_state = state(value)
        return call("valid", request).valid

    def plan(start, goal):
        stages.clear()
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "slider_group"
        motion.start_state = state(start)
        constraint = Constraints()
        constraint.joint_constraints = [JointConstraint(
            joint_name="slide", position=goal,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
        )]
        motion.goal_constraints = [constraint]
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = 2.0
        response = call("plan", request).motion_plan_response
        drain_deadline = time.monotonic() + 0.4
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return {
            "code": response.error_code.val,
            "message": response.error_code.message,
            "source": response.error_code.source,
            "points": [list(p.positions) for p in response.trajectory.joint_trajectory.points],
            "stages": list(stages),
        }

    try:
        deadline = time.monotonic() + 25
        for name, client in clients.items():
            while not client.wait_for_service(timeout_sec=0.1):
                if process.poll() is not None:
                    raise RuntimeError(f"move_group exited {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"{name} discovery timeout")

        control = plan(0.0, 1.0)
        control_valid = [valid(point[0]) for point in control["points"]]
        adapter_order = [x["stage"] for x in control["stages"] if x["stage"]]
        control_ok = (
            control["code"] == 1 and len(control["points"]) >= 2
            and abs(control["points"][0][0]) < 0.01
            and abs(control["points"][-1][0] - 1.0) < 0.01
            and all(control_valid)
            and adapter_order[:2] == ["CheckStartStateBounds", "CheckStartStateCollision"]
        )

        bounds = plan(1.2, 0.8)
        bounds_stage = next((x for x in bounds["stages"] if x["stage"] == "CheckStartStateBounds"), None)
        bounds_checks = {
            "control_valid": control_ok,
            "pipeline_reports_start_state_invalid": bool(bounds_stage) and bounds_stage["code"] == -26,
            "pipeline_preserves_adapter_metadata": bool(bounds_stage)
                and bounds_stage["message"] == "Start state out of bounds."
                and bounds_stage["source"] == "CheckStartStateBounds",
            "service_preserves_pipeline_error_code": bool(bounds_stage) and bounds["code"] == bounds_stage["code"],
        }

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = state(0.8)
        obj = CollisionObject()
        obj.id = "p3_start_box"
        obj.header.frame_id = "base"
        obj.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.2, 0.2, 0.2]
        pose = Pose()
        pose.orientation.w = 1.0
        obj.primitives = [shape]
        obj.primitive_poses = [pose]
        obj.pose.orientation.w = 1.0
        scene.world.collision_objects = [obj]
        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_ok = call("apply", apply_request).success
        current_valid = valid(0.8)
        start_valid = valid(0.0)
        collision = plan(0.0, 1.0)
        collision_stage = next((x for x in collision["stages"] if x["stage"] == "CheckStartStateCollision"), None)
        message = collision_stage["message"] if collision_stage else ""
        collision_checks = {
            "scene_controls_valid": apply_ok and current_valid and not start_valid,
            "pipeline_reports_start_collision": bool(collision_stage) and collision_stage["code"] == -10,
            "diagnostic_reports_detected_contact": bool(collision_stage) and "0 contact(s)" not in message
                and ("p3_start_box" in message or "slider" in message),
        }

        return [
            {
                "case": "bounds_error_preservation",
                "classification": "pass" if all(bounds_checks.values()) else "violation",
                "controls_pass": bounds_checks["control_valid"] and bounds_checks["pipeline_reports_start_state_invalid"],
                "checks": bounds_checks,
                "observed": {"control": control, "control_point_validity": control_valid, "bounds": bounds},
            },
            {
                "case": "collision_diagnostic_request_state",
                "classification": "pass" if all(collision_checks.values()) else "violation",
                "controls_pass": collision_checks["scene_controls_valid"] and collision_checks["pipeline_reports_start_collision"],
                "checks": collision_checks,
                "observed": {
                    "apply_ok": apply_ok, "current_valid": current_valid, "request_start_valid": start_valid,
                    "collision": collision,
                },
            },
        ]
    finally:
        node.destroy_subscription(stage_sub)
        node.destroy_node()
        terminate(process)
        log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).resolve()
    manifest = {
        "schema": "p3-moveit-pipeline-p0-v1",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "repetitions": args.repetitions,
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
    }
    atomic_json(args.out_dir / "manifest.json", manifest)
    import rclpy
    rclpy.init()
    records = []
    try:
        for repeat in range(1, args.repetitions + 1):
            root = args.out_dir / f"repeat-{repeat:02d}"
            root.mkdir()
            try:
                current = run_once(root, repeat)
            except Exception as exc:
                current = [{
                    "case": "fixture", "classification": "execution_error", "controls_pass": False,
                    "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc(),
                }]
            for record in current:
                record["repeat"] = repeat
                records.append(record)
            atomic_json(root / "result.json", current)
            print(json.dumps({"repeat": repeat, "classifications": [x["classification"] for x in current]}), flush=True)
    finally:
        rclpy.shutdown()
    counts = {key: sum(x["classification"] == key for x in records)
              for key in ("pass", "violation", "uncertain", "execution_error")}
    atomic_json(args.out_dir / "summary.json", {
        "schema": manifest["schema"], "counts": counts, "records": records,
        "completed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    })
    print(json.dumps(counts, sort_keys=True))
    return 1 if counts["execution_error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
