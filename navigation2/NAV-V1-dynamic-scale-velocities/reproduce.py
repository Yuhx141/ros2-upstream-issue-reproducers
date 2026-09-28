#!/usr/bin/env python3
"""Small source-guided semantic probes for the remaining Nav2 components."""

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from map_semantic_regression import atomic_json, call, terminate


VELOCITY_PARAMS = """{name}:
  ros__parameters:
    enable_stamped_cmd_vel: false
    feedback: OPEN_LOOP
    smoothing_frequency: 10.0
    velocity_timeout: 5.0
    scale_velocities: {scale}
    max_velocity: [0.5, 0.5, 1.0]
    min_velocity: [-0.5, -0.5, -1.0]
    max_accel: [1.0, 1.0, 10.0]
    max_decel: [-1.0, -1.0, -10.0]
    deadband_velocity: [0.0, 0.0, 0.0]
"""


def transition(node, client, code):
    from lifecycle_msgs.srv import ChangeState

    request = ChangeState.Request()
    request.transition.id = code
    if not call(node, client, request).success:
        raise RuntimeError(f"lifecycle transition {code} rejected")


def run_velocity_arm(
        directory, repeat, arm, initial_scale, dynamic_scale=None, dynamic_accel=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient

    name = f"p3_velocity_semantic_{repeat}_{arm}"
    command_topic = f"/{name}/cmd_vel"
    output_topic = f"/{name}/cmd_vel_smoothed"
    params = directory / f"{arm}.yaml"
    params.write_text(
        VELOCITY_PARAMS.format(name=name, scale=str(initial_scale).lower()), encoding="utf-8")
    log = (directory / f"{arm}.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "nav2_velocity_smoother", "velocity_smoother", "--ros-args",
         "-r", f"__node:={name}", "-r", f"cmd_vel:={command_topic}",
         "-r", f"cmd_vel_smoothed:={output_topic}", "--params-file", str(params)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    node = Node(f"p3_velocity_observer_{repeat}_{arm}_{os.getpid()}")
    outputs = []
    try:
        change = node.create_client(ChangeState, f"/{name}/change_state")
        transition(node, change, Transition.TRANSITION_CONFIGURE)
        transition(node, change, Transition.TRANSITION_ACTIVATE)
        parameter_result = None
        parameter_value = initial_scale
        if dynamic_scale is not None or dynamic_accel is not None:
            parameters = AsyncParameterClient(node, f"/{name}")
            if not parameters.wait_for_services(timeout_sec=8):
                raise RuntimeError("velocity parameter service unavailable")
            parameter_name = "scale_velocities" if dynamic_accel is None else "max_accel"
            parameter_update = dynamic_scale if dynamic_accel is None else dynamic_accel
            future = parameters.set_parameters(
                [Parameter(parameter_name, value=parameter_update)])
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done() or future.exception() is not None:
                raise RuntimeError("velocity parameter update failed")
            parameter_result = future.result().results[0].successful
            future = parameters.get_parameters([parameter_name])
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done() or future.exception() is not None:
                raise RuntimeError("velocity parameter readback failed")
            value = future.result().values[0]
            parameter_value = (
                value.bool_value if dynamic_accel is None else list(value.double_array_value))

        node.create_subscription(
            Twist, output_topic,
            lambda message: outputs.append([message.linear.x, message.linear.y, message.angular.z]),
            10,
        )
        publisher = node.create_publisher(Twist, command_topic, 10)
        command = Twist()
        command.linear.x = 0.5
        command.angular.z = 1.0
        deadline = time.monotonic() + 3
        while not outputs and time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        if not outputs:
            raise RuntimeError("velocity smoother produced no output")
        first = outputs[0]
        try:
            transition(node, change, Transition.TRANSITION_DEACTIVATE)
            transition(node, change, Transition.TRANSITION_CLEANUP)
        except Exception:
            pass
        return {
            "initial_scale": initial_scale,
            "dynamic_scale": dynamic_scale,
            "dynamic_accel": dynamic_accel,
            "parameter_update_success": parameter_result,
            "parameter_readback": parameter_value,
            "first_output": first,
        }
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_velocity(directory, repeat):
    arms = {
        "configured_false": run_velocity_arm(directory, repeat, "configured_false", False),
        "configured_true": run_velocity_arm(directory, repeat, "configured_true", True),
        "dynamic_false_to_true": run_velocity_arm(
            directory, repeat, "dynamic_false_to_true", False, True),
        "dynamic_accel_1_to_2": run_velocity_arm(
            directory, repeat, "dynamic_accel_1_to_2", False,
            dynamic_accel=[2.0, 1.0, 10.0]),
    }
    false_output = arms["configured_false"]["first_output"]
    true_output = arms["configured_true"]["first_output"]
    dynamic_output = arms["dynamic_false_to_true"]["first_output"]
    accel_output = arms["dynamic_accel_1_to_2"]["first_output"]

    def close(observed, expected):
        return max(abs(a - b) for a, b in zip(observed, expected)) < 1e-6

    checks = {
        "configured_false_matches_independent_axis_formula": close(false_output, [0.1, 0.0, 1.0]),
        "configured_true_matches_vector_scale_formula": close(true_output, [0.1, 0.0, 0.2]),
        "dynamic_parameter_update_accepted":
        arms["dynamic_false_to_true"]["parameter_update_success"] is True,
        "dynamic_parameter_reads_back_true":
        arms["dynamic_false_to_true"]["parameter_readback"] is True,
        "dynamic_behavior_matches_configured_true": close(dynamic_output, [0.1, 0.0, 0.2]),
        "dynamic_accel_update_accepted":
        arms["dynamic_accel_1_to_2"]["parameter_update_success"] is True,
        "dynamic_accel_reads_back_requested_value":
        close(arms["dynamic_accel_1_to_2"]["parameter_readback"], [2.0, 1.0, 10.0]),
        "dynamic_accel_changes_first_output": close(accel_output, [0.2, 0.0, 1.0]),
    }
    return {
        "arms": arms,
        "checks": checks,
        "violation": not checks["dynamic_behavior_matches_configured_true"],
        "violation_signature": (
            "parameter_true_behavior_unscaled" if
            close(dynamic_output, [0.1, 0.0, 1.0]) else None),
        "oracle_pass": all(value for key, value in checks.items()
                           if key != "dynamic_behavior_matches_configured_true"),
    }


def collision_configuration(name, arm, polygons):
    polygon_names = ", ".join(item["name"] for item in polygons)
    sections = []
    for item in polygons:
        extra = ""
        if item["action"] == "slowdown":
            extra = f"\n      slowdown_ratio: {item['slowdown_ratio']}"
        elif item["action"] == "limit":
            extra = (f"\n      linear_limit: {item['linear_limit']}"
                     f"\n      angular_limit: {item['angular_limit']}")
        elif item["action"] == "approach":
            extra = (f"\n      time_before_collision: {item['time_before_collision']}"
                     f"\n      simulation_time_step: {item['simulation_time_step']}")
        sections.append(
            f"    {item['name']}:\n"
            "      type: polygon\n"
            f"      points: '{item['points']}'\n"
            f"      action_type: {item['action']}\n"
            f"      min_points: {item['min_points']}\n"
            f"      enabled: true{extra}\n")
    prefix = f"/{name}/{arm}"
    return f"""{name}:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: {prefix}/cmd_vel_in
    cmd_vel_out_topic: {prefix}/cmd_vel_out
    state_topic: {prefix}/state
    enable_stamped_cmd_vel: false
    source_timeout: 2.0
    stop_pub_timeout: 1.0
    polygons: [{polygon_names}]
{''.join(sections)}    observation_sources: [scan]
    scan:
      type: scan
      topic: {prefix}/scan
      enabled: true
"""


def run_collision_arm(directory, repeat, arm, polygons, phases):
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    name = f"p3_collision_semantic_{repeat}_{arm}"
    prefix = f"/{name}/{arm}"
    params = directory / f"collision-{arm}.yaml"
    params.write_text(collision_configuration(name, arm, polygons), encoding="utf-8")
    log = (directory / f"collision-{arm}.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "nav2_collision_monitor", "collision_monitor", "--ros-args",
         "-r", f"__node:={name}", "--params-file", str(params)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    node = Node(f"p3_collision_observer_{repeat}_{arm}_{os.getpid()}")
    outputs = []
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        change = node.create_client(ChangeState, f"/{name}/change_state")
        transition(node, change, Transition.TRANSITION_CONFIGURE)
        transition(node, change, Transition.TRANSITION_ACTIVATE)
        node.create_subscription(
            Twist, prefix + "/cmd_vel_out",
            lambda message: outputs.append([message.linear.x, message.angular.z]), 10)
        commands = node.create_publisher(Twist, prefix + "/cmd_vel_in", 10)
        scans = node.create_publisher(LaserScan, prefix + "/scan", 10)

        def phase(distance, points, command_values=(0.4, 1.0)):
            outputs.clear()
            deadline = time.monotonic() + 3
            while len(outputs) < 3 and time.monotonic() < deadline:
                scan = LaserScan()
                scan.header.frame_id = "base_footprint"
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.angle_min = -0.05 * (points - 1)
                scan.angle_max = 0.05 * (points - 1)
                scan.angle_increment = 0.1
                scan.range_min = 0.01
                scan.range_max = 10.0
                scan.ranges = [distance] * points
                scans.publish(scan)
                rclpy.spin_once(node, timeout_sec=0.02)
                command = Twist()
                command.linear.x = command_values[0]
                command.angular.z = command_values[1]
                commands.publish(command)
                rclpy.spin_once(node, timeout_sec=0.02)
            if len(outputs) < 3:
                raise RuntimeError(f"collision {arm} produced insufficient output")
            return outputs[-3:]

        observed = {
            item["name"]: phase(
                item["distance"], item["points"], item.get("command", (0.4, 1.0)))
            for item in phases
        }
        try:
            transition(node, change, Transition.TRANSITION_DEACTIVATE)
            transition(node, change, Transition.TRANSITION_CLEANUP)
        except Exception:
            pass
        del broadcaster
        return observed
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_collision(directory, repeat):
    front_03 = "[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]"
    front_06 = "[[0.6, 0.6], [0.6, -0.6], [0.0, -0.6], [0.0, 0.6]]"
    front_09 = "[[0.9, 0.9], [0.9, -0.9], [0.0, -0.9], [0.0, 0.9]]"
    stop = {"name": "Stop", "points": front_03, "action": "stop", "min_points": 4}
    slowdown = {"name": "Slow", "points": front_06, "action": "slowdown",
                "min_points": 1, "slowdown_ratio": 0.5}
    limit = {"name": "Limit", "points": front_09, "action": "limit",
             "min_points": 1, "linear_limit": 0.15, "angular_limit": 0.4}
    approach = {"name": "Approach", "points": front_03, "action": "approach",
                "min_points": 1, "time_before_collision": 2.0,
                "simulation_time_step": 0.1}
    arms = {
        "stop_threshold": run_collision_arm(
            directory, repeat, "stop_threshold", [stop],
            [{"name": "far", "distance": 5.0, "points": 4},
             {"name": "below", "distance": 0.2, "points": 3},
             {"name": "at", "distance": 0.2, "points": 4}]),
        "slowdown": run_collision_arm(
            directory, repeat, "slowdown", [slowdown],
            [{"name": "near", "distance": 0.4, "points": 1}]),
        "limit": run_collision_arm(
            directory, repeat, "limit", [limit],
            [{"name": "near", "distance": 0.4, "points": 1}]),
        "priority": run_collision_arm(
            directory, repeat, "priority", [slowdown, stop],
            [{"name": "near", "distance": 0.2, "points": 4}]),
        "approach": run_collision_arm(
            directory, repeat, "approach", [approach],
            [{"name": "far", "distance": 1.5, "points": 1,
              "command": [0.4, 0.0]},
             {"name": "predicted", "distance": 0.71, "points": 1,
              "command": [0.4, 0.0]},
             {"name": "inside", "distance": 0.2, "points": 1,
              "command": [0.4, 0.0]}]),
    }

    def tail(name, phase):
        return arms[name][phase][-1]

    def close(observed, expected, tolerance=1e-6):
        return max(abs(a - b) for a, b in zip(observed, expected)) < tolerance

    checks = {
        "far_scan_passes_command": close(tail("stop_threshold", "far"), [0.4, 1.0]),
        "below_min_points_passes_command": close(
            tail("stop_threshold", "below"), [0.4, 1.0]),
        "at_min_points_stops": close(tail("stop_threshold", "at"), [0.0, 0.0]),
        "slowdown_scales_both_axes": close(tail("slowdown", "near"), [0.2, 0.5]),
        "limit_preserves_curvature_with_most_restrictive_ratio": close(
            tail("limit", "near"), [0.15, 0.375]),
        "stop_overrides_slowdown": close(tail("priority", "near"), [0.0, 0.0]),
        "approach_outside_horizon_passes_command": close(
            tail("approach", "far"), [0.4, 0.0]),
        "approach_scales_from_independent_ttc": close(
            tail("approach", "predicted"), [0.205, 0.0], tolerance=0.021),
        "approach_inside_footprint_stops": close(
            tail("approach", "inside"), [0.0, 0.0]),
    }
    return {"arms": arms, "checks": checks, "pass": all(checks.values())}


def controller_configuration(name, allow_reversing=False):
    return f"""{name}:
  ros__parameters:
    controller_frequency: 10.0
    odom_topic: odom
    enable_stamped_cmd_vel: false
    progress_checker_plugins: [progress_checker]
    goal_checker_plugins: [goal_checker]
    controller_plugins: [FollowPath]
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.5
      movement_time_allowance: 20.0
    goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
    FollowPath:
      plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
      desired_linear_vel: 0.4
      lookahead_dist: 0.5
      use_velocity_scaled_lookahead_dist: false
      use_regulated_linear_velocity_scaling: false
      use_cost_regulated_linear_velocity_scaling: false
      use_collision_detection: false
      use_rotate_to_heading: false
      allow_reversing: {str(allow_reversing).lower()}
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint
      robot_radius: 0.1
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.1
      update_frequency: 5.0
      plugins: [inflation_layer]
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.3
        cost_scaling_factor: 3.0
"""


def run_controller_arm(directory, repeat, arm, points, allow_reversing=False,
                       percentage_limit=None, absolute_limit=None):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    name = f"p3_controller_semantic_{repeat}_{arm}"
    prefix = f"/{name}/{arm}"
    params = directory / f"controller-{arm}.yaml"
    params.write_text(controller_configuration(name, allow_reversing), encoding="utf-8")
    log = (directory / f"controller-{arm}.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "nav2_controller", "controller_server", "--ros-args",
         "-r", f"__node:={name}", "-r", f"cmd_vel:={prefix}/cmd_vel",
         "-r", f"odom:={prefix}/odom", "-r", f"speed_limit:={prefix}/speed_limit",
         "-r", f"follow_path:={prefix}/follow_path", "--params-file", str(params)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    node = Node(f"p3_controller_observer_{repeat}_{arm}_{os.getpid()}")
    outputs = []
    handle = None
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        change = node.create_client(ChangeState, f"/{name}/change_state")
        transition(node, change, Transition.TRANSITION_CONFIGURE)
        transition(node, change, Transition.TRANSITION_ACTIVATE)
        node.create_subscription(
            Twist, prefix + "/cmd_vel",
            lambda message: outputs.append([message.linear.x, message.angular.z]), 10)
        odom = node.create_publisher(Odometry, prefix + "/odom", 10)
        limits = node.create_publisher(SpeedLimit, prefix + "/speed_limit", 10)
        action = ActionClient(node, FollowPath, prefix + "/follow_path")
        if not action.wait_for_server(timeout_sec=8):
            raise RuntimeError("controller action unavailable")

        def pose(x, y):
            value = PoseStamped()
            value.header.frame_id = "odom"
            value.header.stamp = node.get_clock().now().to_msg()
            value.pose.position.x = x
            value.pose.position.y = y
            value.pose.orientation.w = 1.0
            return value

        path = NavPath()
        path.header.frame_id = "odom"
        path.header.stamp = node.get_clock().now().to_msg()
        path.poses = [pose(x, y) for x, y in points]
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("controller goal submission failed")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("controller rejected valid path")

        def drive():
            outputs.clear()
            deadline = time.monotonic() + 3
            while len(outputs) < 4 and time.monotonic() < deadline:
                message = Odometry()
                message.header.frame_id = "odom"
                message.header.stamp = node.get_clock().now().to_msg()
                message.child_frame_id = "base_footprint"
                message.pose.pose.orientation.w = 1.0
                odom.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
            if len(outputs) < 3:
                raise RuntimeError("controller produced insufficient output")
            return outputs[-3:]

        baseline = drive()
        limited = None
        if percentage_limit is not None:
            limit = SpeedLimit()
            limit.percentage = True
            limit.speed_limit = percentage_limit
            limits.publish(limit)
            rclpy.spin_once(node, timeout_sec=0.1)
            limited = drive()
        elif absolute_limit is not None:
            limit = SpeedLimit()
            limit.percentage = False
            limit.speed_limit = absolute_limit
            limits.publish(limit)
            rclpy.spin_once(node, timeout_sec=0.1)
            limited = drive()

        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=8)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=8)
        status = result.result().status if result.done() else None
        try:
            transition(node, change, Transition.TRANSITION_DEACTIVATE)
            transition(node, change, Transition.TRANSITION_CLEANUP)
        except Exception:
            pass
        del broadcaster
        return {"baseline": baseline, "limited": limited,
                "cancel_status": status,
                "cancelled": status == GoalStatus.STATUS_CANCELED}
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def run_controller(directory, repeat):
    arms = {
        "straight_limit": run_controller_arm(
            directory, repeat, "straight_limit",
            [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)], percentage_limit=50.0),
        "curve": run_controller_arm(
            directory, repeat, "curve",
            [(0.0, 0.0), (0.25, 0.25), (0.5, 0.5), (1.0, 1.0)]),
        "absolute_limit": run_controller_arm(
            directory, repeat, "absolute_limit",
            [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)], absolute_limit=0.15),
        "reverse": run_controller_arm(
            directory, repeat, "reverse",
            [(0.0, 0.0), (-0.5, 0.0), (-1.0, 0.0)], allow_reversing=True),
    }

    def tail(arm, phase="baseline"):
        return arms[arm][phase][-1]

    def close(observed, expected, tolerance=1e-6):
        return max(abs(a - b) for a, b in zip(observed, expected)) < tolerance

    curve_curvature = 2.0 * (0.5 / 2 ** 0.5) / (0.5 * 0.5)
    checks = {
        "straight_command_matches_desired_speed": close(
            tail("straight_limit"), [0.4, 0.0]),
        "percentage_speed_limit_uses_base_speed": close(
            tail("straight_limit", "limited"), [0.2, 0.0]),
        "absolute_speed_limit_uses_meters_per_second": close(
            tail("absolute_limit", "limited"), [0.15, 0.0]),
        "curve_matches_pure_pursuit_geometry": close(
            tail("curve"), [0.4, 0.4 * curve_curvature], tolerance=1e-5),
        "reverse_path_commands_negative_linear_speed": close(
            tail("reverse"), [-0.4, 0.0]),
        "all_goals_cancel_cleanly": all(item["cancelled"] for item in arms.values()),
    }
    return {"arms": arms, "checks": checks, "pass": all(checks.values()),
            "expected_curve_curvature": curve_curvature}


def sg_filter(points):
    weights = [-2 / 21, 3 / 21, 6 / 21, 7 / 21, 6 / 21, 3 / 21, -2 / 21]
    result = list(points)
    for index in range(1, len(points) - 1):
        window = [points[min(max(index + offset, 0), len(points) - 1)]
                  for offset in range(-3, 4)]
        result[index] = tuple(sum(weight * point[axis]
                                  for weight, point in zip(weights, window))
                              for axis in (0, 1))
    return result


def run_smoother(directory, repeat):
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import SmoothPath
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node

    name = f"p3_smoother_semantic_{repeat}"
    action_name = f"/{name}/smooth_path"
    params = directory / "smoother.yaml"
    params.write_text(f"""{name}:
  ros__parameters:
    smoother_plugins: [SG, SGRefined]
    SG:
      plugin: nav2_smoother::SavitzkyGolaySmoother
      do_refinement: false
    SGRefined:
      plugin: nav2_smoother::SavitzkyGolaySmoother
      do_refinement: true
      refinement_num: 2
""", encoding="utf-8")
    log = (directory / "smoother.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "nav2_smoother", "smoother_server", "--ros-args",
         "-r", f"__node:={name}", "-r", f"smooth_path:={action_name}",
         "--params-file", str(params)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    node = Node(f"p3_smoother_observer_{repeat}_{os.getpid()}")
    try:
        change = node.create_client(ChangeState, f"/{name}/change_state")
        transition(node, change, Transition.TRANSITION_CONFIGURE)
        transition(node, change, Transition.TRANSITION_ACTIVATE)
        action = ActionClient(node, SmoothPath, action_name)
        if not action.wait_for_server(timeout_sec=8):
            raise RuntimeError("smoother action unavailable")

        def make_path(points):
            path = NavPath()
            path.header.frame_id = "map"
            path.header.stamp = node.get_clock().now().to_msg()
            for x, y in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            return path

        def smooth(points, plugin):
            goal = SmoothPath.Goal()
            goal.path = make_path(points)
            goal.smoother_id = plugin
            goal.max_smoothing_duration = Duration(seconds=1).to_msg()
            goal.check_for_collisions = False
            future = action.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done() or future.exception() is not None:
                raise RuntimeError("smoother goal submission failed")
            handle = future.result()
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=8)
            if not result_future.done() or result_future.exception() is not None:
                raise RuntimeError("smoother action result failed")
            wrapped = result_future.result()
            return {
                "accepted": handle.accepted,
                "status": wrapped.status,
                "error_code": wrapped.result.error_code,
                "was_completed": wrapped.result.was_completed,
                "points": [[pose.pose.position.x, pose.pose.position.y]
                           for pose in wrapped.result.path.poses],
                "orientations": [[pose.pose.orientation.x, pose.pose.orientation.y,
                                  pose.pose.orientation.z, pose.pose.orientation.w]
                                 for pose in wrapped.result.path.poses],
            }

        eleven = [(index / 10, 0.0 if index in (0, 10) else
                   (0.02 if index % 2 else -0.02)) for index in range(11)]
        ten = eleven[:10]
        one_pass = smooth(eleven, "SG")
        refined = smooth(eleven, "SGRefined")
        ten_result = smooth(ten, "SG")
        expected_one = sg_filter(eleven)
        expected_refined = sg_filter(sg_filter(expected_one))

        def positions_match(observed, expected):
            return len(observed) == len(expected) and all(
                max(abs(a - b) for a, b in zip(actual, wanted)) < 1e-9
                for actual, wanted in zip(observed, expected))

        def orientations_match(raw, points):
            if len(raw) != len(points):
                return False
            for index in range(len(points) - 1):
                yaw = math.atan2(
                    points[index + 1][1] - points[index][1],
                    points[index + 1][0] - points[index][0])
                expected = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
                direct = max(abs(a - b) for a, b in zip(raw[index], expected))
                inverse = max(abs(a + b) for a, b in zip(raw[index], expected))
                if min(direct, inverse) >= 1e-9:
                    return False
            return True

        checks = {
            "single_pass_succeeds": one_pass["accepted"] and one_pass["status"] == 4 and
            one_pass["error_code"] == 0 and one_pass["was_completed"],
            "single_pass_matches_7_point_filter": positions_match(
                one_pass["points"], expected_one),
            "single_pass_orientations_match_tangents": orientations_match(
                one_pass["orientations"], expected_one),
            "refinement_matches_three_filter_passes": positions_match(
                refined["points"], expected_refined),
            "ten_pose_path_reports_success_but_is_unchanged":
            ten_result["accepted"] and ten_result["status"] == 4 and
            ten_result["error_code"] == 0 and ten_result["was_completed"] and
            positions_match(ten_result["points"], ten),
        }
        result = {"inputs": {"eleven": eleven, "ten": ten},
                  "one_pass": one_pass, "refined": refined, "ten_pose": ten_result,
                  "expected": {"one_pass": expected_one, "refined": expected_refined},
                  "checks": checks, "pass": all(checks.values()),
                  "ten_pose_noop_source_risk": checks["ten_pose_path_reports_success_but_is_unchanged"]}
        try:
            transition(node, change, Transition.TRANSITION_DEACTIVATE)
            transition(node, change, Transition.TRANSITION_CLEANUP)
        except Exception:
            pass
        return result
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
    manifest = {
        "schema": 1,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "repetitions": args.repetitions,
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "packages": subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Package} ${Version}\\n",
             "ros-jazzy-nav2-velocity-smoother", "ros-jazzy-nav2-collision-monitor",
             "ros-jazzy-nav2-controller",
             "ros-jazzy-nav2-regulated-pure-pursuit-controller",
             "ros-jazzy-nav2-smoother"], text=True).splitlines(),
        "obligations": ["velocity_scale_configured_false", "velocity_scale_configured_true",
                        "velocity_scale_dynamic_false_to_true", "velocity_dynamic_max_accel",
                        "collision_min_points_boundary",
                        "collision_slowdown", "collision_limit", "collision_action_priority",
                        "collision_approach_ttc", "controller_percentage_speed_limit",
                        "controller_absolute_speed_limit", "controller_curve_geometry",
                        "controller_reverse_path", "smoother_sg_exact_filter",
                        "smoother_sg_refinement", "smoother_ten_pose_boundary"],
    }
    source_value = os.environ.get("NAV2_SOURCE_ROOT")
    source_root = Path(source_value) if source_value else None
    source_files = [
        "nav2_velocity_smoother/src/velocity_smoother.cpp",
        "nav2_collision_monitor/src/collision_monitor_node.cpp",
        "nav2_collision_monitor/src/polygon.cpp",
        "nav2_collision_monitor/src/kinematics.cpp",
        "nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp",
        "nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp",
        "nav2_smoother/src/savitzky_golay_smoother.cpp",
    ]
    manifest["source_sha256"] = ({
        path: hashlib.sha256((source_root / path).read_bytes()).hexdigest()
        for path in source_files
    } if source_root else {})
    atomic_json(root / "manifest.json", manifest)
    import rclpy

    results = []
    rclpy.init()
    try:
        for repeat in range(args.repetitions):
            directory = root / f"repeat-{repeat:02d}"
            directory.mkdir()
            try:
                velocity = run_velocity(directory, repeat)
                collision = run_collision(directory, repeat)
                controller = run_controller(directory, repeat)
                smoother = run_smoother(directory, repeat)
                result = {"repeat": repeat, "status": "observed", "velocity": velocity,
                          "collision": collision, "controller": controller,
                          "smoother": smoother}
            except Exception as error:
                result = {"repeat": repeat, "status": "execution_error", "error": repr(error)}
            atomic_json(directory / "result.json", result)
            results.append(result)
            atomic_json(root / "progress.json", {"manifest": manifest, "results": results})
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    observed = [item for item in results if item["status"] == "observed"]
    summary = {
        "manifest": manifest,
        "completed": len(results),
        "execution_errors": sum(item["status"] != "observed" for item in results),
        "oracle_failures": sum(not item["velocity"]["oracle_pass"] for item in observed),
        "velocity_dynamic_violations": sum(item["velocity"]["violation"] for item in observed),
        "collision_failures": sum(not item["collision"]["pass"] for item in observed),
        "controller_failures": sum(not item["controller"]["pass"] for item in observed),
        "smoother_failures": sum(not item["smoother"]["pass"] for item in observed),
        "smoother_ten_pose_noops": sum(
            item["smoother"]["ten_pose_noop_source_risk"] for item in observed),
        "signatures": {signature: sum(
            item["velocity"]["violation_signature"] == signature for item in observed)
            for signature in {item["velocity"]["violation_signature"] for item in observed}
            if signature},
        "results": results,
    }
    atomic_json(root / "summary.json", summary)
    print(json.dumps({key: summary[key] for key in
                      ("completed", "execution_errors", "oracle_failures",
                       "velocity_dynamic_violations", "collision_failures",
                       "controller_failures", "smoother_failures",
                       "smoother_ten_pose_noops", "signatures")}), flush=True)


if __name__ == "__main__":
    main()
