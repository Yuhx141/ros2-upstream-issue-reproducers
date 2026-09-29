#!/usr/bin/env python3
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

PARAMS = """p3_planner_preempt:
  ros__parameters:
    planner_plugins: [GridBased]
    expected_planner_frequency: 0.0
    costmap_update_timeout: 10.0
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.0
      use_astar: true
      allow_unknown: false
      use_final_approach_orientation: false
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      robot_radius: 0.1
      rolling_window: false
      update_frequency: 10.0
      publish_frequency: 0.0
      track_unknown_space: false
      plugins: [static_layer]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_topic: /map
        map_subscribe_transient_local: true
        subscribe_to_updates: false
"""

def await_future(node, future, timeout=12.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"future failed or timed out: {future.exception() if future.done() else 'timeout'}")
    return future.result()

def pose(node, x):
    value = PoseStamped()
    value.header.frame_id = "map"
    value.header.stamp = node.get_clock().now().to_msg()
    value.pose.position.x = x
    value.pose.position.y = 5.5
    value.pose.orientation.w = 1.0
    return value

def goal(node, x):
    value = ComputePathToPose.Goal()
    value.start = pose(node, 1.5)
    value.goal = pose(node, x)
    value.use_start = True
    value.planner_id = "GridBased"
    return value

def result_json(wrapped):
    path = wrapped.result.path
    return {
        "status": wrapped.status,
        "error_code": wrapped.result.error_code,
        "error_msg": wrapped.result.error_msg,
        "path_size": len(path.poses),
        "path_frame": path.header.frame_id,
        "end": ([path.poses[-1].pose.position.x, path.poses[-1].pose.position.y]
                if path.poses else None),
    }

def map_msg(node):
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.info.resolution = 1.0
    msg.info.width = 10
    msg.info.height = 10
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 100
    return msg

def main():
    if len(sys.argv) != 3 or sys.argv[1] not in {"control", "preempt"}:
        raise SystemExit("usage: planner_preempt_probe.py control|preempt RUN_DIR")
    mode = sys.argv[1]
    run_dir = Path(sys.argv[2]); run_dir.mkdir(parents=True, exist_ok=True)
    params = run_dir / "params.yaml"; params.write_text(PARAMS)
    sut_log = (run_dir / "sut.log").open("w")
    process = subprocess.Popen(
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args",
         "-r", "__node:=p3_planner_preempt", "--params-file", str(params)],
        stdout=sut_log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = Node("planner_preempt_observer")
        published = []
        node.create_subscription(NavPath, "/plan", lambda msg: published.append(msg), 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(OccupancyGrid, "/map", map_qos)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = 1.5
        transform.transform.translation.y = 5.5
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

        lifecycle = node.create_client(ChangeState, "/p3_planner_preempt/change_state")
        if not lifecycle.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("lifecycle service unavailable")
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request(); request.transition.id = transition
            response = await_future(node, lifecycle.call_async(request))
            if not response.success:
                raise RuntimeError(f"lifecycle transition {transition} rejected")

        client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("planner action unavailable")

        old_handle = None
        old_future = None
        if mode == "preempt":
            old_handle = await_future(node, client.send_goal_async(goal(node, 4.5)))
            if not old_handle.accepted:
                raise RuntimeError("old goal rejected")
            old_future = old_handle.get_result_async()

        new_handle = await_future(node, client.send_goal_async(goal(node, 7.5)))
        if not new_handle.accepted:
            raise RuntimeError("new goal rejected")
        new_future = new_handle.get_result_async()

        msg = map_msg(node)
        for _ in range(20):
            msg.header.stamp = node.get_clock().now().to_msg()
            maps.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)

        new_result = await_future(node, new_future)
        old_result = await_future(node, old_future) if old_future is not None else None
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        output = {
            "mode": mode,
            "old_accepted": old_handle.accepted if old_handle else None,
            "new_accepted": new_handle.accepted,
            "old_result": result_json(old_result) if old_result else None,
            "new_result": result_json(new_result),
            "published_count": len(published),
            "published_ends": [[p.poses[-1].pose.position.x, p.poses[-1].pose.position.y]
                               for p in published if p.poses],
        }
        pending = [process.pid]
        process_tree = []
        while pending:
            pid = pending.pop()
            if pid in process_tree:
                continue
            process_tree.append(pid)
            children = Path(f"/proc/{pid}/task/{pid}/children")
            if children.exists():
                pending.extend(int(x) for x in children.read_text().split())
        loaded = {}
        for pid in process_tree:
            maps_file = Path(f"/proc/{pid}/maps")
            exe_file = Path(f"/proc/{pid}/exe")
            if not maps_file.exists():
                continue
            libs = sorted({line.split()[-1] for line in maps_file.read_text(errors="replace").splitlines()
                           if "libplanner_server_core" in line})
            loaded[str(pid)] = {"exe": str(exe_file.resolve()) if exe_file.exists() else None,
                                "planner_libraries": libs}
        output["process_tree"] = loaded
        print(json.dumps(output, sort_keys=True))
        new_ok = new_result.status == 4 and new_result.result.error_code == 0 and \
            new_result.result.path.poses and \
            abs(new_result.result.path.poses[-1].pose.position.x - 7.5) < 1e-9
        old_ok = old_result is None or old_result.status in (5, 6)
        return 0 if new_ok and old_ok else 1
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        sut_log.close()
        (run_dir / "sut.exit").write_text(str(process.returncode) + "\n")

if __name__ == "__main__":
    try: raise SystemExit(main())
    except Exception as ex:
        print(json.dumps({"probe_error": type(ex).__name__, "message": str(ex)}, sort_keys=True))
        raise
