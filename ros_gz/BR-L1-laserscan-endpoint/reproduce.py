#!/usr/bin/env python3
"""Small source-guided LaserScan, PointCloud2, and Odometry bridge regression."""

import argparse
import hashlib
import json
import os
import struct
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from map_semantic_regression import atomic_json, terminate


def start_bridge(directory, label, spec):
    log = (directory / f"{label}.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", spec],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return process, log


def ros_to_gz(directory, label, topic, ros_type, gz_name, gz_type, message, sample, matches):
    import rclpy
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node

    ros_name = f"{ros_type.__module__.split('.')[0]}/msg/{ros_type.__name__}"
    process, log = start_bridge(
        directory, label, f"{topic}@{ros_name}]gz.msgs.{gz_name}")
    node = Node(f"p3_{label}_{os.getpid()}")
    received = []
    gz_node = GzNode()
    gz_node.subscribe(gz_type, topic, lambda msg: received.append(sample(msg)))
    publisher = node.create_publisher(ros_type, topic, 10)
    try:
        deadline = time.monotonic() + 8
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"{label} ROS subscription unavailable")
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            for value in received:
                if matches(value):
                    return value
        raise RuntimeError(f"{label} produced no matching Gazebo message")
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def gz_to_ros(directory, label, topic, ros_type, gz_name, gz_type, message, sample, matches):
    import rclpy
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node

    ros_name = f"{ros_type.__module__.split('.')[0]}/msg/{ros_type.__name__}"
    process, log = start_bridge(
        directory, label, f"{topic}@{ros_name}[gz.msgs.{gz_name}")
    node = Node(f"p3_{label}_{os.getpid()}")
    received = []
    node.create_subscription(ros_type, topic, lambda msg: received.append(sample(msg)), 10)
    gz_node = GzNode()
    publisher = gz_node.advertise(topic, gz_type)
    try:
        deadline = time.monotonic() + 8
        while not publisher.has_connections() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not publisher.has_connections():
            raise RuntimeError(f"{label} Gazebo subscription unavailable")
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            for value in received:
                if matches(value):
                    return value
        raise RuntimeError(f"{label} produced no matching ROS message")
    finally:
        terminate(process)
        log.close()
        node.destroy_node()


def header_data(header):
    return {item.key: list(item.value) for item in header.data}


def run_laser(directory, repeat):
    from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
    from sensor_msgs.msg import LaserScan

    def gz_sample(msg):
        return {"count": msg.count, "ranges": list(msg.ranges),
                "intensities": list(msg.intensities), "angle_min": msg.angle_min,
                "angle_max": msg.angle_max, "angle_step": msg.angle_step,
                "frame": msg.frame}

    def ros_sample(msg):
        return {"ranges": list(msg.ranges), "intensities": list(msg.intensities),
                "angle_min": msg.angle_min, "angle_max": msg.angle_max,
                "angle_step": msg.angle_increment, "frame": msg.header.frame_id}

    ros = LaserScan()
    ros.header.frame_id = "laser"
    ros.angle_min = 0.0
    ros.angle_max = 0.2
    ros.angle_increment = 0.1
    ros.range_min = 0.1
    ros.range_max = 10.0
    ros.ranges = [1.1, 1.2, 1.3]
    ros.intensities = [11.0, 12.0, 13.0]
    r2g = ros_to_gz(
        directory, "laser_r2g", f"/p3_laser_r2g_{repeat}_{os.getpid()}",
        LaserScan, "LaserScan", GzLaserScan, ros, gz_sample,
        lambda value: value["ranges"] and abs(value["ranges"][0] - 1.1) < 1e-5)

    gz = GzLaserScan()
    gz.frame = "laser::frame"
    gz.angle_min = 0.0
    gz.angle_max = 0.2
    gz.angle_step = 0.1
    gz.range_min = 0.1
    gz.range_max = 10.0
    gz.count = 3
    gz.vertical_count = 1
    gz.ranges.extend([2.1, 2.2, 2.3])
    gz.intensities.extend([21.0, 22.0, 23.0])
    g2r = gz_to_ros(
        directory, "laser_g2r", f"/p3_laser_g2r_{repeat}_{os.getpid()}",
        LaserScan, "LaserScan", GzLaserScan, gz, ros_sample,
        lambda value: value["ranges"] and abs(value["ranges"][0] - 2.1) < 1e-5)
    checks = {
        "ros_to_gz_preserves_three_aligned_samples":
        len(r2g["ranges"]) == 3 and len(r2g["intensities"]) == 3 and r2g["count"] == 3,
        "gz_to_ros_preserves_three_aligned_samples":
        len(g2r["ranges"]) == 3 and len(g2r["intensities"]) == 3,
        "gz_to_ros_converts_frame_delimiter": g2r["frame"] == "laser/frame",
    }
    return {"ros_to_gz": r2g, "gz_to_ros": g2r, "checks": checks}


def cloud_bytes(values, padding):
    return b"".join(
        struct.pack("<fHxx", x, tag) + (padding if index % 2 else b"")
        for index, (x, tag) in enumerate(values, 1))


def run_cloud(directory, repeat):
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    from sensor_msgs.msg import PointCloud2, PointField

    def gz_sample(msg):
        return {"height": msg.height, "width": msg.width, "point_step": msg.point_step,
                "row_step": msg.row_step, "bigendian": msg.is_bigendian,
                "dense": msg.is_dense, "data": list(msg.data),
                "fields": [[f.name, f.offset, f.datatype, f.count] for f in msg.field]}

    def ros_sample(msg):
        return {"height": msg.height, "width": msg.width, "point_step": msg.point_step,
                "row_step": msg.row_step, "bigendian": msg.is_bigendian,
                "dense": msg.is_dense, "data": list(msg.data),
                "fields": [[f.name, f.offset, f.datatype, f.count] for f in msg.fields]}

    ros_data = cloud_bytes([(1.0, 10), (2.0, 20), (3.0, 30), (4.0, 40)], b"\xaa\xbb\xcc\xdd")
    ros = PointCloud2()
    ros.height = 2
    ros.width = 2
    ros.point_step = 8
    ros.row_step = 20
    ros.is_dense = False
    ros.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="tag", offset=4, datatype=PointField.UINT16, count=1)]
    ros.data = list(ros_data)
    r2g = ros_to_gz(
        directory, "cloud_r2g", f"/p3_cloud_r2g_{repeat}_{os.getpid()}",
        PointCloud2, "PointCloudPacked", PointCloudPacked, ros, gz_sample,
        lambda value: value["data"] == list(ros_data))

    gz_data = cloud_bytes([(5.0, 50), (6.0, 60), (7.0, 70), (8.0, 80)], b"\x11\x22\x33\x44")
    gz = PointCloudPacked()
    gz.height = 2
    gz.width = 2
    gz.point_step = 8
    gz.row_step = 20
    gz.is_dense = False
    for name, offset, datatype in (("x", 0, PointCloudPacked.Field.FLOAT32),
                                   ("tag", 4, PointCloudPacked.Field.UINT16)):
        field = gz.field.add()
        field.name, field.offset, field.datatype, field.count = name, offset, datatype, 1
    gz.data = gz_data
    g2r = gz_to_ros(
        directory, "cloud_g2r", f"/p3_cloud_g2r_{repeat}_{os.getpid()}",
        PointCloud2, "PointCloudPacked", PointCloudPacked, gz, ros_sample,
        lambda value: value["data"] == list(gz_data))
    expected_ros_fields = [["x", 0, PointField.FLOAT32, 1],
                           ["tag", 4, PointField.UINT16, 1]]
    expected_gz_fields = [["x", 0, PointCloudPacked.Field.FLOAT32, 1],
                          ["tag", 4, PointCloudPacked.Field.UINT16, 1]]
    checks = {
        "ros_to_gz_preserves_layout_padding_and_fields":
        r2g == {"height": 2, "width": 2, "point_step": 8, "row_step": 20,
                "bigendian": False, "dense": False, "data": list(ros_data),
                "fields": expected_gz_fields},
        "gz_to_ros_preserves_layout_padding_and_fields":
        g2r == {"height": 2, "width": 2, "point_step": 8, "row_step": 20,
                "bigendian": False, "dense": False, "data": list(gz_data),
                "fields": expected_ros_fields},
    }
    return {"ros_to_gz": r2g, "gz_to_ros": g2r, "checks": checks}


def run_odom(directory, repeat):
    from gz.msgs10.odometry_with_covariance_pb2 import OdometryWithCovariance
    from nav_msgs.msg import Odometry

    def gz_sample(msg):
        return {"stamp": [msg.header.stamp.sec, msg.header.stamp.nsec],
                "header": header_data(msg.header),
                "position": [msg.pose_with_covariance.pose.position.x,
                             msg.pose_with_covariance.pose.position.y,
                             msg.pose_with_covariance.pose.position.z],
                "linear": [msg.twist_with_covariance.twist.linear.x,
                           msg.twist_with_covariance.twist.linear.y,
                           msg.twist_with_covariance.twist.linear.z],
                "pose_cov": list(msg.pose_with_covariance.covariance.data),
                "twist_cov": list(msg.twist_with_covariance.covariance.data)}

    def ros_sample(msg):
        return {"stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
                "frame": msg.header.frame_id, "child": msg.child_frame_id,
                "position": [msg.pose.pose.position.x, msg.pose.pose.position.y,
                             msg.pose.pose.position.z],
                "linear": [msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z],
                "pose_cov": list(msg.pose.covariance),
                "twist_cov": list(msg.twist.covariance)}

    ros = Odometry()
    ros.header.stamp.sec, ros.header.stamp.nanosec = 7, 8
    ros.header.frame_id, ros.child_frame_id = "odom", "base_link"
    ros.pose.pose.position.x, ros.pose.pose.position.y, ros.pose.pose.position.z = 1.0, 2.0, 3.0
    ros.pose.pose.orientation.w = 1.0
    ros.twist.twist.linear.x, ros.twist.twist.linear.y, ros.twist.twist.linear.z = 4.0, 5.0, 6.0
    ros.pose.covariance = [float(i) for i in range(36)]
    ros.twist.covariance = [float(100 + i) for i in range(36)]
    r2g = ros_to_gz(
        directory, "odom_r2g", f"/p3_odom_r2g_{repeat}_{os.getpid()}",
        Odometry, "OdometryWithCovariance", OdometryWithCovariance, ros, gz_sample,
        lambda value: value["stamp"] == [7, 8])

    gz = OdometryWithCovariance()
    gz.header.stamp.sec, gz.header.stamp.nsec = 9, 10
    for key, value in (("frame_id", "odom::map"), ("child_frame_id", "robot::base")):
        pair = gz.header.data.add()
        pair.key = key
        pair.value.append(value)
    gz.pose_with_covariance.pose.position.x = 11.0
    gz.pose_with_covariance.pose.position.y = 12.0
    gz.pose_with_covariance.pose.position.z = 13.0
    gz.pose_with_covariance.pose.orientation.w = 1.0
    gz.twist_with_covariance.twist.linear.x = 14.0
    gz.twist_with_covariance.twist.linear.y = 15.0
    gz.twist_with_covariance.twist.linear.z = 16.0
    gz.pose_with_covariance.covariance.data.extend(float(200 + i) for i in range(36))
    gz.twist_with_covariance.covariance.data.extend(float(300 + i) for i in range(36))
    g2r = gz_to_ros(
        directory, "odom_g2r", f"/p3_odom_g2r_{repeat}_{os.getpid()}",
        Odometry, "OdometryWithCovariance", OdometryWithCovariance, gz, ros_sample,
        lambda value: value["stamp"] == [9, 10])
    checks = {
        "ros_to_gz_preserves_frames_values_and_covariance":
        r2g["header"].get("frame_id") == ["odom"] and
        r2g["header"].get("child_frame_id") == ["base_link"] and
        r2g["position"] == [1.0, 2.0, 3.0] and r2g["linear"] == [4.0, 5.0, 6.0] and
        r2g["pose_cov"] == [float(i) for i in range(36)] and
        r2g["twist_cov"] == [float(100 + i) for i in range(36)],
        "gz_to_ros_preserves_frames_values_and_covariance":
        g2r["frame"] == "odom/map" and g2r["child"] == "robot/base" and
        g2r["position"] == [11.0, 12.0, 13.0] and g2r["linear"] == [14.0, 15.0, 16.0] and
        g2r["pose_cov"] == [float(200 + i) for i in range(36)] and
        g2r["twist_cov"] == [float(300 + i) for i in range(36)],
    }
    return {"ros_to_gz": r2g, "gz_to_ros": g2r, "checks": checks}


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
    source_value = os.environ.get("ROS_GZ_SOURCE_ROOT")
    source_root = (Path(source_value) / "ros_gz_bridge/src/convert" if source_value else None)
    manifest = {
        "schema": 1, "started_at": datetime.now(timezone.utc).isoformat(),
        "repetitions": args.repetitions,
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "source_sha256": ({name: hashlib.sha256((source_root / name).read_bytes()).hexdigest()
                           for name in ("sensor_msgs.cpp", "nav_msgs.cpp", "geometry_msgs.cpp")}
                          if source_root else {}),
        "package": subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Package} ${Version}",
             "ros-jazzy-ros-gz-bridge"], text=True),
    }
    atomic_json(root / "manifest.json", manifest)
    os.environ["GZ_PARTITION"] = f"p3-bridge-structured-{os.getpid()}"
    import rclpy
    rclpy.init()
    results = []
    try:
        for repeat in range(args.repetitions):
            directory = root / f"repeat-{repeat:02d}"
            directory.mkdir()
            try:
                result = {"repeat": repeat, "status": "observed",
                          "laser": run_laser(directory, repeat),
                          "cloud": run_cloud(directory, repeat),
                          "odom": run_odom(directory, repeat)}
            except Exception as error:
                result = {"repeat": repeat, "status": "execution_error", "error": repr(error)}
            results.append(result)
            atomic_json(directory / "result.json", result)
            atomic_json(root / "progress.json", {"manifest": manifest, "results": results})
            print(json.dumps(result), flush=True)
    finally:
        rclpy.shutdown()
    observed = [item for item in results if item["status"] == "observed"]
    summary = {"manifest": manifest, "completed": len(results),
               "execution_errors": sum(item["status"] != "observed" for item in results),
               "laser_failures": sum(not all(item["laser"]["checks"].values()) for item in observed),
               "cloud_failures": sum(not all(item["cloud"]["checks"].values()) for item in observed),
               "odom_failures": sum(not all(item["odom"]["checks"].values()) for item in observed),
               "results": results}
    atomic_json(root / "summary.json", summary)
    print(json.dumps({key: summary[key] for key in
                      ("completed", "execution_errors", "laser_failures",
                       "cloud_failures", "odom_failures")}), flush=True)


if __name__ == "__main__":
    main()
