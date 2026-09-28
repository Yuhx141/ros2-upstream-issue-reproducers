#!/usr/bin/env python3
"""Minimal EKF input/effect admission across processing off/on."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import SetPose, ToggleFilterProcessing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--filter", choices=("ekf", "ukf"), default="ekf")
    parser.add_argument("--scenario", choices=("admission", "off-discard", "set-pose-off",
                                               "invalid-set-pose", "valid-set-pose",
                                               "invalid-sensor-frame", "nan-set-pose",
                                               "nan-sensor", "valid-sensor",
                                               "future-invalid-frame", "future-valid-frame",
                                               "nan-covariance", "valid-covariance",
                                               "current-invalid-frame",
                                               "nan-sensor-covariance", "valid-sensor-covariance",
                                               "negative-covariance", "negative-sensor-covariance"),
                        default="admission")
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain must be 0..232")
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = Path(__file__).with_name("ekf.yaml").resolve()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(out / "ros-logs")
    started = time.monotonic_ns()
    trace = (out / "trace.jsonl").open("x", encoding="utf-8")
    log = (out / "ekf.log").open("x", encoding="utf-8")
    process = node = None
    outputs, phases, calls, errors, mismatches, candidate_signatures = [], [], [], [], [], []

    def emit(event, **fields):
        trace.write(json.dumps({"event": event, "elapsed_ns": time.monotonic_ns() - started,
                                **fields}) + "\n")
        trace.flush()

    try:
        rclpy.init()
        node = Node(f"p3_ekf_probe_{os.getpid()}")
        publisher = node.create_publisher(PoseWithCovarianceStamped, "/p3/pose", 10)

        def received(message):
            nonfinite = [
                f"{name}.{axis}"
                for name, vector in (
                    ("pose.position", message.pose.pose.position),
                    ("pose.orientation", message.pose.pose.orientation),
                    ("twist.linear", message.twist.twist.linear),
                    ("twist.angular", message.twist.twist.angular))
                for axis in ("x", "y", "z", "w")
                if hasattr(vector, axis) and not math.isfinite(getattr(vector, axis))]
            nonfinite += [
                f"{name}[{index}]"
                for name, values in (("pose.covariance", message.pose.covariance),
                                     ("twist.covariance", message.twist.covariance))
                for index, value in enumerate(values) if not math.isfinite(value)]
            x = message.pose.pose.position.x
            cov_x = float(message.pose.covariance[0])
            sample = {"x": x if math.isfinite(x) else repr(x),
                      "pose_cov_x": cov_x if math.isfinite(cov_x) else repr(cov_x),
                      "nonfinite_fields": nonfinite,
                      "stamp": [message.header.stamp.sec, message.header.stamp.nanosec],
                      "elapsed_ns": time.monotonic_ns() - started}
            outputs.append(sample)
            emit("odom_received", **sample)

        subscription = node.create_subscription(Odometry, "/odometry/filtered", received, 10)
        client = node.create_client(ToggleFilterProcessing, "/toggle")
        pose_client = (node.create_client(SetPose, "/set_pose")
                       if args.scenario in ("set-pose-off", "invalid-set-pose",
                                            "valid-set-pose", "nan-set-pose",
                                            "future-invalid-frame", "future-valid-frame",
                                            "nan-covariance", "valid-covariance",
                                            "negative-covariance", "current-invalid-frame")
                       else None)
        process = subprocess.Popen(
            [str(Path(subprocess.check_output(["ros2", "pkg", "prefix", "robot_localization"], text=True).strip()) / "lib" / "robot_localization" / f"{args.filter}_node"), "--ros-args",
             "-r", "__node:=ekf_filter_node", "--params-file", str(config)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        emit("process_start", pid=process.pid)
        deadline = time.monotonic() + 15
        while (not client.wait_for_service(timeout_sec=0.05)
               or publisher.get_subscription_count() < 1):
            if process.poll() is not None:
                raise RuntimeError(f"EKF exited during discovery: {process.returncode}")
            if time.monotonic() >= deadline:
                emit("discovery_timeout", services=node.get_service_names_and_types(),
                     topics=node.get_topic_names_and_types(),
                     matched=publisher.get_subscription_count())
                raise TimeoutError("EKF service or input subscription not discovered")
        emit("setup_complete", subscriptions=publisher.get_subscription_count())
        if pose_client is not None and not pose_client.wait_for_service(timeout_sec=5):
            raise TimeoutError("set_pose service not discovered")

        def publish_phase(name, x, seconds, frame="odom", covariance_x=0.01):
            start_index = len(outputs)
            count = 0
            input_x = x if math.isfinite(x) else repr(x)
            input_covariance = covariance_x if math.isfinite(covariance_x) else repr(covariance_x)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                message = PoseWithCovarianceStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = frame
                message.pose.pose.position.x = x
                message.pose.pose.orientation.w = 1.0
                message.pose.covariance[0] = covariance_x
                publisher.publish(message)
                count += 1
                emit("pose_published", phase=name, x=input_x, frame=frame,
                     covariance_x=input_covariance,
                     stamp=[message.header.stamp.sec, message.header.stamp.nanosec])
                rclpy.spin_once(node, timeout_sec=0.05)
            phase_outputs = outputs[start_index:]
            phases.append({"name": name, "input_x": input_x, "frame": frame,
                           "covariance_x": input_covariance, "sent": count,
                           "output_count": len(phase_outputs),
                           "nonfinite_count": sum(bool(s["nonfinite_fields"]) for s in phase_outputs),
                           "negative_cov_count": sum(isinstance(s["pose_cov_x"], (int, float))
                                                     and s["pose_cov_x"] < -1e-9 for s in phase_outputs),
                           "last_pose_cov_x": phase_outputs[-1]["pose_cov_x"] if phase_outputs else None,
                           "last_x": phase_outputs[-1]["x"] if phase_outputs else None})
            emit("phase_end", **phases[-1])

        def observe_phase(name, seconds):
            start_index = len(outputs)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            phase_outputs = outputs[start_index:]
            phases.append({"name": name, "sent": 0, "output_count": len(phase_outputs),
                           "nonfinite_count": sum(bool(s["nonfinite_fields"]) for s in phase_outputs),
                           "negative_cov_count": sum(isinstance(s["pose_cov_x"], (int, float))
                                                     and s["pose_cov_x"] < -1e-9 for s in phase_outputs),
                           "last_pose_cov_x": phase_outputs[-1]["pose_cov_x"] if phase_outputs else None,
                           "last_x": phase_outputs[-1]["x"] if phase_outputs else None})
            emit("phase_end", **phases[-1])

        def toggle(on):
            request = ToggleFilterProcessing.Request(on=on)
            emit("call_start", operation="toggle", on=on)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or future.exception() is not None:
                raise TimeoutError(f"toggle {on} response timeout or error")
            result = bool(future.result().status)
            calls.append({"on": on, "status": result})
            emit("call_end", operation="toggle", on=on, status=result)
            if not result:
                mismatches.append(f"toggle {on} was rejected")

        def set_pose(x, frame, stamp_offset=0, covariance_x=0.01):
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.stamp.sec += stamp_offset
            pose.header.frame_id = frame
            pose.pose.pose.position.x = x
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = covariance_x
            input_x = x if math.isfinite(x) else repr(x)
            input_covariance = covariance_x if math.isfinite(covariance_x) else repr(covariance_x)
            stamp = [pose.header.stamp.sec, pose.header.stamp.nanosec]
            emit("call_start", operation="set_pose", x=input_x, frame=frame,
                 stamp=stamp, stamp_offset=stamp_offset, covariance_x=input_covariance)
            future = pose_client.call_async(SetPose.Request(pose=pose))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise TimeoutError("set_pose response timeout")
            outcome = "client_exception" if future.exception() is not None else "completed"
            calls.append({"operation": "set_pose", "x": input_x, "frame": frame,
                          "stamp": stamp, "stamp_offset": stamp_offset,
                          "covariance_x": input_covariance, "outcome": outcome})
            emit("call_end", operation="set_pose", frame=frame, outcome=outcome,
                 error=str(future.exception()) if future.exception() else None)
            if outcome == "client_exception" and args.scenario in (
                    "set-pose-off", "valid-set-pose", "future-valid-frame",
                    "valid-covariance"):
                mismatches.append("valid set_pose call failed")
            elif outcome == "client_exception" and args.scenario == "negative-covariance":
                mismatches.append("negative set_pose transport exception; business rejection unknown")

        def near(value, target):
            return (isinstance(value, (int, float)) and math.isfinite(value)
                    and abs(value - target) <= 0.5)

        frame_case = args.scenario in (
            "invalid-set-pose", "valid-set-pose", "nan-set-pose",
            "future-invalid-frame", "future-valid-frame",
            "nan-covariance", "valid-covariance", "negative-covariance",
            "current-invalid-frame")
        baseline_x = (2.0 if frame_case or args.scenario in
                      ("invalid-sensor-frame", "nan-sensor", "valid-sensor",
                       "nan-sensor-covariance", "valid-sensor-covariance",
                       "negative-sensor-covariance") else 0.0)
        publish_phase("baseline", baseline_x, 1.0)
        if frame_case:
            set_pose(math.nan if args.scenario == "nan-set-pose" else 5.0,
                     "missing_frame" if args.scenario in (
                         "invalid-set-pose", "future-invalid-frame",
                         "current-invalid-frame") else "odom",
                     stamp_offset=30 if args.scenario == "future-invalid-frame" else 0,
                     covariance_x=(math.nan if args.scenario == "nan-covariance"
                                   else -0.01 if args.scenario == "negative-covariance"
                                   else 0.01))
            observe_phase("after_set_pose", 0.6)
            publish_phase("recovered", 3.0 if args.scenario in (
                "future-invalid-frame", "future-valid-frame",
                "nan-covariance", "valid-covariance", "negative-covariance",
                "current-invalid-frame") else 2.0, 1.0)
        elif args.scenario == "invalid-sensor-frame":
            publish_phase("invalid_sensor", 5.0, 0.6, frame="missing_frame")
            observe_phase("after_invalid_sensor", 0.4)
            publish_phase("recovered", 2.0, 1.0)
        elif args.scenario in ("nan-sensor", "valid-sensor",
                               "nan-sensor-covariance", "valid-sensor-covariance",
                               "negative-sensor-covariance"):
            publish_phase("sensor_update",
                          math.nan if args.scenario == "nan-sensor" else 5.0, 1.0,
                          covariance_x=(math.nan if args.scenario == "nan-sensor-covariance"
                                        else -0.01 if args.scenario == "negative-sensor-covariance"
                                        else 0.01))
            observe_phase("after_sensor", 0.4)
            publish_phase("recovered", 3.0 if args.scenario in (
                "nan-sensor-covariance", "valid-sensor-covariance",
                "negative-sensor-covariance") else 2.0, 1.0)
        else:
            toggle(False)
            publish_phase("off", 3.0, 1.0)
            if args.scenario == "off-discard":
                observe_phase("off_drained", 0.4)
            if args.scenario == "set-pose-off":
                set_pose(5.0, "odom")
                observe_phase("off_after_set_pose", 0.6)
            toggle(True)
            if args.scenario == "off-discard":
                observe_phase("post_resume_no_input", 0.8)
            if args.scenario == "set-pose-off":
                observe_phase("post_resume_no_input", 0.5)
            publish_phase("recovered", 3.0, 2.0)
        phase_by_name = {phase["name"]: phase for phase in phases}
        baseline = phase_by_name["baseline"]
        recovered = phase_by_name["recovered"]
        if (baseline["output_count"] < 3 or baseline["nonfinite_count"]
                or not near(baseline["last_x"], baseline_x)):
            mismatches.append("baseline EKF output missing or wrong")
        if args.scenario in ("negative-covariance", "negative-sensor-covariance",
                             "valid-covariance", "valid-sensor-covariance") and baseline["negative_cov_count"]:
            mismatches.append("baseline covariance was negative")
        if args.scenario == "invalid-sensor-frame":
            for name in ("invalid_sensor", "after_invalid_sensor", "recovered"):
                phase = phase_by_name[name]
                if phase["output_count"] < 3 or abs(phase["last_x"] - 2.0) > 0.5:
                    mismatches.append(f"invalid sensor frame disturbed state in {name}")
        elif args.scenario in ("nan-sensor", "valid-sensor",
                               "nan-sensor-covariance", "valid-sensor-covariance",
                               "negative-sensor-covariance"):
            update = phase_by_name["sensor_update"]
            after = phase_by_name["after_sensor"]
            if update["output_count"] < 3 or after["output_count"] < 3:
                mismatches.append("insufficient odometry after sensor update")
            elif args.scenario == "nan-sensor":
                if update["nonfinite_count"] or after["nonfinite_count"]:
                    candidate_signatures.append("nan_sensor_nonfinite_odometry")
                    mismatches.append("nonfinite odometry after NaN sensor input")
                elif not near(after["last_x"], 2.0):
                    candidate_signatures.append("nan_sensor_state_changed")
                    mismatches.append("state changed after NaN sensor input")
            elif args.scenario == "nan-sensor-covariance":
                if update["nonfinite_count"] or after["nonfinite_count"]:
                    candidate_signatures.append("nan_sensor_covariance_nonfinite_odometry")
                    mismatches.append("nonfinite odometry after NaN sensor covariance")
            elif args.scenario == "negative-sensor-covariance":
                if (update["nonfinite_count"] or after["nonfinite_count"]
                        or update["negative_cov_count"] or after["negative_cov_count"]):
                    candidate_signatures.append("negative_sensor_covariance_output")
                    mismatches.append("negative sensor covariance reached invalid odometry")
                elif not near(after["last_x"], 5.0):
                    mismatches.append("negative sensor covariance did not reach x=5")
            elif (update["nonfinite_count"] or after["nonfinite_count"]
                  or (args.scenario == "valid-sensor-covariance" and
                      (update["negative_cov_count"] or after["negative_cov_count"]))
                  or not near(after["last_x"], 5.0)):
                mismatches.append("finite sensor control did not reach x=5")
            expected_recovery = 3.0 if args.scenario in (
                "nan-sensor-covariance", "valid-sensor-covariance",
                "negative-sensor-covariance") else 2.0
            if (recovered["output_count"] < 3 or recovered["nonfinite_count"]
                    or (args.scenario in ("negative-sensor-covariance",
                                          "valid-sensor-covariance") and recovered["negative_cov_count"])
                    or not near(recovered["last_x"], expected_recovery)):
                mismatches.append("filter did not recover after sensor update")
        elif frame_case:
            after = phase_by_name["after_set_pose"]
            expected_x = 2.0 if args.scenario == "invalid-set-pose" else 5.0
            if args.scenario in ("future-invalid-frame", "future-valid-frame",
                                 "nan-covariance", "valid-covariance", "negative-covariance",
                                 "current-invalid-frame"):
                if after["output_count"] < 3:
                    mismatches.append("insufficient odometry after set_pose")
                elif after["nonfinite_count"]:
                    if args.scenario == "nan-covariance":
                        candidate_signatures.append("nan_covariance_nonfinite_odometry")
                    elif args.scenario == "negative-covariance":
                        candidate_signatures.append("negative_set_pose_nonfinite_odometry")
                    mismatches.append("nonfinite odometry after set_pose")
                elif args.scenario == "negative-covariance":
                    if after["negative_cov_count"]:
                        candidate_signatures.append("negative_set_pose_covariance_output")
                        mismatches.append("negative covariance published after set_pose")
                    elif not (near(after["last_x"], 2.0) or near(after["last_x"], 5.0)):
                        mismatches.append("negative set_pose result unexplained")
                elif args.scenario in ("future-invalid-frame", "current-invalid-frame") and not near(after["last_x"], 2.0):
                    candidate_signatures.append("invalid_frame_state_changed")
                    mismatches.append("invalid-frame request changed state")
                elif args.scenario in ("future-valid-frame", "valid-covariance") and (
                        (args.scenario == "valid-covariance" and after["negative_cov_count"])
                        or not near(after["last_x"], 5.0)):
                    mismatches.append("valid set_pose control did not reach x=5")
                if (recovered["output_count"] < 3 or recovered["nonfinite_count"]
                        or (args.scenario in ("negative-covariance", "valid-covariance")
                            and recovered["negative_cov_count"])
                        or not near(recovered["last_x"], 3.0)):
                    if args.scenario == "future-invalid-frame" and recovered["output_count"] >= 3:
                        candidate_signatures.append("future_invalid_frame_sensor_blocked")
                    mismatches.append("valid sensor did not reach x=3 after set_pose")
            elif args.scenario == "nan-set-pose":
                if after["output_count"] < 3:
                    mismatches.append("insufficient odometry after NaN set_pose")
                elif after["nonfinite_count"]:
                    candidate_signatures.append("nan_set_pose_nonfinite_odometry")
                    mismatches.append("nonfinite odometry after NaN set_pose")
                elif not near(after["last_x"], 2.0):
                    candidate_signatures.append("nan_set_pose_state_changed")
                    mismatches.append("state changed after NaN set_pose")
            elif (after["output_count"] < 3 or after["nonfinite_count"]
                  or not near(after["last_x"], expected_x)):
                mismatches.append("set_pose result disagrees with frame-validity oracle")
            if args.scenario not in ("future-invalid-frame", "future-valid-frame",
                                        "nan-covariance", "valid-covariance",
                                        "negative-covariance", "current-invalid-frame"):
                if (recovered["output_count"] < 3 or recovered["nonfinite_count"]
                        or not near(recovered["last_x"], 2.0)):
                    mismatches.append("EKF did not recover after set_pose")
        else:
            off = phase_by_name["off"]
            if off["output_count"] < 3 or abs(off["last_x"]) > 0.5:
                mismatches.append("off-window EKF output missing or followed new pose")
            if recovered["output_count"] < 3 or recovered["last_x"] <= 1.0:
                mismatches.append("EKF did not follow new pose after resume")
        if args.scenario == "off-discard":
            quiet = phase_by_name["post_resume_no_input"]
            if quiet["output_count"] < 3 or abs(quiet["last_x"]) > 0.5:
                mismatches.append("off-window pose affected output after resume without new input")
        if args.scenario == "set-pose-off":
            for name in ("off_after_set_pose", "post_resume_no_input"):
                phase = phase_by_name[name]
                if phase["output_count"] < 3 or abs(phase["last_x"] - 5.0) > 0.5:
                    mismatches.append(f"set_pose state not visible in {name}")
        node.destroy_subscription(subscription)
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
        emit("execution_error", error=errors[-1])
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
    status = "execution_error" if errors else "uncertain" if mismatches else "pass"
    summary = {"schema": 1, "experiment": "p3-localization-toggle", "filter": args.filter,
               "scenario": args.scenario,
               "status": status, "domain": args.ros_domain_id, "phases": phases,
               "calls": calls, "errors": errors, "mismatches": mismatches,
               "candidate_signatures": candidate_signatures,
               "process_returncode": process.returncode if process else None,
               "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest()}
    emit("run_end", status=status, errors=errors, mismatches=mismatches)
    trace.close()
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"status": status, "phases": phases,
                      "errors": errors, "mismatches": mismatches,
                      "candidate_signatures": candidate_signatures}))
    raise SystemExit({"pass": 0, "uncertain": 3, "execution_error": 2}[status])


if __name__ == "__main__":
    main()
