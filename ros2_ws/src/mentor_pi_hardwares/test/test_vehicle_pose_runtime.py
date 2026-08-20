#!/usr/bin/env python3

import copy
import math
import os
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "158"

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


ROBOT_NAME = "vehicle_pose_runtime"
SOURCE_TOPIC = f"/vrpn_mocap/{ROBOT_NAME}/pose"
OUTPUT_TOPIC = f"/{ROBOT_NAME}/vehicle/pose"
BASE_FRAME = f"{ROBOT_NAME}/base_footprint"


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


class PoseProbe:
    def __init__(self):
        self.node = rclpy.create_node("vehicle_pose_runtime_probe")
        self.poses = []
        self.transforms = []
        self.publisher = self.node.create_publisher(
            PoseStamped, SOURCE_TOPIC, qos_profile_sensor_data
        )
        self.pose_subscription = self.node.create_subscription(
            PoseStamped,
            OUTPUT_TOPIC,
            self.poses.append,
            qos_profile_sensor_data,
        )
        self.tf_subscription = self.node.create_subscription(
            TFMessage,
            "/tf",
            self._accept_transforms,
            qos_profile_sensor_data,
        )

    def destroy(self):
        self.node.destroy_node()

    def _accept_transforms(self, message):
        self.transforms.extend(
            transform
            for transform in message.transforms
            if transform.child_frame_id == BASE_FRAME
        )

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_for(self, message, duration=0.4):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)


def launch_vehicle_pose(log):
    return subprocess.Popen(
        [
            "ros2",
            "run",
            "mentor_pi_hardwares",
            "vehicle_pose",
            "--ros-args",
            "-r",
            f"__ns:=/{ROBOT_NAME}",
            "-r",
            (
                f"/{ROBOT_NAME}/vehicle/_mocap_pose:="
                f"{SOURCE_TOPIC}"
            ),
            "-p",
            "input_type:=mocap_pose",
            "-p",
            "source_to_geometry_center_m:=0.0",
            "-p",
            f"geometry_center_frame_id:={BASE_FRAME}",
            "-p",
            "output_frame_id:=map",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )


def test_mocap_pose_runtime_contract():
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_vehicle_pose(log)
        if not rclpy.ok():
            rclpy.init()
        probe = PoseProbe()
        try:
            assert probe.spin_until(
                lambda: process.poll() is not None
                or probe.publisher.get_subscription_count() == 1
            )
            assert process.poll() is None

            source = PoseStamped()
            source.header.frame_id = "map"
            source.header.stamp = probe.node.get_clock().now().to_msg()
            source.pose.position.x = 1.25
            source.pose.position.y = -0.75
            source.pose.position.z = 0.10
            source.pose.orientation.z = 1.0
            source.pose.orientation.w = 1.0
            probe.publish_for(source)

            assert probe.spin_until(
                lambda: bool(probe.poses) and bool(probe.transforms)
            )
            output = probe.poses[-1]
            transform = probe.transforms[-1]
            assert output.header == source.header
            assert output.pose.position == source.pose.position
            assert output.pose.orientation.z == pytest.approx(
                math.sqrt(0.5)
            )
            assert output.pose.orientation.w == pytest.approx(
                math.sqrt(0.5)
            )
            assert transform.header == output.header
            assert transform.child_frame_id == BASE_FRAME
            assert transform.transform.translation.x == pytest.approx(1.25)
            assert transform.transform.translation.y == pytest.approx(-0.75)
            assert transform.transform.translation.z == pytest.approx(0.10)

            probe.spin_until(lambda: False, timeout=0.2)
            accepted_count = len(probe.poses)
            transform_count = len(probe.transforms)

            wrong_frame = copy.deepcopy(source)
            wrong_frame.header.frame_id = "odom"
            probe.publish_for(wrong_frame)
            assert len(probe.poses) == accepted_count
            assert len(probe.transforms) == transform_count

            nonfinite = copy.deepcopy(source)
            nonfinite.pose.position.x = math.nan
            probe.publish_for(nonfinite)
            assert len(probe.poses) == accepted_count
            assert len(probe.transforms) == transform_count

            topics = {
                name for name, _ in probe.node.get_topic_names_and_types()
            }
            assert SOURCE_TOPIC in topics
            assert OUTPUT_TOPIC in topics
            assert f"/{ROBOT_NAME}/vehicle/odometry" not in topics
            assert f"/{ROBOT_NAME}/vehicle/tf_odometry" not in topics
            assert all(
                item.header.frame_id == "map"
                and item.child_frame_id == BASE_FRAME
                for item in probe.transforms
            )
        finally:
            stop_process(process)
            probe.destroy()
            if rclpy.ok():
                rclpy.shutdown()
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                pytest.fail(
                    f"vehicle_pose exited {process.returncode}:\n{log.read()}"
                )
