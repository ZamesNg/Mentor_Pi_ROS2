#!/usr/bin/env python3

import os
import signal
import subprocess
import tempfile
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SimulationClient:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.node = rclpy.create_node(f"{robot_name}_command_runtime_probe")
        self.latest_odometry = None
        command_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.publisher = self.node.create_publisher(
            TwistStamped, f"/{robot_name}/vehicle/reference", command_qos
        )
        self.subscription = self.node.create_subscription(
            Odometry,
            f"/{robot_name}/vehicle/_controller_odometry",
            self._accept_odometry,
            10,
        )
        self.controllers = self.node.create_client(
            ListControllers,
            f"/{robot_name}/controller_manager/list_controllers",
        )

    def destroy(self):
        self.node.destroy_node()

    def _accept_odometry(self, message):
        self.latest_odometry = message

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def controller_is_active(self):
        if not self.controllers.service_is_ready():
            return False
        future = self.controllers.call_async(ListControllers.Request())
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.done() and any(
            controller.name == "vehicle" and controller.state == "active"
            for controller in future.result().controller
        )

    def publish_zero_stamped_for(
        self, linear_x, linear_y, angular_z, duration
    ):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.twist.linear.x = linear_x
            message.twist.linear.y = linear_y
            message.twist.angular.z = angular_z
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.03)


def launch_simulation(vehicle_type, robot_name, log):
    return subprocess.Popen(
        [
            "ros2",
            "launch",
            "mentor_pi_hardwares",
            "simulation.launch.py",
            f"vehicle_type:={vehicle_type}",
            f"robot_name:={robot_name}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


@pytest.mark.parametrize("vehicle_type", ["ackermann", "mecanum"])
def test_command_only_simulation_accepts_zero_stamp_and_times_out(vehicle_type):
    robot_name = f"{vehicle_type}_command_runtime"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_simulation(vehicle_type, robot_name, log)
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name)
        try:
            assert client.spin_until(
                lambda: process.poll() is not None
                or (
                    client.latest_odometry is not None
                    and client.controller_is_active()
                    and client.publisher.get_subscription_count() == 1
                ),
                timeout=15.0,
            )
            assert process.poll() is None
            nodes = set(client.node.get_node_names_and_namespaces())
            assert ("vehicle", f"/{robot_name}") in nodes
            assert ("trajectory_tracker", f"/{robot_name}") not in nodes
            assert ("vehicle_pose", f"/{robot_name}") not in nodes
            topics = {
                name for name, _ in client.node.get_topic_names_and_types()
            }
            assert f"/{robot_name}/vehicle/reference" in topics
            assert f"/{robot_name}/vehicle/_controller_odometry" in topics
            assert f"/{robot_name}/vehicle/odometry" not in topics
            assert f"/{robot_name}/vehicle/pose" not in topics

            initial_x = client.latest_odometry.pose.pose.position.x
            client.publish_zero_stamped_for(
                0.12,
                0.08 if vehicle_type == "mecanum" else 0.0,
                0.2,
                0.8,
            )
            assert client.spin_until(
                lambda: client.latest_odometry.pose.pose.position.x
                > initial_x + 0.02,
                timeout=2.0,
            )
            time.sleep(0.35)
            rclpy.spin_once(client.node, timeout_sec=0.05)
            stopped_x = client.latest_odometry.pose.pose.position.x
            time.sleep(0.25)
            rclpy.spin_once(client.node, timeout_sec=0.05)
            assert client.latest_odometry.pose.pose.position.x == pytest.approx(
                stopped_x, abs=0.01
            )
        finally:
            stop_process(process)
            client.destroy()
            if rclpy.ok():
                rclpy.shutdown()
        log.seek(0)
        output = log.read()
        assert "Timestamp in header is missing" not in output
        if process.returncode not in (0, -signal.SIGINT):
            pytest.fail(
                f"simulation launch exited {process.returncode}:\n{output}"
            )
