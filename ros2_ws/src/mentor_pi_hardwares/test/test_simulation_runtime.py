#!/usr/bin/env python3

import os
import signal
import subprocess
import tempfile
import time

from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import (
    MecanumDriveControllerState,
    SteeringControllerStatus,
)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SimulationClient:
    def __init__(self, robot_name, vehicle_type):
        self.robot_name = robot_name
        self.vehicle_type = vehicle_type
        self.node = rclpy.create_node(f"{robot_name}_command_runtime_probe")
        self.latest_odometry = None
        self.latest_controller_state = None
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
        controller_state_type = (
            SteeringControllerStatus
            if vehicle_type == "ackermann"
            else MecanumDriveControllerState
        )
        self.controller_state_subscription = self.node.create_subscription(
            controller_state_type,
            f"/{robot_name}/vehicle/controller_state",
            self._accept_controller_state,
            10,
        )
        self.controllers = self.node.create_client(
            ListControllers,
            f"/{robot_name}/controller_manager/list_controllers",
        )
        self.controller_parameters = self.node.create_client(
            GetParameters,
            f"/{robot_name}/controller_manager/get_parameters",
        )

    def destroy(self):
        self.node.destroy_node()

    def _accept_odometry(self, message):
        self.latest_odometry = message

    def _accept_controller_state(self, message):
        self.latest_controller_state = message

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

    def controller_update_rate(self):
        if not self.controller_parameters.wait_for_service(timeout_sec=1.0):
            return None
        request = GetParameters.Request()
        request.names = ["update_rate"]
        future = self.controller_parameters.call_async(request)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done() or not future.result().values:
            return None
        return future.result().values[0].integer_value

    def controller_command_values(self):
        if self.latest_controller_state is None:
            return ()
        if self.vehicle_type == "mecanum":
            return (
                self.latest_controller_state.reference_velocity.linear.x,
            )
        return tuple(self.latest_controller_state.linear_velocity_command)

    def controller_state_time_seconds(self):
        if self.latest_controller_state is None:
            return None
        stamp = self.latest_controller_state.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def publish_reference(
        self, linear_x, linear_y, angular_z, stamp_mode
    ):
        message = TwistStamped()
        sender_stamp_seconds = None
        if stamp_mode == "now":
            sender_time = self.node.get_clock().now()
            message.header.stamp = sender_time.to_msg()
            sender_stamp_seconds = sender_time.nanoseconds / 1_000_000_000.0
        elif stamp_mode == "stale":
            message.header.stamp.sec = 1
        elif stamp_mode != "zero":
            raise ValueError(f"unsupported stamp mode: {stamp_mode}")
        message.twist.linear.x = linear_x
        message.twist.linear.y = linear_y
        message.twist.angular.z = angular_z
        self.publisher.publish(message)
        return time.monotonic(), sender_stamp_seconds

    def publish_sequence_for(
        self, references, stamp_modes, duration, frequency_hz
    ):
        assert frequency_hz >= 50.0
        assert references
        assert len(references) == len(stamp_modes)
        deadline = time.monotonic() + duration
        period = 1.0 / frequency_hz
        next_publish = time.monotonic()
        publication_count = 0
        last_publication_time = None
        last_sender_stamp_seconds = None
        while time.monotonic() < deadline:
            sequence_index = publication_count % len(references)
            last_publication_time, last_sender_stamp_seconds = (
                self.publish_reference(
                    *references[sequence_index],
                    stamp_mode=stamp_modes[sequence_index],
                )
            )
            publication_count += 1
            rclpy.spin_once(self.node, timeout_sec=0.0)
            next_publish += period
            time.sleep(max(0.0, next_publish - time.monotonic()))
        return (
            publication_count,
            last_publication_time,
            last_sender_stamp_seconds,
        )


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
def test_simulation_uses_receipt_time_latest_reference_and_timeout(
    vehicle_type,
):
    robot_name = f"{vehicle_type}_command_runtime"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_simulation(vehicle_type, robot_name, log)
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name, vehicle_type)
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
            update_rate = client.controller_update_rate()
            assert update_rate is not None and update_rate >= 50
            nodes = set(client.node.get_node_names_and_namespaces())
            assert ("vehicle", f"/{robot_name}") in nodes
            assert ("trajectory_tracker", f"/{robot_name}") not in nodes
            assert ("vehicle_pose", f"/{robot_name}") not in nodes
            topics = {
                name for name, _ in client.node.get_topic_names_and_types()
            }
            assert f"/{robot_name}/vehicle/reference" in topics
            assert f"/{robot_name}/vehicle/controller_state" in topics
            assert f"/{robot_name}/vehicle/_controller_odometry" in topics
            assert f"/{robot_name}/vehicle/odometry" not in topics
            assert f"/{robot_name}/vehicle/pose" not in topics

            lateral_speed = 0.08 if vehicle_type == "mecanum" else 0.0
            forward_reference = (0.12, lateral_speed, 0.2)
            reverse_reference = (-0.12, -lateral_speed, -0.2)

            initial_x = client.latest_odometry.pose.pose.position.x
            forward_publications, _, _ = client.publish_sequence_for(
                (forward_reference,),
                ("zero",),
                duration=0.6,
                frequency_hz=50.0,
            )
            assert forward_publications >= 30
            assert client.spin_until(
                lambda: client.latest_odometry.pose.pose.position.x
                > initial_x + 0.02,
                timeout=2.0,
            )
            reverse_publications, _, _ = client.publish_sequence_for(
                (reverse_reference,),
                ("stale",),
                duration=0.6,
                frequency_hz=60.0,
            )
            assert reverse_publications >= 36
            assert client.spin_until(
                lambda: (
                    client.controller_command_values()
                    and all(
                        value < -0.01
                        for value in client.controller_command_values()
                    )
                ),
                timeout=0.12,
            )
            assert client.spin_until(
                lambda: client.latest_odometry.twist.twist.linear.x < -0.01,
                timeout=0.3,
            )

            alternating_publications, _, _ = client.publish_sequence_for(
                (forward_reference, reverse_reference),
                ("zero", "stale"),
                duration=0.3,
                frequency_hz=60.0,
            )
            assert alternating_publications >= 18

            # Establish an observable negative precondition, then issue two
            # immediate replacements whose newest value is positive. If the
            # pair is dropped or a queued/prior value is retained, the state
            # remains negative and this assertion fails.
            precondition_state_time = client.controller_state_time_seconds()
            client.publish_reference(*reverse_reference, stamp_mode="stale")
            assert client.spin_until(
                lambda: (
                    client.controller_state_time_seconds() is not None
                    and client.controller_state_time_seconds()
                    > precondition_state_time
                    and client.controller_command_values()
                    and all(
                        value < -0.01
                        for value in client.controller_command_values()
                    )
                ),
                timeout=0.1,
            )
            client.publish_reference(*reverse_reference, stamp_mode="stale")
            _, last_sender_stamp_seconds = client.publish_reference(
                *forward_reference, stamp_mode="now"
            )
            assert last_sender_stamp_seconds is not None
            assert client.spin_until(
                lambda: (
                    client.controller_state_time_seconds() is not None
                    and client.controller_state_time_seconds()
                    >= last_sender_stamp_seconds
                    and client.controller_command_values()
                    and all(
                        value > 0.01
                        for value in client.controller_command_values()
                    )
                ),
                timeout=0.1,
            )
            accepted_state_time = client.controller_state_time_seconds()

            # The 100 ms receipt-time deadline is applied on the first 50 Hz
            # controller update after expiry. Controller-state timestamps avoid
            # Ackermann's ten-sample odometry-twist filter and plant lag.
            assert client.spin_until(
                lambda: (
                    client.controller_state_time_seconds() is not None
                    and client.controller_state_time_seconds()
                    > accepted_state_time
                    and client.controller_command_values()
                    and all(
                        abs(value) < 1.0e-6
                        for value in client.controller_command_values()
                    )
                ),
                timeout=0.16,
            )
            timeout_age = (
                client.controller_state_time_seconds()
                - last_sender_stamp_seconds
            )
            assert 0.1 <= timeout_age <= 0.121

            assert client.spin_until(
                lambda: abs(
                    client.latest_odometry.twist.twist.linear.x
                ) < 0.001,
                timeout=0.3,
            )
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
        assert "Received message has timestamp" not in output
        if process.returncode not in (0, -signal.SIGINT):
            pytest.fail(
                f"simulation launch exited {process.returncode}:\n{output}"
            )
