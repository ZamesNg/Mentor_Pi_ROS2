#!/usr/bin/env python3

import math
import os
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped
from mentor_pi_tracking_interfaces.msg import (
    PolynomialSegment,
    PolynomialTrajectory,
)
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        orientation.w * orientation.w
        + orientation.x * orientation.x
        - orientation.y * orientation.y
        - orientation.z * orientation.z,
    )


class SimulationClient:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.node = rclpy.create_node(f"{robot_name}_runtime_probe")
        self.latest_pose = None
        self.robot_description = None
        self.transforms = {}
        self.publisher = self.node.create_publisher(
            TwistStamped, f"/{robot_name}/vehicle/reference", 10
        )
        self.subscription = self.node.create_subscription(
            PoseStamped,
            f"/{robot_name}/vehicle/pose",
            self._accept_pose,
            qos_profile_sensor_data,
        )
        transient_local_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.description_subscription = self.node.create_subscription(
            String,
            f"/{robot_name}/robot_description",
            self._accept_robot_description,
            transient_local_qos,
        )
        self.tf_subscription = self.node.create_subscription(
            TFMessage,
            "/tf",
            self._accept_transforms,
            qos_profile_sensor_data,
        )
        self.tf_static_subscription = self.node.create_subscription(
            TFMessage,
            "/tf_static",
            self._accept_transforms,
            transient_local_qos,
        )
        self.controllers = self.node.create_client(
            ListControllers, f"/{robot_name}/controller_manager/list_controllers"
        )

    def destroy(self):
        self.node.destroy_node()

    def _accept_pose(self, message):
        self.latest_pose = message

    def _accept_robot_description(self, message):
        self.robot_description = message.data

    def _accept_transforms(self, message):
        for transform in message.transforms:
            self.transforms[transform.child_frame_id] = transform

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

    def publish_for(self, linear_x, linear_y, angular_z, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = linear_x
            message.twist.linear.y = linear_y
            message.twist.angular.z = angular_z
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.03)


def launch_simulation(
    vehicle_type,
    robot_name,
    log,
    *,
    initial_x_m=1.25,
    initial_y_m=-0.75,
    initial_yaw_rad=0.0,
    tracking_controller=None,
    tracking_algorithm="adrc",
):
    command = [
        "ros2",
        "launch",
        "mentor_pi_hardwares",
        "simulation.launch.py",
        f"vehicle_type:={vehicle_type}",
        f"robot_name:={robot_name}",
        f"initial_x_m:={initial_x_m}",
        f"initial_y_m:={initial_y_m}",
        f"initial_yaw_rad:={initial_yaw_rad}",
        f"tracking_algorithm:={tracking_algorithm}",
    ]
    if tracking_controller is not None:
        command.append(f"tracking_controller:={tracking_controller}")
    return subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )


def launch_foxglove(port, log):
    return subprocess.Popen(
        [
            "ros2",
            "launch",
            "mentor_pi_hardwares",
            "foxglove.launch.py",
            "address:=127.0.0.1",
            f"port:={port}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )


def unused_loopback_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def port_is_open(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


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
def test_simulation_controller_and_map_pose_runtime(vehicle_type):
    robot_name = f"{vehicle_type}_runtime_test"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_simulation(
            vehicle_type, robot_name, log, tracking_controller="none"
        )
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name)
        try:
            assert client.spin_until(
                lambda: process.poll() is not None
                or (
                    client.latest_pose is not None
                    and client.controller_is_active()
                    and client.robot_description is not None
                    and f"{robot_name}/wheel_left_rear_link"
                    in client.transforms
                ),
                timeout=15.0,
            )
            assert process.poll() is None
            initial = client.latest_pose
            assert initial.header.frame_id == "map"
            assert initial.pose.position.x == pytest.approx(1.25, abs=0.01)
            assert initial.pose.position.y == pytest.approx(-0.75, abs=0.01)
            assert yaw(initial.pose.orientation) == pytest.approx(
                0.0, abs=0.01
            )

            nodes = {
                (name, namespace)
                for name, namespace in client.node.get_node_names_and_namespaces()
            }
            assert ("vehicle", f"/{robot_name}") in nodes
            assert ("vehicle_hardware", f"/{robot_name}") in nodes
            assert ("vehicle_pose", f"/{robot_name}") in nodes
            assert ("_vehicle_odometry", f"/{robot_name}") not in nodes
            assert ("trajectory_tracker", f"/{robot_name}") not in nodes
            assert ("configuration_supervisor", f"/{robot_name}") not in nodes
            topics = {
                name for name, _ in client.node.get_topic_names_and_types()
            }
            assert f"/{robot_name}/vehicle/reference" in topics
            assert f"/{robot_name}/vehicle/pose" in topics
            assert f"/{robot_name}/vehicle/odometry" not in topics
            assert f"/{robot_name}/vehicle/tf_odometry" not in topics
            assert f"/{robot_name}/robot_description" in topics
            assert "/tf" in topics
            assert "/tf_static" in topics
            assert f"/{robot_name}/heartbeat" not in topics
            assert f"/{robot_name}/motors/state" not in topics
            assert client.transforms[f"{robot_name}/base_footprint"].header.frame_id == "map"

            description = ET.fromstring(client.robot_description)
            link_names = {
                link.attrib["name"] for link in description.findall("./link")
            }
            assert link_names
            assert all(name.startswith(f"{robot_name}/") for name in link_names)
            assert all(
                not name.startswith(f"{robot_name}/{robot_name}/")
                for name in link_names
            )
            assert f"{robot_name}/base_footprint" in link_names
            expected_tf_children = link_names - {
                f"{robot_name}/base_footprint"
            }
            assert client.spin_until(
                lambda: expected_tf_children <= client.transforms.keys(),
                timeout=2.0,
            )
            for child_frame in expected_tf_children:
                assert client.transforms[child_frame].header.frame_id in link_names
            if vehicle_type == "ackermann":
                rear_axle = client.transforms[
                    f"{robot_name}/rear_axle_footprint"
                ]
                assert rear_axle.header.frame_id == (
                    f"{robot_name}/base_footprint"
                )
                assert rear_axle.transform.translation.x == pytest.approx(
                    -0.0675
                )
            wheel_frame = f"{robot_name}/wheel_left_rear_link"
            initial_wheel_rotation = client.transforms[
                wheel_frame
            ].transform.rotation
            initial_wheel_quaternion = (
                initial_wheel_rotation.x,
                initial_wheel_rotation.y,
                initial_wheel_rotation.z,
                initial_wheel_rotation.w,
            )

            if vehicle_type == "ackermann":
                client.publish_for(0.12, 0.0, 0.25, 1.0)
            else:
                client.publish_for(0.12, 0.08, 0.25, 1.0)
            positive = client.latest_pose
            positive_wheel_rotation = client.transforms[
                wheel_frame
            ].transform.rotation
            positive_wheel_quaternion = (
                positive_wheel_rotation.x,
                positive_wheel_rotation.y,
                positive_wheel_rotation.z,
                positive_wheel_rotation.w,
            )
            assert positive_wheel_quaternion != pytest.approx(
                initial_wheel_quaternion, abs=1.0e-3
            )
            assert positive.pose.position.x > initial.pose.position.x + 0.03
            assert yaw(positive.pose.orientation) > 0.03
            if vehicle_type == "mecanum":
                assert positive.pose.position.y > (
                    initial.pose.position.y + 0.02
                )

            if vehicle_type == "ackermann":
                client.publish_for(-0.12, 0.0, 0.0, 1.0)
                reverse = client.latest_pose
                assert reverse.pose.position.x < (
                    positive.pose.position.x - 0.02
                )
            else:
                client.publish_for(-0.12, -0.08, -0.25, 1.0)
                reverse = client.latest_pose
                assert reverse.pose.position.x < positive.pose.position.x
                assert reverse.pose.position.y < positive.pose.position.y
                assert yaw(reverse.pose.orientation) < (
                    yaw(positive.pose.orientation) - 0.03
                )

            assert client.spin_until(lambda: client.latest_pose is not None, timeout=1.5)
            stopped = client.latest_pose.pose.position
            time.sleep(0.3)
            rclpy.spin_once(client.node, timeout_sec=0.05)
            assert client.latest_pose.pose.position.x == pytest.approx(
                stopped.x, abs=0.01
            )
            assert client.latest_pose.pose.position.y == pytest.approx(
                stopped.y, abs=0.01
            )
        finally:
            stop_process(process)
            client.destroy()
            if rclpy.ok():
                rclpy.shutdown()
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                pytest.fail(
                    f"simulation launch exited {process.returncode}:\n{log.read()}"
                )


def test_foxglove_bridge_is_separate_and_discovers_simulation_graph():
    robot_name = "foxglove_runtime_test"
    port = unused_loopback_port()
    with (
        tempfile.TemporaryFile(mode="w+") as simulation_log,
        tempfile.TemporaryFile(mode="w+") as bridge_log,
    ):
        simulation = launch_simulation(
            "mecanum",
            robot_name,
            simulation_log,
            tracking_controller="none",
        )
        bridge = None
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name)
        try:
            assert client.spin_until(
                lambda: simulation.poll() is not None
                or (
                    client.latest_pose is not None
                    and client.controller_is_active()
                ),
                timeout=15.0,
            )
            assert simulation.poll() is None
            bridge = launch_foxglove(port, bridge_log)
            assert client.spin_until(
                lambda: bridge.poll() is not None or port_is_open(port),
                timeout=10.0,
            )
            assert bridge.poll() is None
            assert port_is_open(port)
            assert client.spin_until(
                lambda: ("foxglove_bridge", "/")
                in set(client.node.get_node_names_and_namespaces()),
                timeout=5.0,
            )
            topics = {
                name for name, _ in client.node.get_topic_names_and_types()
            }
            assert f"/{robot_name}/robot_description" in topics
            assert "/tf" in topics
            assert "/tf_static" in topics
        finally:
            if bridge is not None:
                stop_process(bridge)
            stop_process(simulation)
            client.destroy()
            if rclpy.ok():
                rclpy.shutdown()
            for process, log, label in (
                (simulation, simulation_log, "simulation"),
                (bridge, bridge_log, "Foxglove bridge"),
            ):
                if process is not None and process.returncode not in (
                    0,
                    -signal.SIGINT,
                ):
                    log.seek(0)
                    pytest.fail(
                        f"{label} exited {process.returncode}:\n{log.read()}"
                    )


@pytest.mark.parametrize(
    "vehicle_type,tracking_algorithm",
    [
        ("ackermann", "adrc"),
        ("ackermann", "mpc"),
        ("mecanum", "adrc"),
        ("mecanum", "mpc"),
    ],
)
def test_default_tracker_drives_simulation_without_low_level_topics(
    vehicle_type, tracking_algorithm
):
    robot_name = f"{vehicle_type}_{tracking_algorithm}_tracker_test"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_simulation(
            vehicle_type,
            robot_name,
            log,
            initial_x_m=0.0,
            initial_y_m=0.0,
            tracking_algorithm=tracking_algorithm,
        )
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name)
        trajectory_publisher = client.node.create_publisher(
            PolynomialTrajectory,
            f"/{robot_name}/trajectory_tracker/reference_trajectory",
            10,
        )
        try:
            assert client.spin_until(
                lambda: process.poll() is not None
                or (
                    client.latest_pose is not None
                    and client.controller_is_active()
                    and trajectory_publisher.get_subscription_count() == 1
                ),
                timeout=15.0,
            )
            assert process.poll() is None
            nodes = set(client.node.get_node_names_and_namespaces())
            assert ("trajectory_tracker", f"/{robot_name}") in nodes
            topics = dict(client.node.get_topic_names_and_types())
            assert topics[
                f"/{robot_name}/trajectory_tracker/diagnostics"
            ] == ["diagnostic_msgs/msg/DiagnosticArray"]
            for absent in (
                f"/{robot_name}/motors/state",
                f"/{robot_name}/heartbeat",
                f"/{robot_name}/configuration/motion_authorization",
                f"/{robot_name}/diagnostics",
            ):
                assert absent not in topics

            trajectory = PolynomialTrajectory()
            trajectory.header.frame_id = "map"
            trajectory.header.stamp = (
                client.node.get_clock().now()
                + Duration(seconds=0.5)
            ).to_msg()
            trajectory.trajectory_id = (
                f"{vehicle_type}-{tracking_algorithm}-runtime"
            )
            segment = PolynomialSegment()
            segment.duration.sec = 2
            segment.x_coefficients[1] = 0.10
            if vehicle_type == "mecanum":
                segment.y_coefficients[1] = 0.05
                segment.yaw_coefficients[1] = 0.20
            trajectory.segments.append(segment)
            initial = client.latest_pose
            trajectory_publisher.publish(trajectory)

            assert client.spin_until(
                lambda: client.latest_pose is not None
                and client.latest_pose.pose.position.x
                > initial.pose.position.x + 0.01,
                timeout=4.0,
            )
            moved = client.latest_pose
            if vehicle_type == "mecanum":
                assert moved.pose.position.y > 0.002
                assert yaw(moved.pose.orientation) > 0.005

            assert client.spin_until(lambda: client.latest_pose is not None, timeout=3.5)
        finally:
            stop_process(process)
            client.destroy()
            if rclpy.ok():
                rclpy.shutdown()
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                pytest.fail(
                    f"simulation launch exited {process.returncode}:\n{log.read()}"
                )


def test_tracker_continues_correcting_after_execution_horizon():
    robot_name = "mecanum_terminal_hold_test"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = launch_simulation(
            "mecanum",
            robot_name,
            log,
            initial_x_m=0.0,
            initial_y_m=0.0,
            tracking_algorithm="adrc",
        )
        if not rclpy.ok():
            rclpy.init()
        client = SimulationClient(robot_name)
        trajectory_publisher = client.node.create_publisher(
            PolynomialTrajectory,
            f"/{robot_name}/trajectory_tracker/reference_trajectory",
            10,
        )
        try:
            assert client.spin_until(
                lambda: process.poll() is not None
                or (
                    client.latest_pose is not None
                    and client.controller_is_active()
                    and trajectory_publisher.get_subscription_count() == 1
                ),
                timeout=15.0,
            )
            assert process.poll() is None

            trajectory = PolynomialTrajectory()
            trajectory.header.frame_id = "map"
            trajectory.header.stamp = (
                client.node.get_clock().now() + Duration(seconds=0.5)
            ).to_msg()
            trajectory.trajectory_id = "terminal-hold-runtime"
            segment = PolynomialSegment()
            segment.duration.nanosec = 200_000_000
            segment.x_coefficients[0] = 0.15
            trajectory.segments.append(segment)
            trajectory_publisher.publish(trajectory)

            assert client.spin_until(
                lambda: client.latest_pose is not None
                and client.latest_pose.pose.position.x > 0.08,
                timeout=4.0,
            )
            assert client.spin_until(
                lambda: client.latest_pose is not None
                and abs(client.latest_pose.pose.position.x - 0.15) < 0.04,
                timeout=4.0,
            )
            held_x = client.latest_pose.pose.position.x
            hold_deadline = time.monotonic() + 0.5
            while time.monotonic() < hold_deadline:
                rclpy.spin_once(client.node, timeout_sec=0.02)
            assert (
                abs(client.latest_pose.pose.position.x - held_x)
                < 0.02
            )
        finally:
            stop_process(process)
            client.destroy()
            if rclpy.ok():
                rclpy.shutdown()
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                pytest.fail(
                    f"simulation launch exited {process.returncode}:\n{log.read()}"
                )
