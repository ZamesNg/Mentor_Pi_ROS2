import os
import re
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


_ROBOT_NAME_PATTERN = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_VEHICLE_TYPES = frozenset(("mecanum", "ackermann"))
_VEHICLE_CONTROLLER_NAME = "vehicle"


def _shutdown_on_exit(action, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=action,
            on_exit=[EmitEvent(event=Shutdown(reason=f"{label} exited"))],
        )
    )


def _shutdown_if_failed(event, context, label):
    del context
    if event.returncode == 0:
        return []
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"{label} failed with exit code {event.returncode}"
            )
        )
    ]


def _shutdown_on_failure(action, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=action,
            on_exit=partial(_shutdown_if_failed, label=label),
        )
    )


def _validate_robot_name(robot_name):
    return (
        isinstance(robot_name, str)
        and _ROBOT_NAME_PATTERN.fullmatch(robot_name) is not None
    )


def controller_state_estimate_remappings(robot_name):
    if not _validate_robot_name(robot_name):
        raise ValueError("robot_name must be a valid relative ROS namespace")
    controller_prefix = f"/{robot_name}/vehicle"
    return [
        (
            f"{controller_prefix}/odometry",
            f"{controller_prefix}/_controller_odometry",
        ),
        (
            f"{controller_prefix}/tf_odometry",
            f"{controller_prefix}/_controller_tf_odometry",
        ),
    ]


def _launch_simulation(context):
    vehicle_type = LaunchConfiguration("vehicle_type").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError("vehicle_type must be ackermann or mecanum")
    if not _validate_robot_name(robot_name):
        raise ValueError("robot_name must be a valid relative ROS namespace")
    hardware_share = get_package_share_directory("mentor_pi_hardwares")
    description_file = os.path.join(
        hardware_share, "config", vehicle_type, "simulation.urdf.xacro"
    )
    controllers_file = os.path.join(
        hardware_share, "config", vehicle_type, "controllers.yaml"
    )
    robot_description = {
        "robot_description": Command(
            ["xacro ", description_file, " robot_name:=", robot_name]
        )
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[robot_description],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_name,
        output="screen",
        parameters=[controllers_file, robot_description],
        remappings=controller_state_estimate_remappings(robot_name),
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        namespace=robot_name,
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            f"/{robot_name}/controller_manager",
        ],
    )
    vehicle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_vehicle",
        namespace=robot_name,
        output="screen",
        arguments=[
            _VEHICLE_CONTROLLER_NAME,
            "--controller-manager",
            f"/{robot_name}/controller_manager",
        ],
    )
    delayed_actions = [
        _shutdown_on_failure(
            joint_state_spawner, "joint state broadcaster spawner"
        ),
        joint_state_spawner,
        _shutdown_on_failure(vehicle_spawner, "vehicle spawner"),
        vehicle_spawner,
    ]
    return [
        _shutdown_on_exit(robot_state_publisher, "robot state publisher"),
        robot_state_publisher,
        _shutdown_on_exit(controller_manager, "controller manager"),
        controller_manager,
        TimerAction(period=2.0, actions=delayed_actions),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_type"),
            DeclareLaunchArgument("robot_name", default_value="mentor_pi_sim"),
            OpaqueFunction(function=_launch_simulation),
        ]
    )
