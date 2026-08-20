import math
import os
import re
from functools import partial

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


_ROBOT_NAME_PATTERN = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_VEHICLE_TYPES = frozenset(("mecanum", "ackermann"))
_VEHICLE_CONTROLLER_NAME = "vehicle"


def _validate_native_runtime():
    if os.path.exists("/.dockerenv"):
        raise RuntimeError(
            "ROS hardware runtime must run on native Ubuntu 22.04, not the "
            "Dev Container"
        )


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


def load_vehicle_profile(path):
    if not isinstance(path, str) or not os.path.isabs(path):
        raise ValueError("vehicle_config must be an absolute path")
    resolved = os.path.realpath(path)
    if not os.path.isfile(resolved):
        raise ValueError(f"vehicle_config is not a readable file: {path}")
    try:
        with open(resolved, encoding="utf-8") as stream:
            profile = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"could not read vehicle_config: {error}") from error
    if not isinstance(profile, dict):
        raise ValueError("vehicle_config must contain a YAML mapping")

    vehicle = profile.get("vehicle")
    hardware = profile.get("hardware")
    if not isinstance(vehicle, dict) or set(vehicle) != {
        "robot_name",
        "vehicle_type",
    }:
        raise ValueError(
            "vehicle_config vehicle must contain only robot_name and vehicle_type"
        )
    if not isinstance(hardware, dict):
        raise ValueError("vehicle_config hardware must be a YAML mapping")

    robot_name = vehicle["robot_name"]
    vehicle_type = vehicle["vehicle_type"]
    if not _validate_robot_name(robot_name):
        raise ValueError("robot_name must be a valid relative ROS namespace")
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError(f"unsupported vehicle_type: {vehicle_type}")
    for name, value in hardware.items():
        if not isinstance(name, str) or re.fullmatch(r"[a-z][a-z0-9_]*", name) is None:
            raise ValueError(f"invalid hardware setting name: {name}")
        if isinstance(value, bool):
            continue
        if isinstance(value, int):
            continue
        if not isinstance(value, float) or not math.isfinite(value):
            raise ValueError(
                f"hardware setting must be finite numeric or boolean: {name}"
            )

    return {
        "path": resolved,
        "robot_name": robot_name,
        "vehicle_type": vehicle_type,
        "hardware": hardware,
    }


def _launch_vehicle(context):
    _validate_native_runtime()
    profile = load_vehicle_profile(
        LaunchConfiguration("vehicle_config").perform(context)
    )
    vehicle_type = profile["vehicle_type"]
    robot_name = profile["robot_name"]

    hardware_share = get_package_share_directory("mentor_pi_hardwares")
    bringup_share = get_package_share_directory("mentor_pi_bringup")
    description_file = os.path.join(
        hardware_share, "config", vehicle_type, "mentor_pi.urdf.xacro"
    )
    controllers_file = os.path.join(
        os.path.dirname(profile["path"]), "controllers.yaml"
    )
    if not os.path.isfile(controllers_file):
        raise ValueError(
            "vehicle controller profile is not a readable sibling file: "
            f"{controllers_file}"
        )
    hardware_settings = profile["hardware"]
    xacro_settings = []
    for name, value in hardware_settings.items():
        rendered = str(value).lower() if isinstance(value, bool) else str(value)
        xacro_settings.extend([f" {name}:=", rendered])
    bringup_launch = os.path.join(bringup_share, "launch", "controller.launch.py")
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                description_file,
                " robot_name:=",
                robot_name,
                *xacro_settings,
            ]
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
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{_VEHICLE_CONTROLLER_NAME}",
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
        _shutdown_on_failure(
            drive_controller_spawner, f"{_VEHICLE_CONTROLLER_NAME} spawner"
        ),
        drive_controller_spawner,
    ]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={"robot_name": robot_name}.items(),
        ),
        _shutdown_on_exit(robot_state_publisher, "robot state publisher"),
        robot_state_publisher,
        _shutdown_on_exit(controller_manager, "controller manager"),
        controller_manager,
        TimerAction(
            period=2.0,
            actions=delayed_actions,
        ),
    ]


def generate_vehicle_launch():
    hardware_share = get_package_share_directory("mentor_pi_hardwares")
    default_config = os.path.join(
        hardware_share, "config", "generated", "vehicle.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_config", default_value=default_config),
            OpaqueFunction(function=_launch_vehicle),
        ]
    )


def generate_launch_description():
    return generate_vehicle_launch()
