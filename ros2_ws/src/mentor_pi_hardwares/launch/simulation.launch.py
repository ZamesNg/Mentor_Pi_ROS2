import math
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
_TRACKING_ALGORITHMS = frozenset(("mpc", "adrc"))
_VEHICLE_CONTROLLER_NAME = "vehicle"
_DEFAULT_TRACKING_CONTROLLER = "auto"
_DEFAULT_TRACKING_ALGORITHM = "adrc"
_CONTROLLER_PLUGINS = {
    ("mecanum", "mpc"): "mentor_pi_tracking/MecanumMpc",
    ("ackermann", "mpc"): "mentor_pi_tracking/AckermannMpc",
    ("mecanum", "adrc"): "mentor_pi_tracking/MecanumAdrc",
    ("ackermann", "adrc"): "mentor_pi_tracking/AckermannAdrc",
}
_TRACKING_GEOMETRY = {
    "mecanum": {
        "wheel_radius": 0.0325,
        "wheelbase": 0.145,
        "wheel_track": 0.140,
        "rear_axle_to_geometry_center": 0.0,
        "mecanum_radius_sum": 0.14,
        "max_steering_angle": 0.5,
        "driven_wheel_angular_speed_limit_rad_s": 37.69911184307752,
    },
    "ackermann": {
        "wheel_radius": 0.0325,
        "wheelbase": 0.135,
        "wheel_track": 0.140,
        "rear_axle_to_geometry_center": 0.0675,
        "mecanum_radius_sum": 0.14,
        "max_steering_angle": 0.6,
        "driven_wheel_angular_speed_limit_rad_s": 37.69911184307752,
    },
}


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


def _finite_launch_value(context, name):
    text = LaunchConfiguration(name).perform(context)
    try:
        value = float(text)
    except ValueError as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def tracking_parameters(vehicle_type, tracking_algorithm):
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError(f"unsupported vehicle_type: {vehicle_type}")
    if tracking_algorithm not in _TRACKING_ALGORITHMS:
        raise ValueError("tracking_algorithm must be mpc or adrc")
    return {
        "vehicle_type": vehicle_type,
        "tracking_algorithm": tracking_algorithm,
        "controller_plugin": _CONTROLLER_PLUGINS[
            (vehicle_type, tracking_algorithm)
        ],
        **_TRACKING_GEOMETRY[vehicle_type],
    }


def resolve_tracking_controller(selection, vehicle_type):
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError("unsupported vehicle_type")
    if selection == "auto":
        return vehicle_type
    if selection == "none":
        return None
    if selection != vehicle_type:
        raise ValueError(
            "tracking_controller must be auto, none, or match the selected "
            "vehicle_type"
        )
    return vehicle_type


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


def simulation_pose_parameters(
    robot_name, vehicle_type, initial_x_m, initial_y_m, initial_yaw_rad
):
    if not _validate_robot_name(robot_name):
        raise ValueError("robot_name must be a valid relative ROS namespace")
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError(f"unsupported vehicle_type: {vehicle_type}")
    values = (initial_x_m, initial_y_m, initial_yaw_rad)
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in values):
        raise ValueError("initial geometry-center pose must be finite")
    offset = _TRACKING_GEOMETRY[vehicle_type][
        "rear_axle_to_geometry_center"
    ]
    return {
        "input_type": "controller_odometry",
        "source_to_geometry_center_m": offset,
        "geometry_center_frame_id": f"{robot_name}/base_footprint",
        "output_frame_id": "map",
        "output_origin_x_m": (
            initial_x_m - offset * math.cos(initial_yaw_rad)
        ),
        "output_origin_y_m": (
            initial_y_m - offset * math.sin(initial_yaw_rad)
        ),
        "output_origin_yaw_rad": initial_yaw_rad,
    }


def _launch_simulation(context):
    vehicle_type = LaunchConfiguration("vehicle_type").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if vehicle_type not in _VEHICLE_TYPES:
        raise ValueError("vehicle_type must be ackermann or mecanum")
    if not _validate_robot_name(robot_name):
        raise ValueError("robot_name must be a valid relative ROS namespace")
    initial_x_m = _finite_launch_value(context, "initial_x_m")
    initial_y_m = _finite_launch_value(context, "initial_y_m")
    initial_yaw_rad = _finite_launch_value(context, "initial_yaw_rad")

    hardware_share = get_package_share_directory("mentor_pi_hardwares")
    tracking_share = get_package_share_directory("mentor_pi_tracking")
    tracking_controller = resolve_tracking_controller(
        LaunchConfiguration("tracking_controller").perform(context),
        vehicle_type,
    )
    tracking_algorithm = LaunchConfiguration("tracking_algorithm").perform(
        context
    )
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
    pose_adapter = Node(
        package="mentor_pi_hardwares",
        executable="vehicle_pose",
        name="vehicle_pose",
        namespace=robot_name,
        output="screen",
        parameters=[
            simulation_pose_parameters(
                robot_name,
                vehicle_type,
                initial_x_m,
                initial_y_m,
                initial_yaw_rad,
            )
        ],
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
    if tracking_controller is not None:
        tracking_config = os.path.join(
            tracking_share, "config", f"{tracking_algorithm}.yaml"
        )
        tracker = Node(
            package="mentor_pi_tracking",
            executable="trajectory_tracker",
            namespace=robot_name,
            output="screen",
            parameters=[
                tracking_config,
                tracking_parameters(vehicle_type, tracking_algorithm),
            ],
        )
        delayed_actions.extend(
            [_shutdown_on_exit(tracker, "tracking controller"), tracker]
        )
    return [
        _shutdown_on_exit(robot_state_publisher, "robot state publisher"),
        robot_state_publisher,
        _shutdown_on_exit(controller_manager, "controller manager"),
        controller_manager,
        _shutdown_on_exit(pose_adapter, "vehicle pose adapter"),
        pose_adapter,
        TimerAction(period=2.0, actions=delayed_actions),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_type"),
            DeclareLaunchArgument("robot_name", default_value="mentor_pi_sim"),
            DeclareLaunchArgument("initial_x_m", default_value="0.0"),
            DeclareLaunchArgument("initial_y_m", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument(
                "tracking_controller",
                default_value=_DEFAULT_TRACKING_CONTROLLER,
            ),
            DeclareLaunchArgument(
                "tracking_algorithm",
                default_value=_DEFAULT_TRACKING_ALGORITHM,
            ),
            OpaqueFunction(function=_launch_simulation),
        ]
    )
