import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_REQUIRED_RUNTIME_ACK = "PID_FIRMWARE_ACTUATORS_PREPARED"


def _validate_development_artifact():
    project_root = os.environ.get("MENTOR_PI_PROJECT_ROOT", "")
    verifier = os.environ.get("MENTOR_PI_FIRMWARE_VERIFIER", "")
    if not os.path.isabs(project_root) or not os.path.isdir(project_root):
        raise RuntimeError(
            "MENTOR_PI_PROJECT_ROOT is missing; use make -C ros2_ws run"
        )
    if not os.path.isabs(verifier) or not os.access(verifier, os.X_OK):
        raise RuntimeError(
            "MENTOR_PI_FIRMWARE_VERIFIER is missing; use make -C ros2_ws run"
        )
    try:
        subprocess.run([verifier], check=True, cwd=project_root)
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError("the authoritative PID firmware artifact is invalid") from error


def _shutdown_on_exit(action, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=action,
            on_exit=[EmitEvent(event=Shutdown(reason=f"{label} exited"))],
        )
    )


def _launch_controller(context):
    config_file = LaunchConfiguration("config_file").perform(context)

    if os.environ.get("RRCLITE_RUNTIME_ACK") != _REQUIRED_RUNTIME_ACK:
        raise RuntimeError(
            f"set RRCLITE_RUNTIME_ACK={_REQUIRED_RUNTIME_ACK} only after "
            "completing the guarded fixture checks"
        )
    if os.environ.get("MENTOR_PI_DEVELOPMENT_RUNTIME") == "1":
        _validate_development_artifact()
    supervisor = Node(
        package="mentor_pi_bringup",
        executable="configuration_supervisor",
        output="screen",
        parameters=[config_file],
    )
    return [
        _shutdown_on_exit(supervisor, "configuration supervisor"),
        supervisor,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mentor_pi_bringup"), "config", "controller.yaml"]
                ),
                description="Absolute controller configuration YAML path",
            ),
            OpaqueFunction(function=_launch_controller),
        ]
    )
