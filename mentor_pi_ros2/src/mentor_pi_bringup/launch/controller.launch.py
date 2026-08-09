import os
import re
import stat
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_REQUIRED_RUNTIME_ACK = "PID_FIRMWARE_ACTUATORS_PREPARED"
_DEVICE_PATTERN = re.compile(r"^/dev/[A-Za-z0-9._/+:-]+$")


def _property_value(properties, key):
    prefix = f"{key}="
    for line in properties.splitlines():
        if line.startswith(prefix):
            return line[len(prefix) :]
    return ""


def _validate_device(serial_device):
    if (
        _DEVICE_PATTERN.fullmatch(serial_device) is None
        or "/../" in serial_device
        or serial_device.endswith("/..")
        or "/./" in serial_device
        or serial_device.endswith("/.")
    ):
        raise RuntimeError("serial_device must be an explicit /dev path")

    try:
        device_stat = os.stat(serial_device)
    except OSError as error:
        raise RuntimeError(f"serial device is unavailable: {error}") from error
    if not stat.S_ISCHR(device_stat.st_mode):
        raise RuntimeError("serial_device is not a character device")

    resolved_device = os.path.realpath(serial_device)
    if not resolved_device.startswith("/dev/"):
        raise RuntimeError("serial_device did not resolve below /dev")

    try:
        properties = subprocess.run(
            ["udevadm", "info", "--query=property", f"--name={resolved_device}"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError("udevadm could not inspect serial_device") from error
    if (
        _property_value(properties, "ID_VENDOR_ID") != "1a86"
        or _property_value(properties, "ID_MODEL_ID") != "55d4"
    ):
        raise RuntimeError("serial_device is not the Mentor Pi CH9102F (1a86:55d4)")

    try:
        owner_status = subprocess.run(
            ["fuser", resolved_device],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
    except OSError as error:
        raise RuntimeError("fuser is required to verify serial ownership") from error
    if owner_status == 0:
        raise RuntimeError("another process already owns serial_device")
    return resolved_device


def _validate_development_artifact():
    project_root = os.environ.get("MENTOR_PI_PROJECT_ROOT", "")
    verifier = os.environ.get("MENTOR_PI_FIRMWARE_VERIFIER", "")
    if not os.path.isabs(project_root) or not os.path.isdir(project_root):
        raise RuntimeError(
            "MENTOR_PI_PROJECT_ROOT is missing; enter with make shell first"
        )
    if not os.path.isabs(verifier) or not os.access(verifier, os.X_OK):
        raise RuntimeError(
            "MENTOR_PI_FIRMWARE_VERIFIER is missing; enter with make shell first"
        )
    try:
        subprocess.run([verifier, "PID", project_root], check=True)
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
    serial_device = LaunchConfiguration("serial_device").perform(context)
    agent_executable = LaunchConfiguration("agent_executable").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    agent_verbosity = LaunchConfiguration("agent_verbosity").perform(context)

    if os.environ.get("RRCLITE_RUNTIME_ACK") != _REQUIRED_RUNTIME_ACK:
        raise RuntimeError(
            f"set RRCLITE_RUNTIME_ACK={_REQUIRED_RUNTIME_ACK} only after "
            "completing the guarded fixture checks"
        )
    serial_device = _validate_device(serial_device)
    if not os.path.isabs(agent_executable) or not os.access(
        agent_executable, os.X_OK
    ):
        raise RuntimeError("agent_executable must be an executable absolute path")
    if os.environ.get("MENTOR_PI_DEVELOPMENT_RUNTIME") == "1":
        _validate_development_artifact()

    agent = ExecuteProcess(
        cmd=[
            agent_executable,
            "serial",
            "--dev",
            serial_device,
            "--baudrate",
            "1000000",
            f"-v{agent_verbosity}",
        ],
        name="micro_ros_agent",
        output="screen",
        additional_env={"MENTOR_PI_RRCLITE_AUTORESET": "1"},
    )
    supervisor = Node(
        package="mentor_pi_bringup",
        executable="configuration_supervisor",
        output="screen",
        parameters=[config_file],
    )
    return [
        _shutdown_on_exit(agent, "micro-ROS Agent"),
        _shutdown_on_exit(supervisor, "configuration supervisor"),
        agent,
        supervisor,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_device", default_value="/dev/mentor_pi_mcu"
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mentor_pi_bringup"), "config", "controller.yaml"]
                ),
                description="Absolute controller configuration YAML path",
            ),
            DeclareLaunchArgument("agent_verbosity", default_value="4"),
            DeclareLaunchArgument(
                "agent_executable",
                default_value=EnvironmentVariable(
                    "MENTOR_PI_AGENT_EXECUTABLE",
                    default_value="/opt/mentor_pi/bin/mentor_pi_micro_ros_agent",
                ),
            ),
            OpaqueFunction(function=_launch_controller),
        ]
    )
