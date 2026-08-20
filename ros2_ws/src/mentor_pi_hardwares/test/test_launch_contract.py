#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent
import pytest
import yaml


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def vehicle_launch_module(share):
    return load_launch(share / "launch" / "vehicle.launch.py")


def test_vehicle_launch_rejects_the_dev_container(monkeypatch):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    monkeypatch.setattr(
        module.os.path, "exists", lambda path: path == "/.dockerenv"
    )
    with pytest.raises(
        RuntimeError, match="native Ubuntu 22.04, not the Dev Container"
    ):
        module._validate_native_runtime()


def test_controller_spawner_exit_only_shuts_down_on_failure():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)

    assert module._shutdown_if_failed(
        SimpleNamespace(returncode=0), None, "controller spawner"
    ) == []
    actions = module._shutdown_if_failed(
        SimpleNamespace(returncode=7), None, "controller spawner"
    )
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert actions[0].event.reason == "controller spawner failed with exit code 7"


def render_description(share, mode, robot_name=None):
    profile = vehicle_launch_module(share).load_vehicle_profile(
        str(share / "config" / mode / "hardware.yaml")
    )
    resolved_robot_name = robot_name or profile["robot_name"]
    description = share / "config" / mode / "mentor_pi.urdf.xacro"
    arguments = [
        "xacro",
        str(description),
        f"robot_name:={resolved_robot_name}",
    ]
    for name, value in profile["hardware"].items():
        rendered = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.append(f"{name}:={rendered}")
    result = subprocess.run(
        arguments,
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def render_simulation_description(share, mode, robot_name="mentor_pi_sim"):
    description = share / "config" / mode / "simulation.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(description), f"robot_name:={robot_name}"],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def parameters(root):
    hardware = root.find("./ros2_control/hardware")
    assert hardware is not None
    return {
        parameter.attrib["name"]: parameter.text
        for parameter in hardware.findall("param")
    }


_LINK_SUFFIXES = {
    "ackermann": {
        "rear_axle_footprint",
        "base_footprint",
        "base_link",
        "wheel_left_front_link",
        "wheel_right_front_link",
        "wheel_left_rear_link",
        "wheel_right_rear_link",
        "imu_link",
    },
    "mecanum": {
        "base_footprint",
        "base_link",
        "wheel_left_front_link",
        "wheel_right_front_link",
        "wheel_left_rear_link",
        "wheel_right_rear_link",
        "imu_link",
    },
}

_JOINT_NAMES = {
    "ackermann": [
        "rear_axle_footprint_joint",
        "base_footprint_joint",
        "wheel_left_front_joint",
        "wheel_right_front_joint",
        "wheel_left_rear_joint",
        "wheel_right_rear_joint",
        "imu_joint",
    ],
    "mecanum": [
        "base_footprint_joint",
        "wheel_left_front_joint",
        "wheel_right_front_joint",
        "wheel_left_rear_joint",
        "wheel_right_rear_joint",
        "imu_joint",
    ],
}

_CONTROL_JOINT_NAMES = [
    "wheel_left_front_joint",
    "wheel_right_front_joint",
    "wheel_left_rear_joint",
    "wheel_right_rear_joint",
]


def assert_description_uses_one_robot_prefix(root, mode, robot_name):
    prefix = f"{robot_name}/"
    expected_links = {
        f"{prefix}{suffix}" for suffix in _LINK_SUFFIXES[mode]
    }
    links = {link.attrib["name"] for link in root.findall("./link")}
    assert links == expected_links
    assert all(not name.startswith(f"{prefix}{prefix}") for name in links)

    joints = root.findall("./joint")
    assert [joint.attrib["name"] for joint in joints] == _JOINT_NAMES[mode]
    for joint in joints:
        assert joint.find("parent").attrib["link"] in expected_links
        assert joint.find("child").attrib["link"] in expected_links

    control_joint_names = [
        joint.attrib["name"] for joint in root.findall("./ros2_control/joint")
    ]
    assert control_joint_names == _CONTROL_JOINT_NAMES


@pytest.mark.parametrize("mode", ["ackermann", "mecanum"])
@pytest.mark.parametrize("robot_name", ["robot_one", "fleet/robot_two"])
def test_real_and_simulated_descriptions_bake_robot_name_into_links(
    mode, robot_name
):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    assert_description_uses_one_robot_prefix(
        render_description(share, mode, robot_name), mode, robot_name
    )
    assert_description_uses_one_robot_prefix(
        render_simulation_description(share, mode, robot_name),
        mode,
        robot_name,
    )


def test_xacro_modes_export_expected_plugins_interfaces_and_configuration():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))

    mecanum = render_description(share, "mecanum")
    assert mecanum.findtext("./ros2_control/hardware/plugin") == (
        "mentor_pi/MecanumHardware"
    )
    mecanum_parameters = parameters(mecanum)
    assert mecanum_parameters["feedback_timeout_ms"] == "100"
    assert mecanum_parameters["imu_timeout_ms"] == "100"
    assert mecanum_parameters["wheel_radius_m"] == "0.0325"
    assert mecanum_parameters["wheel_projection_sum_m"] == "0.14"
    assert mecanum_parameters["linear_adrc_input_gain_per_second"] == "5.0"
    assert mecanum_parameters["linear_adrc_measurement_lpf_cutoff_hz"] == "5.0"
    assert mecanum_parameters["yaw_adrc_input_gain_per_second"] == "5.0"
    assert mecanum_parameters["yaw_adrc_measurement_lpf_cutoff_hz"] == "5.0"
    joint_names = [
        joint.attrib["name"] for joint in mecanum.findall("./ros2_control/joint")
    ]
    assert joint_names == [
        "wheel_left_front_joint",
        "wheel_right_front_joint",
        "wheel_left_rear_joint",
        "wheel_right_rear_joint",
    ]

    ackermann = render_description(share, "ackermann")
    assert ackermann.findtext("./ros2_control/hardware/plugin") == (
        "mentor_pi/AckermannHardware"
    )
    ackermann_parameters = parameters(ackermann)
    assert ackermann_parameters["steering_pwm_channel"] == "3"
    assert ackermann_parameters["steering_pwm_center_us"] == "1500"
    assert ackermann_parameters["steering_inverted"].lower() == "true"
    assert ackermann_parameters["feedback_timeout_ms"] == "100"
    assert ackermann_parameters["imu_timeout_ms"] == "100"
    assert ackermann_parameters["rear_wheel_radius_m"] == "0.0325"
    assert ackermann_parameters["wheelbase_m"] == "0.135"
    assert ackermann_parameters["yaw_adrc_input_gain_per_mps"] == "30.0"
    assert ackermann_parameters["linear_adrc_measurement_lpf_cutoff_hz"] == "5.0"
    assert ackermann_parameters["yaw_adrc_measurement_lpf_cutoff_hz"] == "5.0"
    assert ackermann_parameters["yaw_adrc_minimum_speed_mps"] == "0.1"
    rear_axle_joint = ackermann.find("./joint[@name='rear_axle_footprint_joint']")
    assert rear_axle_joint is not None
    assert rear_axle_joint.find("parent").attrib["link"] == (
        "mentor_pi/base_footprint"
    )
    assert rear_axle_joint.find("child").attrib["link"] == (
        "mentor_pi/rear_axle_footprint"
    )
    assert rear_axle_joint.find("origin").attrib["xyz"] == "-0.0675 0 0"

    with open(
        share / "config" / "ackermann" / "controllers.yaml", encoding="utf-8"
    ) as stream:
        ackermann_controllers = yaml.safe_load(stream)
    ackermann_controller = ackermann_controllers["/**/vehicle"][
        "ros__parameters"
    ]
    assert ackermann_controllers["/**/controller_manager"]["ros__parameters"][
        "vehicle"
    ]["type"] == "ackermann_steering_controller/AckermannSteeringController"
    assert ackermann_controller["base_frame_id"] == "rear_axle_footprint"
    assert ackermann_controller["rear_wheels_names"] == [
        "wheel_right_rear_joint",
        "wheel_left_rear_joint",
    ]
    assert ackermann_controller["front_wheels_names"] == [
        "wheel_right_front_joint",
        "wheel_left_front_joint",
    ]
    assert ackermann_controller["front_wheel_track"] == 0.140
    assert ackermann_controller["rear_wheel_track"] == 0.140
    assert ackermann_controller["wheelbase"] == 0.135
    assert ackermann_controller["front_wheels_radius"] == 0.0325
    assert ackermann_controller["rear_wheels_radius"] == 0.0325
    assert ackermann_controller["enable_odom_tf"] is False

    with open(
        share / "config" / "mecanum" / "controllers.yaml", encoding="utf-8"
    ) as stream:
        mecanum_controllers = yaml.safe_load(stream)
    assert mecanum_controllers["/**/controller_manager"]["ros__parameters"][
        "vehicle"
    ]["type"] == "mecanum_drive_controller/MecanumDriveController"
    assert "/**/vehicle" in mecanum_controllers
    assert mecanum_controllers["/**/vehicle"]["ros__parameters"][
        "enable_odom_tf"
    ] is False
    assert "/**/ackermann_steering_controller" not in ackermann_controllers
    assert "/**/mecanum_drive_controller" not in mecanum_controllers


def test_simulation_xacros_use_only_actuator_limited_plugins():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    expected = {
        "ackermann": "mentor_pi/AckermannSimulationHardware",
        "mecanum": "mentor_pi/MecanumSimulationHardware",
    }
    for mode, plugin in expected.items():
        root = render_simulation_description(share, mode, f"{mode}_sim")
        assert root.findtext("./ros2_control/hardware/plugin") == plugin
        simulation_parameters = parameters(root)
        assert simulation_parameters["robot_name"] == f"{mode}_sim"
        assert math.isclose(
            float(simulation_parameters["wheel_angular_speed_limit_rad_s"]),
            37.699112,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            float(
                simulation_parameters[
                    "wheel_angular_acceleration_limit_rad_s2"
                ]
            ),
            188.495559,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        )
        assert "feedback_timeout_ms" not in simulation_parameters
        assert "imu_timeout_ms" not in simulation_parameters
    ackermann = parameters(render_simulation_description(share, "ackermann"))
    assert ackermann["steering_angle_min_rad"] == "-0.6"
    assert ackermann["steering_angle_max_rad"] == "0.6"
    assert ackermann["steering_rate_limit_rad_s"] == "60.0"


def test_default_and_custom_profiles_are_yaml_authoritative(tmp_path):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    for mode in ("mecanum", "ackermann"):
        profile = module.load_vehicle_profile(
            str(share / "config" / mode / "hardware.yaml")
        )
        assert profile["robot_name"] == "mentor_pi"
        assert profile["vehicle_type"] == mode

    custom = tmp_path / "robot_two.yaml"
    custom.write_text(
        yaml.safe_dump(
            {
                "vehicle": {
                    "robot_name": "fleet/robot_two",
                    "vehicle_type": "mecanum",
                },
                "hardware": {"feedback_timeout_ms": 75},
            }
        ),
        encoding="utf-8",
    )
    profile = module.load_vehicle_profile(str(custom))
    assert profile["robot_name"] == "fleet/robot_two"
    assert profile["vehicle_type"] == "mecanum"
    assert profile["hardware"]["feedback_timeout_ms"] == 75


@pytest.mark.parametrize(
    "profile,error",
    [
        ({}, "vehicle must contain"),
        (
            {
                "vehicle": {"robot_name": "/absolute", "vehicle_type": "mecanum"},
                "hardware": {},
            },
            "relative ROS namespace",
        ),
        (
            {
                "vehicle": {"robot_name": "robot-two", "vehicle_type": "mecanum"},
                "hardware": {},
            },
            "relative ROS namespace",
        ),
        (
            {
                "vehicle": {"robot_name": "fleet/2", "vehicle_type": "mecanum"},
                "hardware": {},
            },
            "relative ROS namespace",
        ),
        (
            {
                "vehicle": {"robot_name": "mentor_pi", "vehicle_type": "tracked"},
                "hardware": {},
            },
            "unsupported vehicle_type",
        ),
        (
            {
                "vehicle": {"robot_name": "mentor_pi", "vehicle_type": "mecanum"},
                "hardware": {"feedback_timeout_ms": float("nan")},
            },
            "finite numeric or boolean",
        ),
    ],
)
def test_invalid_profiles_fail_closed(tmp_path, profile, error):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(ValueError, match=error):
        module.load_vehicle_profile(str(path))


def test_missing_relative_and_malformed_profiles_fail_closed(tmp_path):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    with pytest.raises(ValueError, match="absolute path"):
        module.load_vehicle_profile("relative.yaml")
    with pytest.raises(ValueError, match="readable file"):
        module.load_vehicle_profile(str(tmp_path / "missing.yaml"))
    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("vehicle: [\n", encoding="utf-8")
    with pytest.raises(ValueError, match="could not read"):
        module.load_vehicle_profile(str(malformed))


def test_physical_launch_is_command_only():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    launch_directory = share / "launch"
    assert sorted(path.name for path in launch_directory.glob("*.py")) == [
        "foxglove.launch.py",
        "simulation.launch.py",
        "vehicle.launch.py",
    ]
    module = load_launch(launch_directory / "vehicle.launch.py")
    assert module._VEHICLE_CONTROLLER_NAME == "vehicle"
    description = module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {"vehicle_config"}
    source = (launch_directory / "vehicle.launch.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "mentor_pi_tracking",
        "trajectory_tracker",
        "tracking_controller",
        "tracking_algorithm",
        "vehicle_pose",
        "vrpn_mocap",
    ):
        assert forbidden not in source
    assert 'executable="ros2_control_node"' in source
    assert 'executable="spawner"' in source


def test_simulation_launch_is_command_only():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    path = share / "launch" / "simulation.launch.py"
    module = load_launch(path)
    description = module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {"vehicle_type", "robot_name"}
    source = path.read_text(encoding="utf-8")
    assert 'executable="ros2_control_node"' in source
    assert 'name="spawner_vehicle"' in source
    for forbidden in (
        "mentor_pi_bringup",
        "mentor_pi_tracking",
        "trajectory_tracker",
        "tracking_controller",
        "tracking_algorithm",
        "vehicle_pose",
        "vrpn_mocap",
        "configuration_supervisor",
    ):
        assert forbidden not in source


def test_controller_state_estimates_are_private():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    physical = vehicle_launch_module(share)
    simulation = load_launch(share / "launch" / "simulation.launch.py")
    expected = [
        (
            "/fleet/robot_two/vehicle/odometry",
            "/fleet/robot_two/vehicle/_controller_odometry",
        ),
        (
            "/fleet/robot_two/vehicle/tf_odometry",
            "/fleet/robot_two/vehicle/_controller_tf_odometry",
        ),
    ]
    assert physical.controller_state_estimate_remappings(
        "fleet/robot_two"
    ) == expected
    assert simulation.controller_state_estimate_remappings(
        "fleet/robot_two"
    ) == expected


def test_controller_profiles_preserve_timeout_and_stamped_reference():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    for vehicle_type in ("ackermann", "mecanum"):
        with open(
            share / "config" / vehicle_type / "controllers.yaml",
            encoding="utf-8",
        ) as stream:
            document = yaml.safe_load(stream)
        parameters = document["/**/vehicle"]["ros__parameters"]
        assert parameters["reference_timeout"] == pytest.approx(0.1)
        assert parameters["use_stamped_vel"] is True
    ackermann = yaml.safe_load(
        (share / "config" / "ackermann" / "controllers.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert ackermann["/**/vehicle"]["ros__parameters"]["use_stamped_vel"] is True


def test_foxglove_is_a_separate_loopback_bridge_launch():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    path = share / "launch" / "foxglove.launch.py"
    module = load_launch(path)
    description = module.generate_launch_description()
    defaults = {
        entity.name: "".join(
            getattr(value, "text", str(value)) for value in entity.default_value
        )
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert defaults == {"address": "127.0.0.1", "port": "8765"}
    source = path.read_text(encoding="utf-8")
    assert "foxglove_bridge_launch.xml" in source
    assert "rviz" not in source.lower()
