#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def vehicle_launch_module(share):
    return load_launch(share / "launch" / "vehicle_launch.py")


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


def render_description(share, mode):
    profile = vehicle_launch_module(share).load_vehicle_profile(
        str(share / "config" / mode / "hardware.yaml")
    )
    description = share / "config" / mode / "mentor_pi.urdf.xacro"
    arguments = [
        "xacro",
        str(description),
        f"robot_name:={profile['robot_name']}",
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


def parameters(root):
    hardware = root.find("./ros2_control/hardware")
    assert hardware is not None
    return {
        parameter.attrib["name"]: parameter.text
        for parameter in hardware.findall("param")
    }


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
    assert mecanum_parameters["yaw_adrc_input_gain_per_second"] == "5.0"
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
    assert ackermann_parameters["imu_timeout_ms"] == "100"
    assert ackermann_parameters["rear_wheel_radius_m"] == "0.0325"
    assert ackermann_parameters["wheelbase_m"] == "0.135"
    assert ackermann_parameters["yaw_adrc_input_gain_per_mps"] == "30.0"
    assert ackermann_parameters["yaw_adrc_minimum_speed_mps"] == "0.1"
    rear_axle_joint = ackermann.find("./joint[@name='rear_axle_footprint_joint']")
    assert rear_axle_joint is not None
    assert rear_axle_joint.find("parent").attrib["link"] == "rear_axle_footprint"
    assert rear_axle_joint.find("child").attrib["link"] == "base_footprint"
    assert rear_axle_joint.find("origin").attrib["xyz"] == "0.0675 0 0"

    with open(
        share / "config" / "ackermann" / "controllers.yaml", encoding="utf-8"
    ) as stream:
        ackermann_controller = yaml.safe_load(stream)[
            "/**/ackermann_steering_controller"
        ]["ros__parameters"]
    assert ackermann_controller["base_frame_id"] == "rear_axle_footprint"
    assert ackermann_controller["front_wheel_track"] == 0.140
    assert ackermann_controller["rear_wheel_track"] == 0.140
    assert ackermann_controller["wheelbase"] == 0.135
    assert ackermann_controller["front_wheels_radius"] == 0.0325
    assert ackermann_controller["rear_wheels_radius"] == 0.0325


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


def test_launches_accept_only_a_vehicle_profile_for_name_and_type():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    launch_directory = share / "launch"
    for name in (
        "vehicle.launch.py",
        "mecanum.launch.py",
        "ackermann.launch.py",
        "exp_vehicle_launch.py",
    ):
        module = load_launch(launch_directory / name)
        description = module.generate_launch_description()
        arguments = {
            entity.name
            for entity in description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        assert {
            "vehicle_config",
            "tracking_controller",
            "tracking_algorithm",
        } <= arguments
        assert "start_bringup" not in arguments
        assert "robot_name" not in arguments
        assert "vehicle_type" not in arguments

    launch_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in launch_directory.glob("*.py")
    )
    for forbidden in (
        "MENTOR_PI_NAME",
        "MENTOR_PI_TYPE",
        "MENTOR_PI_ROBOT_NAME",
        'LaunchConfiguration("robot_name")',
        'LaunchConfiguration("vehicle_type")',
        'LaunchConfiguration("start_bringup")',
        'name="controller_manager"',
    ):
        assert forbidden not in launch_sources


def test_tracking_parameters_select_generic_tracker_plugin_and_geometry():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)

    assert module.tracking_parameters("mecanum", "mpc") == {
        "vehicle_type": "mecanum",
        "tracking_algorithm": "mpc",
        "controller_plugin": "mentor_pi_tracking/MecanumMpc",
        "wheel_radius": 0.0325,
        "wheelbase": 0.145,
        "wheel_track": 0.140,
        "rear_axle_to_geometry_center": 0.0,
        "mecanum_radius_sum": 0.14,
        "max_steering_angle": 0.5,
    }
    assert module.tracking_parameters("ackermann", "adrc") == {
        "vehicle_type": "ackermann",
        "tracking_algorithm": "adrc",
        "controller_plugin": "mentor_pi_tracking/AckermannAdrc",
        "wheel_radius": 0.0325,
        "wheelbase": 0.135,
        "wheel_track": 0.140,
        "rear_axle_to_geometry_center": 0.0675,
        "mecanum_radius_sum": 0.14,
        "max_steering_angle": 0.6,
    }


@pytest.mark.parametrize(
    "vehicle_type,tracking_algorithm,error",
    [
        ("tracked", "mpc", "unsupported vehicle_type"),
        ("ackermann", "pid", "tracking_algorithm must be mpc or adrc"),
    ],
)
def test_tracking_parameters_fail_closed_for_unknown_values(
    vehicle_type, tracking_algorithm, error
):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    with pytest.raises(ValueError, match=error):
        module.tracking_parameters(vehicle_type, tracking_algorithm)


def test_tracking_parameters_validate_the_selected_hardware_geometry():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    profile = module.load_vehicle_profile(
        str(share / "config" / "ackermann" / "hardware.yaml")
    )
    parameters = module.tracking_parameters(
        "ackermann", "adrc", profile["hardware"]
    )
    assert parameters["wheel_radius"] == 0.0325
    assert parameters["wheelbase"] == 0.135
    assert parameters["max_steering_angle"] == 0.6

    changed = dict(profile["hardware"])
    changed["steering_angle_max_rad"] = 0.4
    with pytest.raises(ValueError, match="steering_angle_max_rad must be 0.6"):
        module.tracking_parameters("ackermann", "adrc", changed)


def test_tracking_contract_remains_on_the_fixed_mentor_pi_namespace():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    source = (share / "launch" / "vehicle_launch.py").read_text(encoding="utf-8")
    assert 'tracking_controller != "none" and robot_name != "mentor_pi"' in source
    assert "trajectory tracking uses the fixed /mentor_pi API" in source
