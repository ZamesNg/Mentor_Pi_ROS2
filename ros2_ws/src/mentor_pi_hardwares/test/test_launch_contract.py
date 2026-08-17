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

    with open(
        share / "config" / "mecanum" / "controllers.yaml", encoding="utf-8"
    ) as stream:
        mecanum_controllers = yaml.safe_load(stream)
    assert mecanum_controllers["/**/controller_manager"]["ros__parameters"][
        "vehicle"
    ]["type"] == "mecanum_drive_controller/MecanumDriveController"
    assert "/**/vehicle" in mecanum_controllers
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


def test_physical_launch_accepts_only_a_vehicle_profile_for_name_and_type():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    launch_directory = share / "launch"
    assert sorted(path.name for path in launch_directory.glob("*.py")) == [
        "foxglove.launch.py",
        "simulation.launch.py",
        "vehicle.launch.py",
    ]
    module = load_launch(launch_directory / "vehicle.launch.py")
    assert module._VEHICLE_CONTROLLER_NAME == "vehicle"
    assert module._DEFAULT_TRACKING_CONTROLLER == "auto"
    assert module._DEFAULT_TRACKING_ALGORITHM == "adrc"
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

    launch_source = (launch_directory / "vehicle.launch.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "MENTOR_PI_NAME",
        "MENTOR_PI_TYPE",
        "MENTOR_PI_ROBOT_NAME",
        'LaunchConfiguration("robot_name")',
        'LaunchConfiguration("vehicle_type")',
        'LaunchConfiguration("start_bringup")',
        'name="controller_manager"',
        '"frame_prefix"',
        '"mecanum_drive_controller"',
        '"ackermann_steering_controller"',
    ):
        assert forbidden not in launch_source


def test_simulation_launch_is_separate_and_has_no_physical_dependencies():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    path = share / "launch" / "simulation.launch.py"
    module = load_launch(path)
    description = module.generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {
        "vehicle_type",
        "robot_name",
        "initial_x_m",
        "initial_y_m",
        "initial_yaw_rad",
        "tracking_controller",
        "tracking_algorithm",
    }
    assert module._DEFAULT_TRACKING_CONTROLLER == "auto"
    assert module._DEFAULT_TRACKING_ALGORITHM == "adrc"
    source = path.read_text(encoding="utf-8")
    assert 'executable="ros2_control_node"' in source
    assert 'executable="vehicle_odometry"' in source
    assert 'executable="trajectory_tracker"' in source
    assert 'name="spawner_vehicle"' in source
    for forbidden in (
        "mentor_pi_bringup",
        "configuration_supervisor",
        "heartbeat",
        "motors/state",
        "motion_authorization",
        '"imu"',
        "/imu",
        '"frame_prefix"',
    ):
        assert forbidden not in source


def test_simulation_initial_pose_is_geometry_center_pose():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = load_launch(share / "launch" / "simulation.launch.py")
    ackermann = module.simulation_odometry_parameters(
        "ackermann_sim", "ackermann", 2.0, -3.0, math.pi / 2.0
    )
    assert ackermann.pop("geometry_center_frame_id") == (
        "ackermann_sim/base_footprint"
    )
    assert ackermann.pop("output_odom_frame_id") == "ackermann_sim/odom"
    assert ackermann == pytest.approx(
        {
            "source_to_geometry_center_m": 0.0675,
            "output_odom_origin_x_m": 2.0,
            "output_odom_origin_y_m": -3.0675,
            "output_odom_origin_yaw_rad": math.pi / 2.0,
        }
    )
    mecanum = module.simulation_odometry_parameters(
        "mecanum_sim", "mecanum", -1.0, 4.0, -0.5
    )
    assert mecanum.pop("geometry_center_frame_id") == (
        "mecanum_sim/base_footprint"
    )
    assert mecanum.pop("output_odom_frame_id") == "mecanum_sim/odom"
    assert mecanum == pytest.approx(
        {
            "source_to_geometry_center_m": 0.0,
            "output_odom_origin_x_m": -1.0,
            "output_odom_origin_y_m": 4.0,
            "output_odom_origin_yaw_rad": -0.5,
        }
    )
    with pytest.raises(ValueError, match="finite"):
        module.simulation_odometry_parameters(
            "ackermann_sim", "ackermann", float("nan"), 0.0, 0.0
        )
    with pytest.raises(ValueError, match="relative ROS namespace"):
        module.simulation_odometry_parameters(
            "/ackermann_sim", "ackermann", 0.0, 0.0, 0.0
        )


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


@pytest.mark.parametrize(
    "selection,vehicle_type,expected",
    [
        ("auto", "ackermann", "ackermann"),
        ("auto", "mecanum", "mecanum"),
        ("ackermann", "ackermann", "ackermann"),
        ("mecanum", "mecanum", "mecanum"),
        ("none", "ackermann", None),
        ("none", "mecanum", None),
    ],
)
def test_tracking_selection_defaults_to_the_profile_vehicle_type(
    selection, vehicle_type, expected
):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    assert module.resolve_tracking_controller(selection, vehicle_type) == expected


def test_controller_odometry_is_hidden_behind_common_geometry_center_adapter():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    assert module.controller_odometry_remappings("fleet/robot_two") == [
        (
            "/fleet/robot_two/vehicle/odometry",
            "/fleet/robot_two/vehicle/_controller_odometry",
        ),
        (
            "/fleet/robot_two/vehicle/tf_odometry",
            "/fleet/robot_two/vehicle/_controller_tf_odometry",
        ),
    ]
    assert module.odometry_adapter_parameters("ackermann_1", "ackermann") == {
        "source_to_geometry_center_m": 0.0675,
        "geometry_center_frame_id": "ackermann_1/base_footprint",
        "output_odom_frame_id": "ackermann_1/odom",
    }
    assert module.odometry_adapter_parameters("mecanum_2", "mecanum") == {
        "source_to_geometry_center_m": 0.0,
        "geometry_center_frame_id": "mecanum_2/base_footprint",
        "output_odom_frame_id": "mecanum_2/odom",
    }

    launch_source = (share / "launch" / "vehicle.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="vehicle_odometry"' in launch_source
    assert '"vehicle odometry adapter"' in launch_source


@pytest.mark.parametrize(
    "selection,vehicle_type,error",
    [
        ("mecanum", "ackermann", "must be auto, none, or match"),
        ("ackermann", "mecanum", "must be auto, none, or match"),
        ("enabled", "mecanum", "must be auto, none, or match"),
        ("auto", "tracked", "unsupported vehicle_type"),
    ],
)
def test_tracking_selection_rejects_mismatches(selection, vehicle_type, error):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    module = vehicle_launch_module(share)
    with pytest.raises(ValueError, match=error):
        module.resolve_tracking_controller(selection, vehicle_type)


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
        "driven_wheel_angular_speed_limit_rad_s": 37.69911184307752,
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
        "driven_wheel_angular_speed_limit_rad_s": 37.69911184307752,
    }


def test_simulation_and_physical_launches_share_tracker_selection_contract():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    physical = vehicle_launch_module(share)
    simulation = load_launch(share / "launch" / "simulation.launch.py")

    for vehicle_type in ("ackermann", "mecanum"):
        for algorithm in ("adrc", "mpc"):
            assert simulation.tracking_parameters(
                vehicle_type, algorithm
            ) == physical.tracking_parameters(vehicle_type, algorithm)
        for selection in ("auto", "none", vehicle_type):
            assert simulation.resolve_tracking_controller(
                selection, vehicle_type
            ) == physical.resolve_tracking_controller(selection, vehicle_type)
    with pytest.raises(ValueError, match="must be auto, none, or match"):
        simulation.resolve_tracking_controller("mecanum", "ackermann")

    tracking_share = Path(get_package_share_directory("mentor_pi_tracking"))
    for algorithm in ("adrc", "mpc"):
        with open(
            tracking_share / "config" / f"{algorithm}.yaml",
            encoding="utf-8",
        ) as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
        assert (
            parameters["driven_wheel_angular_speed_limit_rad_s"]
            == 37.69911184307752
        )


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


def test_tracking_contract_accepts_the_generated_vehicle_namespace():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    source = (share / "launch" / "vehicle.launch.py").read_text(encoding="utf-8")
    assert "trajectory tracking uses the fixed /mentor_pi API" not in source
    assert "namespace=robot_name" in source
