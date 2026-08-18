#!/usr/bin/env python3

import hashlib
import math
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest


_EXPECTED = {
    "ackermann/meshes/base_link.stl": (
        830384,
        16606,
        "5673bcdcb55d1631f71d567687db05ba7daf78c96b3389ec973a74a75296347c",
        (-0.103781, -0.050426, -0.043),
        (0.109563, 0.050426, 0.057511),
    ),
    "ackermann/meshes/cam_link.stl": (
        4752784,
        95054,
        "10698ec8ab4ca4126243feb947c8e1133b515556f0b2f3f17204deb2128c8f37",
        (-0.026777, -0.044217, -0.025831),
        (0.013735, 0.045583, 0.000353),
    ),
    "ackermann/meshes/lidar_link.stl": (
        911284,
        18224,
        "c902e4018546df554096c5d063b5d831646384d04a7a6d4962e8a937c2c90f1b",
        (-0.026803, -0.018745, -0.035),
        (0.026934, 0.027787, 0.0),
    ),
    "ackermann/meshes/wheel_left_front_link.stl": (
        3171784,
        63434,
        "33f6b38472ac44a39b9fb14a050c1dd460d14b94abfd05c82033062bf9844911",
        (-0.032346, -0.016076, -0.032348),
        (0.032348, 0.009317, 0.032346),
    ),
    "ackermann/meshes/wheel_left_rear_link.stl": (
        3242784,
        64854,
        "dd9f2a8e4f85a4f86eddd296b3d5a58eb3d43b82ff0ef65d597ed389a60a444d",
        (-0.032346, -0.019214, -0.032348),
        (0.032348, 0.009317, 0.032346),
    ),
    "ackermann/meshes/wheel_right_front_link.stl": (
        3190784,
        63814,
        "1f08a58badc36baeb4286e9eae04d9856518eb065c6a092bb48130eaf7a57dc5",
        (-0.032349, -0.009317, -0.032345),
        (0.032345, 0.016076, 0.032349),
    ),
    "ackermann/meshes/wheel_right_rear_link.stl": (
        3265684,
        65312,
        "301cd8642c44dedf3e14e50652b96ebab04d7dc1d38bab478e715fc2d67e7c6c",
        (-0.032349, -0.009317, -0.032345),
        (0.032345, 0.019214, 0.032349),
    ),
    "mecanum/meshes/base_link.stl": (
        696484,
        13928,
        "52b2819186ff186a49b9a3d3cb9e9943214ecffd773a56ae6c41f898cf6a2b37",
        (-0.103002, -0.05, -0.03485),
        (0.108942, 0.05, 0.057501),
    ),
    "mecanum/meshes/cam_link.stl": (
        4717884,
        94356,
        "4bf9e75f7d7070db2263b978cbb34b159f981f6db57a50a64b64547c8835292e",
        (-0.026777, -0.044217, -0.025831),
        (0.013735, 0.045583, 0.000353),
    ),
    "mecanum/meshes/lidar_link.stl": (
        876284,
        17524,
        "93c5329be97117f2b2f0196586627730e48798565415bcdcd7be4799ffb7047d",
        (-0.026803, -0.018745, -0.035),
        (0.026934, 0.027787, 0.0),
    ),
    "mecanum/meshes/wheel_left_front_link.stl": (
        10168884,
        203376,
        "e44b584d2f6945443527811cad9aabba4e27f8851b711d6d55420650ae03d38b",
        (-0.03232, -0.024624, -0.032345),
        (0.032332, 0.009776, 0.032331),
    ),
    "mecanum/meshes/wheel_left_rear_link.stl": (
        10206484,
        204128,
        "a0624c1830906c7b8299a7af09a2fceff7e8fb906ca6549dfd03825891eedc21",
        (-0.032302, -0.024624, -0.032339),
        (0.032338, 0.009776, 0.032338),
    ),
    "mecanum/meshes/wheel_right_front_link.stl": (
        10252084,
        205040,
        "2e2ba3bd1a423b14a643fe31f58964b703482fbec2ea8d2b58eefe66ee7a5bf3",
        (-0.03232, -0.009776, -0.032345),
        (0.032321, 0.024624, 0.032331),
    ),
    "mecanum/meshes/wheel_right_rear_link.stl": (
        10193884,
        203876,
        "11fd9a6e3c1cbbe139a458ef7e3792b45439b22625e600e2a35f8ebd764ebf88",
        (-0.032332, -0.009776, -0.032345),
        (0.03232, 0.024624, 0.032331),
    ),
}

_WHEELS = (
    "wheel_left_front",
    "wheel_right_front",
    "wheel_left_rear",
    "wheel_right_rear",
)

_WHEEL_ORIGINS = {
    "ackermann": {
        "wheel_left_front_joint": (0.0675, 0.070, 0.0325),
        "wheel_right_front_joint": (0.0675, -0.070, 0.0325),
        "wheel_left_rear_joint": (-0.0675, 0.070, 0.0325),
        "wheel_right_rear_joint": (-0.0675, -0.070, 0.0325),
    },
    "mecanum": {
        "wheel_left_front_joint": (0.065812864, 0.074187136, 0.0325),
        "wheel_right_front_joint": (0.065812864, -0.074187136, 0.0325),
        "wheel_left_rear_joint": (-0.065812864, 0.074187136, 0.0325),
        "wheel_right_rear_joint": (-0.065812864, -0.074187136, 0.0325),
    },
}

_BASE_VISUAL_ORIGINS = {
    "ackermann": {
        "base_link.stl": (0.0, 0.0, 0.070),
        "cam_link.stl": (0.064015, -0.00013463, 0.121155),
        "lidar_link.stl": (-0.0096019, -0.00008533, 0.162501),
    },
    "mecanum": {
        "base_link.stl": (0.0, 0.0, 0.050),
        "cam_link.stl": (0.064015, -0.00013463, 0.101155),
        "lidar_link.stl": (-0.0096019, -0.00008533, 0.142501),
    },
}


def _render(share, vehicle, simulation):
    filename = "simulation.urdf.xacro" if simulation else "mentor_pi.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            str(share / "config" / vehicle / filename),
            "robot_name:=mesh_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _mesh_path(share, uri):
    prefix = "package://mentor_pi_hardwares/"
    assert uri.startswith(prefix)
    return share / uri.removeprefix(prefix)


def _stl_properties(path):
    data = path.read_bytes()
    assert len(data) >= 84
    face_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * face_count

    minima = [math.inf, math.inf, math.inf]
    maxima = [-math.inf, -math.inf, -math.inf]
    for face in range(face_count):
        values = struct.unpack_from("<12f", data, 84 + 50 * face)
        assert all(math.isfinite(value) for value in values)
        for vertex in (values[3:6], values[6:9], values[9:12]):
            minima = [min(old, new) for old, new in zip(minima, vertex)]
            maxima = [max(old, new) for old, new in zip(maxima, vertex)]

    return (
        len(data),
        face_count,
        hashlib.sha256(data).hexdigest(),
        tuple(minima),
        tuple(maxima),
    )


def test_installed_meshes_are_exact_mpc_assets():
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    config = share / "config"
    installed = {
        str(path.relative_to(config))
        for path in config.glob("*/meshes/*.stl")
    }
    assert installed == set(_EXPECTED)

    for relative, expected in _EXPECTED.items():
        actual = _stl_properties(config / relative)
        assert actual[:3] == expected[:3]
        assert actual[3] == pytest.approx(expected[3], abs=1.0e-6)
        assert actual[4] == pytest.approx(expected[4], abs=1.0e-6)


@pytest.mark.parametrize("vehicle", ["ackermann", "mecanum"])
@pytest.mark.parametrize("simulation", [False, True])
def test_descriptions_reference_every_retained_mesh(vehicle, simulation):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    root = _render(share, vehicle, simulation)
    expected = {
        f"package://mentor_pi_hardwares/config/{relative}"
        for relative in _EXPECTED
        if relative.startswith(f"{vehicle}/")
    }
    referenced = {
        mesh.attrib["filename"]
        for mesh in root.findall(".//visual/geometry/mesh")
    }
    assert referenced == expected
    assert all(_mesh_path(share, uri).is_file() for uri in referenced)
    assert root.findall(".//collision/geometry/mesh") == []
    assert root.find(".//collision/geometry/box") is not None
    assert len(root.findall(".//collision/geometry/cylinder")) == 4

    base = root.find("./link[@name='mesh_test/base_link']")
    assert base is not None
    base_visuals = base.findall("./visual")
    assert len(base_visuals) == 3
    for visual in base_visuals:
        mesh = visual.find("./geometry/mesh")
        origin = visual.find("./origin")
        assert mesh is not None
        assert origin is not None
        filename = Path(mesh.attrib["filename"]).name
        actual_xyz = tuple(
            float(value) for value in origin.attrib["xyz"].split()
        )
        actual_rpy = tuple(
            float(value) for value in origin.attrib["rpy"].split()
        )
        assert actual_xyz == pytest.approx(
            _BASE_VISUAL_ORIGINS[vehicle][filename], abs=1.0e-12
        )
        assert actual_rpy == pytest.approx(
            (-math.pi / 2.0, 0.0, 0.0), abs=1.0e-12
        )
    for wheel in _WHEELS:
        link = root.find(f"./link[@name='mesh_test/{wheel}_link']")
        assert link is not None
        assert len(link.findall("./visual/geometry/mesh")) == 1
        assert link.find("./visual/origin") is None


@pytest.mark.parametrize("vehicle", ["ackermann", "mecanum"])
@pytest.mark.parametrize("simulation", [False, True])
def test_mesh_wheel_frames_match_controller_geometry(vehicle, simulation):
    share = Path(get_package_share_directory("mentor_pi_hardwares"))
    root = _render(share, vehicle, simulation)
    for joint_name, expected in _WHEEL_ORIGINS[vehicle].items():
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        actual = tuple(
            float(value)
            for value in joint.find("origin").attrib["xyz"].split()
        )
        assert actual == pytest.approx(expected, abs=1.0e-12)
