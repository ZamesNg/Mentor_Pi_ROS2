#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import tempfile

import yaml


NAME_PATTERN = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)


def write_yaml(destination, content):
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        dir=destination.parent, prefix=f".{destination.name}."
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(content, stream, sort_keys=False)
        os.chmod(temporary, 0o644)
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", choices=("mecanum", "ackermann"), required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    args = parser.parse_args()
    if NAME_PATTERN.fullmatch(args.name) is None:
        parser.error("name must be a valid relative ROS namespace")

    source_directory = args.package_root / "config" / args.type
    hardware_source = source_directory / "hardware.yaml"
    controllers_source = source_directory / "controllers.yaml"
    destination_directory = args.package_root / "config" / "generated"
    vehicle_destination = destination_directory / "vehicle.yaml"
    controllers_destination = destination_directory / "controllers.yaml"
    if not hardware_source.is_file():
        parser.error(f"hardware profile is missing: {hardware_source}")
    if not controllers_source.is_file():
        parser.error(f"controller profile is missing: {controllers_source}")
    with hardware_source.open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or not isinstance(profile.get("vehicle"), dict):
        parser.error("hardware profile has an invalid schema")
    if profile["vehicle"].get("vehicle_type") != args.type:
        parser.error("hardware profile type does not match its directory")
    profile["vehicle"]["robot_name"] = args.name

    with controllers_source.open(encoding="utf-8") as stream:
        controllers = yaml.safe_load(stream)
    if not isinstance(controllers, dict):
        parser.error("controller profile has an invalid schema")
    controller_key = "/**/vehicle"
    try:
        controller_parameters = controllers[controller_key]["ros__parameters"]
        base_frame = controller_parameters["base_frame_id"]
        odom_frame = controller_parameters["odom_frame_id"]
    except (KeyError, TypeError):
        parser.error("controller profile is missing its frame configuration")
    controller_parameters["base_frame_id"] = f"{args.name}/{base_frame}"
    controller_parameters["odom_frame_id"] = f"{args.name}/{odom_frame}"

    write_yaml(vehicle_destination, profile)
    write_yaml(controllers_destination, controllers)
    print(vehicle_destination)


if __name__ == "__main__":
    main()
