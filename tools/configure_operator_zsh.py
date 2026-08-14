#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import shutil
import tempfile
import time


BEGIN = "# >>> mentor-pi onboard setup >>>"
END = "# <<< mentor-pi onboard setup <<<"
VARIABLES = (
    "MENTOR_PI_TYPE",
    "MENTOR_PI_NAME",
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "RMW_IMPLEMENTATION",
    "ROS_DISCOVERY_SERVER",
)
ARGCOMPLETE_LINES = (
    "# argcomplete for ros2 & colcon",
    'eval "$(register-python-argcomplete3 ros2)"',
    'eval "$(register-python-argcomplete3 colcon)"',
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zshrc", type=Path, required=True)
    parser.add_argument("--workspace-setup", type=Path, required=True)
    parser.add_argument("--vehicle-type", choices=("mecanum", "ackermann"), required=True)
    parser.add_argument("--vehicle-name", required=True)
    args = parser.parse_args()
    zshrc = args.zshrc
    workspace = str(args.workspace_setup)
    if not args.workspace_setup.is_absolute():
        parser.error("--workspace-setup must be absolute")
    if any(character in workspace for character in ('\n', '\r', '"')):
        parser.error("--workspace-setup contains unsupported characters")
    if not re.fullmatch(
        r"[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*",
        args.vehicle_name,
    ):
        parser.error("--vehicle-name must be a valid relative ROS namespace")
    original = zshrc.read_text(encoding="utf-8") if zshrc.exists() else ""
    mode = zshrc.stat().st_mode & 0o777 if zshrc.exists() else 0o644

    lines = []
    managed = False
    assignment = re.compile(
        r"^\s*#?\s*(?:export\s+)?(" + "|".join(VARIABLES) + r")\s*="
    )
    for line in original.splitlines():
        if line == BEGIN:
            managed = True
            continue
        if line == END:
            managed = False
            continue
        if managed or assignment.match(line):
            continue
        if line.strip() in ARGCOMPLETE_LINES:
            continue
        if "/opt/ros/humble/setup." in line or workspace in line:
            continue
        lines.append(line)
    while lines and not lines[-1].strip():
        lines.pop()
    block = [
        BEGIN,
        "source /opt/ros/humble/setup.zsh",
        f'[[ -f "{workspace}" ]] && source "{workspace}"',
        "",
        *ARGCOMPLETE_LINES,
        "",
        f"export MENTOR_PI_TYPE={args.vehicle_type}",
        f"export MENTOR_PI_NAME={args.vehicle_name}",
        "export ROS_DOMAIN_ID=42",
        "export ROS_LOCALHOST_ONLY=0",
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "export ROS_DISCOVERY_SERVER=192.168.2.191:11811",
        END,
    ]
    rendered = "\n".join(lines + ([""] if lines else []) + block) + "\n"
    if rendered == original:
        return
    zshrc.parent.mkdir(parents=True, exist_ok=True)
    if zshrc.exists():
        backup = zshrc.with_name(f"{zshrc.name}.mentor-pi-backup-{time.time_ns()}")
        shutil.copy2(zshrc, backup)
    descriptor, temporary = tempfile.mkstemp(dir=zshrc.parent, prefix=".zshrc.")
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(rendered)
        os.chmod(temporary, mode)
        os.replace(temporary, zshrc)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


if __name__ == "__main__":
    main()
