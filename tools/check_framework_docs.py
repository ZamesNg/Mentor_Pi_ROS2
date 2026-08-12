#!/usr/bin/env python3

"""Check first-party Markdown links, traceability, and tutorial layout."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
FRAMEWORK = ROOT / "docs/framework"
REQUIREMENTS = FRAMEWORK / "requirements.md"
VERIFICATION = FRAMEWORK / "verification.md"
LEGACY_AUDIT = FRAMEWORK / "legacy-audit.md"
TUTORIALS = ROOT / "docs/tutorials"
TRACKS = {
    "host": (
        "01-prerequisites-and-safety.md",
        "02-firmware-setup-and-build.md",
        "03-passive-hardware-checks.md",
        "04-firmware-flash.md",
        "05-ros-environment.md",
        "06-ros-apps-build-and-test.md",
        "07-connect-and-run.md",
        "08-evidence-and-qualification.md",
    ),
    "onboard": (
        "01-prerequisites-and-safety.md",
        "02-firmware-setup-and-build.md",
        "03-passive-checks-and-flash.md",
        "04-agent-build.md",
        "05-agent-service-installation.md",
        "06-ros-apps-build-and-test.md",
        "07-integrated-runtime-and-recovery.md",
        "08-evidence-and-qualification.md",
    ),
}
EXCLUDED_PARTS = {
    ".git", "build", "generated", "install", "log", "third_party",
    "__pycache__",
}
LINK = re.compile(r"!?\[[^\]\n]*\]\((?P<target><[^>\n]+>|[^)\n]+)\)")
REQ_ID = re.compile(
    r"^(?:SCOPE|PLAT|HW|HOST|TRN|ROS|CTRL|RT|SAFE|QUAL|RES|MEM|PERF|SOAK|REC)-\d{3}$"
)
VER_ID = re.compile(r"VER-[A-Z0-9]+(?:-[A-Z0-9]+)+")
AUDIT_ID = re.compile(r"^(?:ROS|HW)-[A-Z]+-\d{2}$")


def markdown_files() -> list[Path]:
    files: list[Path] = []
    for path in ROOT.rglob("*.md"):
        relative = path.relative_to(ROOT)
        if any(part in EXCLUDED_PARTS for part in relative.parts):
            continue
        if relative.parts[:2] == ("docs", "reference"):
            continue
        files.append(path)
    return sorted(files)


def check_links(files: list[Path]) -> tuple[list[str], int]:
    errors: list[str] = []
    count = 0
    for source in files:
        text = source.read_text(encoding="utf-8")
        for match in LINK.finditer(text):
            target = match.group("target").strip()
            if target.startswith("<") and target.endswith(">"):
                raw = target[1:-1]
            else:
                raw = target.split(maxsplit=1)[0]
            parsed = urlsplit(raw)
            if parsed.scheme or raw.startswith("#"):
                continue
            count += 1
            destination = (source.parent / unquote(parsed.path)).resolve()
            try:
                destination.relative_to(ROOT.resolve())
            except ValueError:
                errors.append(f"{source}: link escapes repository: {raw}")
                continue
            reference_root = (ROOT / "docs/reference").resolve()
            if destination == reference_root or reference_root in destination.parents:
                # Raw legacy evidence is intentionally ignored and may be absent.
                continue
            if not destination.exists():
                errors.append(f"{source}: broken relative link: {raw}")
    return errors, count


def table_rows(path: Path) -> list[list[str]]:
    rows: list[list[str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip().strip("`") for cell in line.strip("|").split("|")]
        if cells and not all(set(cell) <= {"-", ":"} for cell in cells):
            rows.append(cells)
    return rows


def check_traceability() -> tuple[list[str], dict[str, int]]:
    errors: list[str] = []
    requirement_rows = {
        row[0]: row for row in table_rows(REQUIREMENTS)
        if row and REQ_ID.fullmatch(row[0])
    }
    mandatory = {
        identifier for identifier, row in requirement_rows.items()
        if len(row) > 1 and re.search(r"\bshall\b", row[1], re.IGNORECASE)
    }
    verification_rows = table_rows(VERIFICATION)
    definitions = {
        row[0] for row in verification_rows
        if row and VER_ID.fullmatch(row[0])
    }
    definitions.update(
        match.group(0)
        for line in VERIFICATION.read_text(encoding="utf-8").splitlines()
        for match in [VER_ID.search(line)]
        if line.startswith("### `VER-") and match is not None
    )
    mappings: dict[str, set[str]] = {}
    in_mapping = False
    for line in VERIFICATION.read_text(encoding="utf-8").splitlines():
        if line.strip() == "## Mandatory requirement traceability":
            in_mapping = True
            continue
        if in_mapping and line.startswith("## "):
            break
        if not in_mapping or not line.startswith("|"):
            continue
        cells = [cell.strip().strip("`") for cell in line.strip("|").split("|")]
        if cells and REQ_ID.fullmatch(cells[0]):
            mappings[cells[0]] = set(VER_ID.findall(" ".join(cells[1:])))
    for identifier in sorted(mandatory):
        if not mappings.get(identifier):
            errors.append(f"{VERIFICATION}: mandatory requirement {identifier} is unmapped")
    for identifier, cases in sorted(mappings.items()):
        if identifier not in requirement_rows:
            errors.append(f"{VERIFICATION}: mapping has unknown requirement {identifier}")
        for case in sorted(cases - definitions):
            errors.append(f"{VERIFICATION}: mapping references undefined {case}")

    audit_rows = {
        row[0]: row for row in table_rows(LEGACY_AUDIT)
        if row and AUDIT_ID.fullmatch(row[0])
    }
    for identifier, row in sorted(audit_rows.items()):
        cases = set(VER_ID.findall(" ".join(row[1:])))
        if not cases:
            errors.append(f"{LEGACY_AUDIT}: audit row {identifier} is unmapped")
        for case in sorted(cases - definitions):
            errors.append(f"{LEGACY_AUDIT}: audit row {identifier} references undefined {case}")
    return errors, {
        "requirements": len(requirement_rows),
        "mandatory": len(mandatory),
        "verification": len(definitions),
        "audit": len(audit_rows),
    }


def check_tutorials() -> list[str]:
    errors: list[str] = []
    actual_tracks = sorted(path.name for path in TUTORIALS.iterdir() if path.is_dir())
    if actual_tracks != sorted(TRACKS):
        errors.append(f"{TUTORIALS}: expected only host and onboard tracks")
    if list(TUTORIALS.glob("*.md")):
        errors.append(f"{TUTORIALS}: tutorial Markdown must be inside a track")
    for track, expected in TRACKS.items():
        directory = TUTORIALS / track
        actual = tuple(sorted(path.name for path in directory.glob("*.md")))
        if actual != expected:
            errors.append(f"{directory}: tutorial filenames do not match the 01-08 contract")
        for index, filename in enumerate(expected, start=1):
            path = directory / filename
            if path.is_file() and not path.read_text(encoding="utf-8").startswith(
                f"# {index:02d}"
            ):
                errors.append(f"{path}: title must start with '# {index:02d}'")

    host_text = "\n".join(
        (TUTORIALS / "host" / name).read_text(encoding="utf-8")
        for name in TRACKS["host"]
    )
    onboard_text = "\n".join(
        (TUTORIALS / "onboard" / name).read_text(encoding="utf-8")
        for name in TRACKS["onboard"]
    )
    required_markers = {
        "host": (
            "VS Code Dev Container", "make -C firmware build",
            "make -C firmware flash", "make -C ros2_ws test",
            "current-limited supply",
        ),
        "onboard": (
            "Ubuntu 22.04", "make -C firmware build",
            "make -C micro_ros_agent build", "install-service",
            "mentor-pi-agent.service", "make -C ros2_ws test",
            "/var/log/mentor-pi/actions", "current-limited supply",
        ),
    }
    for track, text in (("host", host_text), ("onboard", onboard_text)):
        for marker in required_markers[track]:
            if marker not in text:
                errors.append(f"{TUTORIALS / track}: missing required marker {marker!r}")
    return errors


def main() -> int:
    required = (REQUIREMENTS, VERIFICATION, LEGACY_AUDIT)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        for path in missing:
            print(f"missing documentation input: {path}", file=sys.stderr)
        return 1
    files = markdown_files()
    link_errors, links = check_links(files)
    trace_errors, counts = check_traceability()
    tutorial_errors = check_tutorials()
    errors = link_errors + trace_errors + tutorial_errors
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        print(f"documentation checks failed with {len(errors)} error(s)", file=sys.stderr)
        return 1
    print(
        "documentation checks passed: "
        f"{len(files)} Markdown files, {links} relative links, "
        f"{counts['mandatory']}/{counts['requirements']} mandatory requirements, "
        f"{counts['verification']} verification cases, {counts['audit']} audit rows, "
        "16 ordered tutorials"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
