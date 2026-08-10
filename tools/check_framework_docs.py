#!/usr/bin/env python3

"""Validate first-party Markdown links and RRCLite traceability tables.

This is a build-time documentation checker. It is not installed and is never
part of the host or MCU runtime data path.
"""

from __future__ import annotations

import re
import sys
import unicodedata
from pathlib import Path
from urllib.parse import unquote, urlsplit


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS_PATH = PROJECT_ROOT / "docs/framework/requirements.md"
LEGACY_AUDIT_PATH = PROJECT_ROOT / "docs/framework/legacy-audit.md"
VERIFICATION_PATH = PROJECT_ROOT / "docs/framework/verification.md"
TUTORIAL_DIRECTORY = PROJECT_ROOT / "docs/tutorials"
TUTORIAL_FILENAMES = (
    "01-prepare-ubuntu-development-host.md",
    "02-build-and-flash-default-pid-firmware.md",
    "03-build-and-run-humble-host.md",
    "04-run-passive-board-bringup.md",
    "05-characterize-board-hardware.md",
    "06-ros2-cli-hardware-checkout.md",
    "07-run-stress-soak-and-release-gates.md",
    "08-run-mentor-pi-hardwares.md",
)
TUTORIAL_PATHS = tuple(
    TUTORIAL_DIRECTORY / filename for filename in TUTORIAL_FILENAMES
)
RETIRED_DOCUMENT_FILENAMES = (
    "board-arrival-bringup-checklist.md",
    "ci-and-hardware-gates.md",
    "flashing-and-first-bringup.md",
    "host-preparation-and-handoff.md",
    "ros2-cli-examples.md",
    "qualification-evidence-ledger.md",
)

EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    "__pycache__",
    "build",
    "generated",
    "install",
    "log",
    "third_party",
    "tmp",
}
EXCLUDED_DIRECTORY_PATHS = {
    (PROJECT_ROOT / "docs/reference").resolve(),
}

INLINE_LINK_PATTERN = re.compile(
    r"!?\[[^\]\n]*\]\((?P<destination><[^>\n]+>|[^)\n]+)\)"
)
REFERENCE_DEFINITION_PATTERN = re.compile(
    r"^\s*\[[^\]]+\]:\s*(?P<destination><[^>\n]+>|\S+)"
)
HEADING_PATTERN = re.compile(r"^\s{0,3}(?P<level>#{1,6})\s+(?P<text>.+?)\s*$")
REQUIREMENT_ID_PATTERN = re.compile(
    r"^(?:SCOPE|PLAT|HW|HOST|TRN|ROS|CTRL|RT|SAFE|QUAL|RES|MEM|PERF|SOAK|REC)-\d{3}$"
)
AUDIT_ID_PATTERN = re.compile(r"^(?:ROS|HW)-[A-Z]+-\d{2}$")
VERIFICATION_ID_PATTERN = re.compile(r"VER-[A-Z0-9]+(?:-[A-Z0-9]+)+")
TABLE_FIRST_CELL_PATTERN = re.compile(
    r"^\|\s*`?(?P<identifier>[A-Z][A-Z0-9-]+)`?\s*\|(?P<rest>.*)$"
)
VERIFICATION_HEADING_PATTERN = re.compile(
    r"^#{2,6}\s+`(?P<identifier>VER-[A-Z0-9]+(?:-[A-Z0-9]+)+)`"
)


def is_excluded(path: Path) -> bool:
    """Return whether path belongs to generated, vendor, or evidence input."""
    resolved = path.resolve()
    if any(resolved == prefix or prefix in resolved.parents
           for prefix in EXCLUDED_DIRECTORY_PATHS):
        return True
    try:
        relative = resolved.relative_to(PROJECT_ROOT)
    except ValueError:
        return True
    return any(part in EXCLUDED_DIRECTORY_NAMES for part in relative.parts)


def markdown_files() -> list[Path]:
    """Return deterministic first-party Markdown inputs."""
    return sorted(
        path
        for path in PROJECT_ROOT.rglob("*.md")
        if path.is_file() and not is_excluded(path)
    )


def remove_inline_markup(text: str) -> str:
    """Reduce a Markdown heading to the visible text used by GitHub slugs."""
    text = re.sub(r"<[^>]*>", "", text)
    text = re.sub(r"!\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", text)
    text = text.replace("`", "").replace("*", "").replace("~", "")
    return text


def github_slug(text: str) -> str:
    """Return the GitHub-compatible base anchor for a heading."""
    visible = remove_inline_markup(text).strip().lower()
    characters: list[str] = []
    for character in visible:
        category = unicodedata.category(character)
        if character.isspace():
            characters.append("-")
        elif character in {"-", "_"} or category[0] in {"L", "N"}:
            characters.append(character)
    return "".join(characters)


def markdown_anchors(path: Path) -> set[str]:
    """Extract GitHub-style anchors, including duplicate-heading suffixes."""
    anchors: set[str] = set()
    base_counts: dict[str, int] = {}
    in_fence = False
    fence_marker = ""
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.lstrip()
        if stripped.startswith(("```", "~~~")):
            marker = stripped[:3]
            if not in_fence:
                in_fence = True
                fence_marker = marker
            elif marker == fence_marker:
                in_fence = False
                fence_marker = ""
            continue
        if in_fence:
            continue
        match = HEADING_PATTERN.match(line)
        if match is None:
            continue
        base = github_slug(match.group("text"))
        if not base:
            continue
        duplicate_index = base_counts.get(base, 0)
        anchor = base if duplicate_index == 0 else f"{base}-{duplicate_index}"
        base_counts[base] = duplicate_index + 1
        anchors.add(anchor)
    return anchors


def normalized_destination(raw_destination: str) -> str:
    """Remove Markdown angle brackets and optional link titles."""
    destination = raw_destination.strip()
    if destination.startswith("<") and destination.endswith(">"):
        return destination[1:-1]
    # Relative paths containing spaces must use Markdown angle brackets. For
    # ordinary destinations, whitespace begins the optional quoted title.
    return destination.split(maxsplit=1)[0]


def validate_destination(
    source: Path,
    line_number: int,
    raw_destination: str,
    anchor_cache: dict[Path, set[str]],
) -> str | None:
    """Validate one relative Markdown destination and optional fragment."""
    destination = normalized_destination(raw_destination)
    if not destination:
        return f"{source}:{line_number}: empty Markdown link destination"

    parsed = urlsplit(destination)
    if parsed.scheme or parsed.netloc or destination.startswith("//"):
        return None
    if parsed.path.startswith("/"):
        # Site-root links are not filesystem-relative and are outside this
        # checker's stated contract.
        return None

    decoded_path = unquote(parsed.path)
    target = source if not decoded_path else source.parent / decoded_path
    target = target.resolve()
    try:
        target.relative_to(PROJECT_ROOT)
    except ValueError:
        return (
            f"{source}:{line_number}: relative link escapes the repository: "
            f"{destination}"
        )
    if any(target == prefix or prefix in target.parents
           for prefix in EXCLUDED_DIRECTORY_PATHS):
        # docs/reference is intentionally local, ignored by Git, and unavailable
        # on hosted CI. Its links are evidence citations rather than deliverable
        # links; validate them during the source audit that supplies that tree.
        return None
    if not target.exists():
        return f"{source}:{line_number}: missing relative link target: {destination}"

    fragment = unquote(parsed.fragment)
    if not fragment:
        return None
    if not target.is_file() or target.suffix.lower() != ".md":
        return (
            f"{source}:{line_number}: anchor targets a non-Markdown path: "
            f"{destination}"
        )
    if target not in anchor_cache:
        anchor_cache[target] = markdown_anchors(target)
    if fragment not in anchor_cache[target]:
        return (
            f"{source}:{line_number}: missing Markdown anchor "
            f"#{fragment} in {target.relative_to(PROJECT_ROOT)}"
        )
    return None


def validate_markdown_links(paths: list[Path]) -> tuple[list[str], int]:
    """Validate inline links and reference-definition destinations."""
    errors: list[str] = []
    checked = 0
    anchor_cache: dict[Path, set[str]] = {}
    for path in paths:
        in_fence = False
        fence_marker = ""
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            stripped = line.lstrip()
            if stripped.startswith(("```", "~~~")):
                marker = stripped[:3]
                if not in_fence:
                    in_fence = True
                    fence_marker = marker
                elif marker == fence_marker:
                    in_fence = False
                    fence_marker = ""
                continue
            if in_fence:
                continue

            destinations = [
                match.group("destination")
                for match in INLINE_LINK_PATTERN.finditer(line)
            ]
            reference = REFERENCE_DEFINITION_PATTERN.match(line)
            if reference is not None:
                destinations.append(reference.group("destination"))
            for destination in destinations:
                parsed = urlsplit(normalized_destination(destination))
                if parsed.scheme or parsed.netloc or destination.startswith("//"):
                    continue
                checked += 1
                error = validate_destination(
                    path, line_number, destination, anchor_cache
                )
                if error is not None:
                    errors.append(error)
    return errors, checked


def parse_table_rows(path: Path) -> list[tuple[int, str, str]]:
    """Return line number, first-cell ID, and remaining table cells."""
    rows: list[tuple[int, str, str]] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        match = TABLE_FIRST_CELL_PATTERN.match(line)
        if match is not None:
            rows.append((line_number, match.group("identifier"), match.group("rest")))
    return rows


def defined_verification_cases(text: str) -> tuple[set[str], list[str]]:
    """Extract cases defined by a table row or a dedicated heading."""
    cases: set[str] = set()
    errors: list[str] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        table_match = TABLE_FIRST_CELL_PATTERN.match(line)
        heading_match = VERIFICATION_HEADING_PATTERN.match(line)
        identifier = None
        if table_match is not None:
            candidate = table_match.group("identifier")
            if VERIFICATION_ID_PATTERN.fullmatch(candidate):
                identifier = candidate
        if heading_match is not None:
            identifier = heading_match.group("identifier")
        if identifier is None:
            continue
        if identifier in cases:
            errors.append(
                f"{VERIFICATION_PATH}:{line_number}: duplicate verification "
                f"definition {identifier}"
            )
        cases.add(identifier)
    return cases, errors


def validate_traceability() -> tuple[list[str], dict[str, int]]:
    """Check requirement and legacy-audit mappings against defined VER cases."""
    errors: list[str] = []
    requirement_rows = [
        row
        for row in parse_table_rows(REQUIREMENTS_PATH)
        if REQUIREMENT_ID_PATTERN.fullmatch(row[1])
    ]
    requirement_ids = {identifier for _, identifier, _ in requirement_rows}
    mandatory_ids = {
        identifier
        for _, identifier, rest in requirement_rows
        if re.search(r"\*\*shall(?: not)?\*\*", rest, re.IGNORECASE)
    }
    if len(requirement_ids) != len(requirement_rows):
        errors.append(f"{REQUIREMENTS_PATH}: duplicate requirement ID")

    verification_text = VERIFICATION_PATH.read_text(encoding="utf-8")
    defined_cases, definition_errors = defined_verification_cases(verification_text)
    errors.extend(definition_errors)

    requirement_mappings: dict[str, set[str]] = {}
    for line_number, identifier, rest in parse_table_rows(VERIFICATION_PATH):
        if not REQUIREMENT_ID_PATTERN.fullmatch(identifier):
            continue
        if identifier in requirement_mappings:
            errors.append(
                f"{VERIFICATION_PATH}:{line_number}: duplicate requirement "
                f"mapping {identifier}"
            )
        requirement_mappings[identifier] = set(
            VERIFICATION_ID_PATTERN.findall(rest)
        )

    for identifier in sorted(mandatory_ids - requirement_mappings.keys()):
        errors.append(
            f"{VERIFICATION_PATH}: mandatory requirement {identifier} has no mapping"
        )
    for identifier in sorted(requirement_mappings.keys() - requirement_ids):
        errors.append(
            f"{VERIFICATION_PATH}: mapping references unknown requirement {identifier}"
        )
    for identifier, cases in sorted(requirement_mappings.items()):
        if not cases:
            errors.append(
                f"{VERIFICATION_PATH}: requirement {identifier} maps to no VER case"
            )
        for case in sorted(cases - defined_cases):
            errors.append(
                f"{VERIFICATION_PATH}: requirement {identifier} references "
                f"undefined case {case}"
            )

    audit_rows = [
        row
        for row in parse_table_rows(LEGACY_AUDIT_PATH)
        if AUDIT_ID_PATTERN.fullmatch(row[1])
    ]
    audit_ids = {identifier for _, identifier, _ in audit_rows}
    if len(audit_ids) != len(audit_rows):
        errors.append(f"{LEGACY_AUDIT_PATH}: duplicate legacy audit ID")
    audit_references: set[str] = set()
    for line_number, identifier, rest in audit_rows:
        cases = set(VERIFICATION_ID_PATTERN.findall(rest))
        if not cases:
            errors.append(
                f"{LEGACY_AUDIT_PATH}:{line_number}: audit row {identifier} "
                "has no VER case"
            )
        audit_references.update(cases)
        for case in sorted(cases - defined_cases):
            errors.append(
                f"{LEGACY_AUDIT_PATH}:{line_number}: audit row {identifier} "
                f"references undefined case {case}"
            )

    mapped_cases = set().union(*requirement_mappings.values())
    referenced_cases = mapped_cases | audit_references
    for case in sorted(defined_cases - referenced_cases):
        errors.append(f"{VERIFICATION_PATH}: defined case {case} is unreferenced")

    counts = {
        "requirements": len(requirement_ids),
        "mandatory_requirements": len(mandatory_ids),
        "audit_rows": len(audit_ids),
        "verification_cases": len(defined_cases),
    }
    return errors, counts


def validate_tutorial_sequence(paths: list[Path]) -> list[str]:
    """Require one exact, navigable Docker-first 01--08 sequence."""
    errors: list[str] = []
    actual_names = tuple(path.name for path in sorted(TUTORIAL_DIRECTORY.glob("*.md")))
    if actual_names != TUTORIAL_FILENAMES:
        errors.append(
            f"{TUTORIAL_DIRECTORY}: expected tutorials "
            f"{', '.join(TUTORIAL_FILENAMES)}; found {', '.join(actual_names)}"
        )
    legacy_directories = [
        path for path in TUTORIAL_DIRECTORY.iterdir() if path.is_dir()
    ]
    for legacy_directory in legacy_directories:
        errors.append(f"{legacy_directory}: tutorial host-track directory is obsolete")

    for index, filename in enumerate(TUTORIAL_FILENAMES, start=1):
        path = TUTORIAL_DIRECTORY / filename
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        title = f"# Tutorial {index:02d}:"
        if title not in text:
            errors.append(f"{path}: missing exact title prefix {title!r}")
        if index >= 2 and "**Warning:**" not in text:
            errors.append(f"{path}: hardware-sensitive tutorial has no warning")
        if index == 1:
            if "There is no previous tutorial" not in text:
                errors.append(f"{path}: missing start-of-sequence marker")
        elif TUTORIAL_FILENAMES[index - 2] not in text:
            errors.append(f"{path}: missing previous-tutorial link")
        if index == len(TUTORIAL_FILENAMES):
            if "Next: none" not in text:
                errors.append(f"{path}: missing end-of-sequence marker")
        elif TUTORIAL_FILENAMES[index] not in text:
            errors.append(f"{path}: missing next-tutorial link")
        if 'cd "${HOME}/Mentor_Pi"' not in text:
            errors.append(f"{path}: missing exact repository command")
        for description, pattern in {
            "replacement placeholder": r"REPLACE_WITH",
            "domain placeholder": r"THE_SAME_ID",
            "repository lookup": r"matching repository root",
            "native ROS command": r"source\s+[^\n]*setup\.(?:bash|zsh)",
            "backward command delegation": (
                r"(?:repeat|exactly as in|commands? from) Tutorials? [0-9]"
            ),
        }.items():
            if re.search(pattern, text, flags=re.IGNORECASE):
                errors.append(f"{path}: contains {description}")

    required_actions = {
        TUTORIAL_FILENAMES[0]: ("make doctor", "make setup"),
        TUTORIAL_FILENAMES[1]: (
            "make firmware",
            "make serial-setup",
            "make flash",
        ),
        TUTORIAL_FILENAMES[2]: (
            "make host",
            "make start",
            "make shell",
        ),
        TUTORIAL_FILENAMES[3]: (
            "make passive-check",
            "make peripheral-smoke",
        ),
        TUTORIAL_FILENAMES[4]: ("make characterize-board",),
        TUTORIAL_FILENAMES[5]: (
            "make start",
            "make shell",
            "controller.launch.py",
            "/mentor_pi/motors/command",
            "/mentor_pi/motors/set_pid",
        ),
        TUTORIAL_FILENAMES[6]: (
            "make release-software-gates", "make release-onboard-gates",
            "make qualification-preflight",
            "make campaign-load",
            "make campaign-soak",
            "make campaign-recovery",
        ),
        TUTORIAL_FILENAMES[7]: (
            "make host",
            "make start-mecanum",
            "make start-ackermann",
            "make start-hardware",
            "make shell",
        ),
    }
    for filename, actions in required_actions.items():
        path = TUTORIAL_DIRECTORY / filename
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        for action in actions:
            if action not in text:
                errors.append(f"{path}: missing one-line action {action!r}")

    for filename in RETIRED_DOCUMENT_FILENAMES:
        retired_matches = [
            path for path in PROJECT_ROOT.rglob(filename) if not is_excluded(path)
        ]
        for retired_path in retired_matches:
            errors.append(f"retired documentation still exists: {retired_path}")
        for source in paths:
            if filename in source.read_text(encoding="utf-8"):
                errors.append(
                    f"{source}: stale reference to retired document {filename}"
                )
    return errors


def validate_tutorial_feature_paths() -> tuple[list[str], int]:
    """Require a safe executable tutorial path for each retained feature."""
    errors: list[str] = []
    tutorials = " ".join(
        " ".join(path.read_text(encoding="utf-8").split())
        for path in TUTORIAL_PATHS
        if path.is_file()
    )

    safety_markers = {
        "motor-power passive fixture": "Motor power is disconnected",
        "PWM/bus-servo passive fixture": "PWM and bus servos unplugged",
        "probe-free passive IMU procedure": "six stationary board orientations",
        "first-board passive characterization": "CHARACTERIZATION PASS",
        "runtime acknowledgement": "PID_FIRMWARE_ACTUATORS_PREPARED",
        "guarded powered checkout": "raise or equivalently",
        "current-limited powered fixture": "current-limited supply",
        "reachable powered stop": "physical motor-power stop reachable",
        "automatic boot control": "make flash",
    }
    for description, marker in safety_markers.items():
        if marker not in tutorials:
            errors.append(f"{TUTORIAL_DIRECTORY}: missing {description} marker")

    feature_markers = {
        "four motors": (
            "/mentor_pi/motors/command",
            "update_mask: 15",
            "independent lease",
        ),
        "four PWM servos": (
            "/mentor_pi/pwm_servos/command",
            "500--2500 microseconds",
            "-100 through +100 us",
        ),
        "up to sixteen bus servos": (
            "/mentor_pi/bus_servos/get_state",
            "/mentor_pi/bus_servos/configure",
            "/mentor_pi/bus_servos/stop",
        ),
        "two host LEDs and heartbeat LED3": (
            "/mentor_pi/leds/command",
            "LED3",
        ),
        "buzzer": (
            "/mentor_pi/buzzer/command",
            "low-battery alarm has priority",
        ),
        "host RGB and MCU status RGB": (
            "/mentor_pi/rgb/command",
            "Only RGB1 is host-controlled",
            "RGB2 reports firmware RX/TX activity",
        ),
        "two buttons": (
            "/mentor_pi/buttons/events",
            "ButtonEvent",
        ),
        "QMI8658": (
            "/mentor_pi/imu",
            "six stationary board orientations",
        ),
        "battery monitor": (
            "/mentor_pi/battery/state",
            "PB0/ADC1 channel 8",
            "11:1",
            "6300 mV low threshold",
            "at or below 4900 mV",
            "/mentor_pi/battery/set_low_threshold",
        ),
        "OLED": (
            "/mentor_pi/oled/command",
            "line_1",
            "line_2",
        ),
    }
    for description, markers in feature_markers.items():
        missing = [marker for marker in markers if marker not in tutorials]
        if missing:
            errors.append(
                f"{TUTORIAL_DIRECTORY}: incomplete {description} path; "
                f"missing {', '.join(repr(marker) for marker in missing)}"
            )

    return errors, len(feature_markers)


def main() -> int:
    """Run all checks and print a compact CI-friendly result."""
    required_inputs = (
        REQUIREMENTS_PATH,
        LEGACY_AUDIT_PATH,
        VERIFICATION_PATH,
        *TUTORIAL_PATHS,
    )
    missing_inputs = [str(path) for path in required_inputs if not path.is_file()]
    if missing_inputs:
        for path in missing_inputs:
            print(f"missing required documentation input: {path}", file=sys.stderr)
        return 1

    paths = markdown_files()
    link_errors, link_count = validate_markdown_links(paths)
    trace_errors, counts = validate_traceability()
    sequence_errors = validate_tutorial_sequence(paths)
    tutorial_errors, tutorial_feature_count = validate_tutorial_feature_paths()
    errors = link_errors + trace_errors + sequence_errors + tutorial_errors
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        print(f"documentation checks failed with {len(errors)} error(s)", file=sys.stderr)
        return 1

    print(
        "documentation checks passed: "
        f"{len(paths)} Markdown files, {link_count} relative links, "
        f"{counts['mandatory_requirements']}/{counts['requirements']} mandatory "
        "requirements, "
        f"{counts['audit_rows']} audit rows, "
        f"{counts['verification_cases']} verification cases, "
        f"{len(TUTORIAL_PATHS)} ordered tutorials, and "
        f"{tutorial_feature_count} tutorial feature paths"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
