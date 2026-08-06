"""Make PlatformIO flash/debug a verified CMake-produced ELF."""

import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from SCons.Script import AlwaysBuild, COMMAND_LINE_TARGETS

Import("env")  # type: ignore[name-defined]  # PlatformIO injects this symbol.

if "nobuild" in COMMAND_LINE_TARGETS:
    raise RuntimeError(
        "PlatformIO's nobuild target bypasses firmware verification and is "
        "unsupported. Run the normal verified upload target."
    )


def _resolve_authoritative_elf(project_root, configured_path):
    authoritative_elf = (
        project_root
        / "firmware"
        / "mentor_pi_mcu"
        / "build"
        / "stm32"
        / "mentor_pi_mcu.elf"
    ).resolve()
    elf_path = Path(configured_path)
    if not elf_path.is_absolute():
        elf_path = project_root / elf_path
    elf_path = elf_path.resolve()
    if elf_path != authoritative_elf:
        raise RuntimeError(
            "custom_firmware_elf may not override the verified authoritative "
            f"artifact: expected {authoritative_elf}, got {elf_path}"
        )
    return elf_path


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _metadata_value(metadata, key):
    prefix = f"{key}="
    values = [line[len(prefix) :] for line in metadata.splitlines() if line.startswith(prefix)]
    if len(values) != 1:
        raise RuntimeError(f"Firmware metadata must contain exactly one {key}")
    return values[0]


def _snapshot_verified_elf(elf_path, metadata_path, motor_mode, build_directory):
    metadata_before = metadata_path.read_text(encoding="utf-8")
    if _metadata_value(metadata_before, "motor_mode") != motor_mode:
        raise RuntimeError("Firmware metadata mode changed during verification")
    expected_hash = _metadata_value(metadata_before, "elf_sha256")
    if re.fullmatch(r"[0-9a-f]{64}", expected_hash) is None:
        raise RuntimeError("Firmware metadata contains a malformed ELF SHA-256")

    snapshot_directory = build_directory / "verified-artifacts"
    snapshot_directory.mkdir(parents=True, exist_ok=True)
    snapshot = snapshot_directory / f"mentor_pi_mcu-{expected_hash}.elf"
    temporary_handle = tempfile.NamedTemporaryFile(
        prefix=".mentor_pi_mcu-", suffix=".elf.tmp", dir=snapshot_directory, delete=False
    )
    temporary_path = Path(temporary_handle.name)
    temporary_handle.close()
    try:
        shutil.copyfile(elf_path, temporary_path)
        if _sha256(temporary_path) != expected_hash:
            raise RuntimeError("Firmware ELF changed while its snapshot was copied")
        if metadata_path.read_text(encoding="utf-8") != metadata_before:
            raise RuntimeError("Firmware metadata changed while its snapshot was copied")
        os.chmod(temporary_path, 0o444)
        try:
            # Publish only after the complete copy and digest check. A hard
            # link is atomic and cannot replace an existing hash-named
            # snapshot; concurrent preparation of the same digest converges
            # on the same verified bytes.
            os.link(temporary_path, snapshot)
        except FileExistsError:
            if _sha256(snapshot) != expected_hash:
                raise RuntimeError(
                    "Existing verified firmware snapshot hash mismatch"
                )
    finally:
        temporary_path.unlink(missing_ok=True)
    if _sha256(snapshot) != expected_hash:
        raise RuntimeError("Verified firmware snapshot hash mismatch")
    os.chmod(snapshot, 0o444)
    return snapshot


def _use_cmake_elf(build_environment):
    project_root = Path(build_environment.subst("$PROJECT_DIR"))
    motor_mode = build_environment.GetProjectOption(
        "custom_firmware_motor_mode", "LOCKED"
    )
    if motor_mode not in ("LOCKED", "COMMISSIONING"):
        raise RuntimeError(
            "custom_firmware_motor_mode must be LOCKED or COMMISSIONING"
        )
    if motor_mode == "COMMISSIONING":
        required_ack = "MOTORS_RAISED_CURRENT_LIMITED"
        if os.environ.get("RRCLITE_COMMISSIONING_UPLOAD_ACK") != required_ack:
            raise RuntimeError(
                "Commissioning flash/debug requires "
                f"RRCLITE_COMMISSIONING_UPLOAD_ACK={required_ack} after "
                "raising all wheels and enabling the current limit."
            )

    configured_path = build_environment.GetProjectOption(
        "custom_firmware_elf",
        "firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf",
    )
    elf_path = _resolve_authoritative_elf(project_root, configured_path)
    if not elf_path.is_file():
        raise RuntimeError(
            "CMake firmware ELF is missing: "
            f"{elf_path}. Run ./tools/build_firmware.sh first."
        )

    verifier = project_root / "tools" / "verify_firmware_artifact.sh"
    if not verifier.is_file():
        raise RuntimeError(f"Firmware artifact verifier is missing: {verifier}")
    verification = subprocess.run(
        [str(verifier), motor_mode, str(project_root)],
        check=False,
        capture_output=True,
        text=True,
    )
    if verification.returncode != 0:
        details = verification.stderr.strip() or verification.stdout.strip()
        raise RuntimeError(
            "CMake firmware artifact is not safe to flash/debug. "
            f"{details}"
        )
    if verification.stdout:
        print(verification.stdout.strip())

    metadata_path = elf_path.parent / "rrclite-build-metadata.txt"
    snapshot = _snapshot_verified_elf(
        elf_path,
        metadata_path,
        motor_mode,
        Path(build_environment.subst("$BUILD_DIR")),
    )
    # Recheck after the atomic snapshot. A concurrent rebuild is rejected even
    # though the snapshot has already captured internally consistent bytes.
    post_snapshot_verification = subprocess.run(
        [str(verifier), motor_mode, str(project_root)],
        check=False,
        capture_output=True,
        text=True,
    )
    if post_snapshot_verification.returncode != 0:
        details = (
            post_snapshot_verification.stderr.strip()
            or post_snapshot_verification.stdout.strip()
        )
        raise RuntimeError(f"Firmware changed during verification. {details}")

    program = build_environment.File(str(snapshot))
    build_environment.Replace(
        PROGNAME="mentor_pi_mcu",
        PROGPATH=str(snapshot),
        PROG_PATH=str(snapshot),
        PIOMAINPROG=program,
    )
    AlwaysBuild(build_environment.Alias("checkprogsize", program))
    return program


# STM32's builder calls env.BuildProgram(). Replacing only that method leaves
# its standard ST-Link/J-Link upload and debug recipes intact while preventing
# PlatformIO from creating a second firmware dependency graph.
env.AddMethod(_use_cmake_elf, "BuildProgram")  # type: ignore[name-defined]
