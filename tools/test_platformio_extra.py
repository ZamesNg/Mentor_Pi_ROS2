#!/usr/bin/env python3

import builtins
import configparser
import hashlib
import os
from pathlib import Path
import runpy
import stat
import sys
import tempfile
import types
import unittest
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCRIPT = PROJECT_ROOT / "firmware" / "mentor_pi_mcu" / "platformio_extra.py"


class FakeEnvironment:
    def AddMethod(self, method, name):
        setattr(self, name, types.MethodType(method, self))


class CommissioningGateEnvironment:
    def __init__(self, project_root):
        self.project_root = project_root

    def subst(self, variable):
        if variable != "$PROJECT_DIR":
            raise AssertionError(f"unexpected substitution: {variable}")
        return str(self.project_root)

    def GetProjectOption(self, option, default=None):
        if option == "custom_firmware_motor_mode":
            return "COMMISSIONING"
        if option == "custom_firmware_elf":
            return default
        raise AssertionError(f"unexpected project option: {option}")


def LoadExtraScript(command_line_targets=()):
    scons = types.ModuleType("SCons")
    scons_script = types.ModuleType("SCons.Script")
    scons_script.AlwaysBuild = lambda value: value
    scons_script.COMMAND_LINE_TARGETS = list(command_line_targets)
    previous_scons = sys.modules.get("SCons")
    previous_script = sys.modules.get("SCons.Script")
    previous_import = getattr(builtins, "Import", None)
    sys.modules["SCons"] = scons
    sys.modules["SCons.Script"] = scons_script
    builtins.Import = lambda _name: None
    try:
        return runpy.run_path(str(SCRIPT), init_globals={"env": FakeEnvironment()})
    finally:
        if previous_scons is None:
            del sys.modules["SCons"]
        else:
            sys.modules["SCons"] = previous_scons
        if previous_script is None:
            del sys.modules["SCons.Script"]
        else:
            sys.modules["SCons.Script"] = previous_script
        if previous_import is None:
            delattr(builtins, "Import")
        else:
            builtins.Import = previous_import


class PlatformioExtraTest(unittest.TestCase):
    def test_nobuild_is_rejected_at_script_load(self):
        with self.assertRaisesRegex(RuntimeError, "nobuild"):
            LoadExtraScript(("nobuild", "upload"))

    def test_only_canonical_authoritative_elf_is_accepted(self):
        functions = LoadExtraScript()
        resolve = functions["_resolve_authoritative_elf"]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            authoritative = (
                root
                / "firmware"
                / "mentor_pi_mcu"
                / "build"
                / "stm32"
                / "mentor_pi_mcu.elf"
            )
            authoritative.parent.mkdir(parents=True)
            authoritative.write_bytes(b"verified")
            authoritative = authoritative.resolve()
            relative = "firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
            self.assertEqual(resolve(root, relative), authoritative)
            self.assertEqual(resolve(root, authoritative), authoritative)

            alias = root / "authoritative-alias.elf"
            alias.symlink_to(authoritative)
            self.assertEqual(resolve(root, alias), authoritative)

            unrelated = root / "unrelated.elf"
            unrelated.write_bytes(b"wrong")
            with self.assertRaisesRegex(RuntimeError, "may not override"):
                resolve(root, unrelated)
            unrelated_alias = root / "unrelated-alias.elf"
            unrelated_alias.symlink_to(unrelated)
            with self.assertRaisesRegex(RuntimeError, "may not override"):
                resolve(root, unrelated_alias)

    def test_snapshot_is_hash_bound_and_survives_source_replacement(self):
        functions = LoadExtraScript()
        snapshot_verified = functions["_snapshot_verified_elf"]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = root / "mentor_pi_mcu.elf"
            original = b"verified locked firmware"
            elf.write_bytes(original)
            digest = hashlib.sha256(original).hexdigest()
            metadata = root / "rrclite-build-metadata.txt"
            metadata.write_text(
                "schema=rrclite-firmware-build-v1\n"
                "motor_mode=LOCKED\n"
                f"elf_sha256={digest}\n",
                encoding="utf-8",
            )
            snapshot = snapshot_verified(
                elf, metadata, "LOCKED", root / "platformio-build"
            )
            self.assertEqual(stat.S_IMODE(snapshot.stat().st_mode), 0o444)
            elf.write_bytes(b"later replacement")
            self.assertEqual(snapshot.read_bytes(), original)
            self.assertEqual(hashlib.sha256(snapshot.read_bytes()).hexdigest(), digest)

            # Re-preparing the same digest reuses, rather than replaces, the
            # atomically published snapshot.
            elf.write_bytes(original)
            self.assertEqual(
                snapshot_verified(
                    elf, metadata, "LOCKED", root / "platformio-build"
                ),
                snapshot,
            )

    def test_snapshot_never_replaces_conflicting_existing_destination(self):
        functions = LoadExtraScript()
        snapshot_verified = functions["_snapshot_verified_elf"]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = root / "mentor_pi_mcu.elf"
            content = b"reviewed firmware"
            elf.write_bytes(content)
            digest = hashlib.sha256(content).hexdigest()
            metadata = root / "rrclite-build-metadata.txt"
            metadata.write_text(
                "motor_mode=LOCKED\n" f"elf_sha256={digest}\n",
                encoding="utf-8",
            )
            destination = (
                root
                / "build"
                / "verified-artifacts"
                / f"mentor_pi_mcu-{digest}.elf"
            )
            destination.parent.mkdir(parents=True)
            destination.write_bytes(b"conflicting bytes")
            with self.assertRaisesRegex(RuntimeError, "Existing verified"):
                snapshot_verified(elf, metadata, "LOCKED", root / "build")
            self.assertEqual(destination.read_bytes(), b"conflicting bytes")

    def test_snapshot_rejects_wrong_mode_and_hash(self):
        functions = LoadExtraScript()
        snapshot_verified = functions["_snapshot_verified_elf"]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = root / "mentor_pi_mcu.elf"
            elf.write_bytes(b"firmware")
            metadata = root / "rrclite-build-metadata.txt"
            metadata.write_text(
                "motor_mode=COMMISSIONING\n" + "elf_sha256=" + "0" * 64 + "\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "mode changed"):
                snapshot_verified(elf, metadata, "LOCKED", root / "build")
            with self.assertRaisesRegex(RuntimeError, "snapshot was copied"):
                snapshot_verified(elf, metadata, "COMMISSIONING", root / "build")

    def test_commissioning_upload_requires_second_exact_acknowledgement(self):
        functions = LoadExtraScript()
        use_cmake_elf = functions["_use_cmake_elf"]
        with tempfile.TemporaryDirectory() as temporary:
            environment = CommissioningGateEnvironment(Path(temporary))
            with mock.patch.dict(os.environ, {}, clear=True):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "RRCLITE_COMMISSIONING_UPLOAD_ACK="
                    "MOTORS_RAISED_CURRENT_LIMITED",
                ):
                    use_cmake_elf(environment)
            with mock.patch.dict(
                os.environ,
                {"RRCLITE_COMMISSIONING_UPLOAD_ACK": "MOTORS_RAISED"},
                clear=True,
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "RRCLITE_COMMISSIONING_UPLOAD_ACK="
                    "MOTORS_RAISED_CURRENT_LIMITED",
                ):
                    use_cmake_elf(environment)
            with mock.patch.dict(
                os.environ,
                {
                    "RRCLITE_COMMISSIONING_UPLOAD_ACK":
                    "MOTORS_RAISED_CURRENT_LIMITED"
                },
                clear=True,
            ):
                with self.assertRaisesRegex(RuntimeError, "ELF is missing"):
                    use_cmake_elf(environment)

    def test_platformio_environments_bind_exact_motor_modes(self):
        configuration = configparser.ConfigParser()
        configuration.read(PROJECT_ROOT / "platformio.ini")
        self.assertEqual(
            configuration["env"]["custom_firmware_motor_mode"],
            "LOCKED",
        )
        self.assertNotIn(
            "custom_firmware_motor_mode",
            configuration["env:rrclite_stlink"],
        )
        self.assertNotIn(
            "custom_firmware_motor_mode",
            configuration["env:rrclite_jlink"],
        )
        self.assertNotIn(
            "custom_firmware_motor_mode",
            configuration["env:rrclite_uart"],
        )
        self.assertEqual(
            configuration["env:rrclite_uart"]["upload_protocol"],
            "custom",
        )
        self.assertIn(
            "platformio_uart_upload.sh",
            configuration["env:rrclite_uart"]["upload_command"],
        )
        self.assertIn(
            "$PROGPATH",
            configuration["env:rrclite_uart"]["upload_command"],
        )
        self.assertNotIn(
            "$SOURCE",
            configuration["env:rrclite_uart"]["upload_command"],
        )
        self.assertEqual(
            configuration["env:rrclite_stlink_commissioning"][
                "custom_firmware_motor_mode"
            ],
            "COMMISSIONING",
        )
        self.assertEqual(
            configuration["env:rrclite_jlink_commissioning"][
                "custom_firmware_motor_mode"
            ],
            "COMMISSIONING",
        )
        self.assertEqual(
            configuration["env:rrclite_uart_commissioning"][
                "custom_firmware_motor_mode"
            ],
            "COMMISSIONING",
        )

    def test_platformio_debug_is_attach_only(self):
        configuration = configparser.ConfigParser()
        configuration.read(PROJECT_ROOT / "platformio.ini")
        self.assertEqual(configuration["env"]["debug_load_mode"], "manual")
        flashing_guide = (
            PROJECT_ROOT / "docs" / "flashing-and-first-bringup.md"
        ).read_text(encoding="utf-8")
        board_checklist = (
            PROJECT_ROOT / "docs" / "board-arrival-bringup-checklist.md"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "pio debug -e rrclite_stlink --interface gdb", flashing_guide
        )
        self.assertIn(
            "pio debug -e rrclite_jlink --interface gdb", flashing_guide
        )
        self.assertIn(
            "pio debug -e rrclite_stlink_commissioning --interface gdb",
            flashing_guide,
        )
        self.assertIn(
            "pio debug -e rrclite_stlink --interface gdb", board_checklist
        )


if __name__ == "__main__":
    unittest.main()
