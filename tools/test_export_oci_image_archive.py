#!/usr/bin/env python3

import hashlib
import io
import json
import pathlib
import subprocess
import tarfile
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).with_name("export_oci_image_archive.py")


def add_bytes(archive: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(data)
    archive.addfile(info, io.BytesIO(data))


class OciArchiveConverterTest(unittest.TestCase):
    def test_converts_legacy_docker_save_paths_and_binds_image_id(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker.tar"
            output = root / "oci.tar"
            config = json.dumps({
                "architecture": "amd64",
                "os": "linux",
                "rootfs": {"diff_ids": [], "type": "layers"},
            }, separators=(",", ":")).encode()
            config_digest = hashlib.sha256(config).hexdigest()
            layer = b"legacy graphdriver layer fixture"
            manifest = json.dumps([{
                "Config": f"{config_digest}.json",
                "Layers": ["legacy-layer-id/layer.tar"],
                "RepoTags": ["fixture:latest"],
            }], separators=(",", ":")).encode()
            with tarfile.open(source, "w") as archive:
                add_bytes(archive, "manifest.json", manifest)
                add_bytes(archive, f"{config_digest}.json", config)
                add_bytes(archive, "legacy-layer-id/layer.tar", layer)

            command = [
                str(SCRIPT),
                "--docker-archive", str(source),
                "--output", str(output),
                "--image-id", f"sha256:{config_digest}",
                "--os", "linux",
                "--architecture", "amd64",
                "--reference", "fixture:latest",
            ]
            subprocess.run(command, check=True)
            with tarfile.open(output) as archive:
                names = set(archive.getnames())
            self.assertIn("oci-layout", names)
            self.assertIn("index.json", names)
            self.assertIn(
                f"blobs/sha256/{hashlib.sha256(layer).hexdigest()}", names
            )

            output.unlink()
            command[command.index(f"sha256:{config_digest}")] = \
                "sha256:" + "0" * 64
            failed = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("matches neither", failed.stderr)


if __name__ == "__main__":
    unittest.main()
