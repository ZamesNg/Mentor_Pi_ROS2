#!/usr/bin/env python3

import hashlib
import gzip
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
    def write_modern_oci_save(
            self, source: pathlib.Path, *, corrupt_nested_index: bool = False,
            absent_arm64_descriptor: bool = False,
    ) -> tuple[str, str, dict[str, bytes]]:
        config = json.dumps({
            "architecture": "amd64",
            "os": "linux",
            "rootfs": {"diff_ids": [], "type": "layers"},
        }, separators=(",", ":")).encode()
        compressed_layer = gzip.compress(b"compressed OCI layer fixture", mtime=0)
        config_digest = hashlib.sha256(config).hexdigest()
        layer_digest = hashlib.sha256(compressed_layer).hexdigest()
        image_manifest = json.dumps({
            "config": {
                "digest": f"sha256:{config_digest}",
                "mediaType": "application/vnd.oci.image.config.v1+json",
                "size": len(config),
            },
            "layers": [{
                "digest": f"sha256:{layer_digest}",
                "mediaType": "application/vnd.oci.image.layer.v1.tar+gzip",
                "size": len(compressed_layer),
            }],
            "mediaType": "application/vnd.oci.image.manifest.v1+json",
            "schemaVersion": 2,
        }, sort_keys=True, separators=(",", ":")).encode()
        image_manifest_digest = hashlib.sha256(image_manifest).hexdigest()
        attestation = b'{"fixture":"attestation"}'
        attestation_digest = hashlib.sha256(attestation).hexdigest()
        nested_manifests = [{
                "digest": f"sha256:{image_manifest_digest}",
                "mediaType": "application/vnd.oci.image.manifest.v1+json",
                "platform": {"architecture": "amd64", "os": "linux"},
                "size": len(image_manifest),
            }, {
                "digest": f"sha256:{attestation_digest}",
                "mediaType": "application/vnd.oci.image.manifest.v1+json",
                "size": len(attestation),
            }]
        absent_arm64_digest = "a" * 64
        if absent_arm64_descriptor:
            nested_manifests.append({
                "digest": f"sha256:{absent_arm64_digest}",
                "mediaType": "application/vnd.oci.image.manifest.v1+json",
                "platform": {"architecture": "arm64", "os": "linux"},
                "size": 123,
            })
        nested_index = json.dumps({
            "manifests": nested_manifests,
            "schemaVersion": 2,
        }, sort_keys=True, separators=(",", ":")).encode()
        nested_index_digest = hashlib.sha256(nested_index).hexdigest()
        source_index = json.dumps({
            "manifests": [{
                "digest": f"sha256:{nested_index_digest}",
                "mediaType": "application/vnd.oci.image.index.v1+json",
                "size": len(nested_index),
            }],
            "schemaVersion": 2,
        }, sort_keys=True, separators=(",", ":")).encode()
        blobs = {
            nested_index_digest: b"x" * len(nested_index)
            if corrupt_nested_index else nested_index,
            image_manifest_digest: image_manifest,
            config_digest: config,
            layer_digest: compressed_layer,
            attestation_digest: attestation,
        }
        with tarfile.open(source, "w") as archive:
            add_bytes(archive, "oci-layout", b'{"imageLayoutVersion":"1.0.0"}')
            add_bytes(archive, "index.json", source_index)
            for digest, data in blobs.items():
                add_bytes(archive, f"blobs/sha256/{digest}", data)
        return nested_index_digest, image_manifest_digest, {
            image_manifest_digest: image_manifest,
            config_digest: config,
            layer_digest: compressed_layer,
        }

    def modern_command(
            self, source: pathlib.Path, output: pathlib.Path, image_id: str,
            architecture: str = "amd64") -> list[str]:
        return [
            str(SCRIPT), "--docker-archive", str(source), "--output", str(output),
            "--image-id", image_id, "--os", "linux",
            "--architecture", architecture, "--reference", "fixture:modern",
        ]

    def test_preserves_modern_oci_save_and_binds_root_image_id(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker-modern.tar"
            output = root / "oci.tar"
            root_digest, image_manifest_digest, selected_blobs = \
                self.write_modern_oci_save(source)

            completed = subprocess.run(
                self.modern_command(source, output, f"sha256:{root_digest}"),
                check=True, capture_output=True, text=True,
            )
            self.assertEqual(
                completed.stdout, f"sha256:{image_manifest_digest}\n"
            )

            with tarfile.open(output) as archive:
                index = json.loads(archive.extractfile("index.json").read())
                descriptor = index["manifests"][0]
                self.assertEqual(
                    descriptor["digest"], f"sha256:{image_manifest_digest}"
                )
                self.assertEqual(
                    descriptor["annotations"]["org.opencontainers.image.ref.name"],
                    "fixture:modern",
                )
                self.assertEqual(
                    descriptor["annotations"]["org.mentor-pi.source-image-id"],
                    f"sha256:{root_digest}",
                )
                self.assertEqual(
                    set(archive.getnames()),
                    {"oci-layout", "index.json"} |
                    {f"blobs/sha256/{digest}" for digest in selected_blobs},
                )
                output_manifest = json.loads(
                    archive.extractfile(
                        f"blobs/sha256/{image_manifest_digest}"
                    ).read()
                )
                for child in [output_manifest["config"]] + output_manifest["layers"]:
                    self.assertIn(
                        child["digest"].removeprefix("sha256:"), selected_blobs
                    )
                for digest, expected in selected_blobs.items():
                    self.assertEqual(
                        archive.extractfile(f"blobs/sha256/{digest}").read(), expected
                    )

    def test_modern_oci_rejects_wrong_image_id(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker-modern.tar"
            output = root / "oci.tar"
            self.write_modern_oci_save(source)
            failed = subprocess.run(self.modern_command(
                source, output, "sha256:" + "0" * 64
            ), capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("does not match the OCI root descriptor", failed.stderr)

    def test_modern_oci_rejects_wrong_platform(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker-modern.tar"
            output = root / "oci.tar"
            root_digest, _, _ = self.write_modern_oci_save(source)
            failed = subprocess.run(self.modern_command(
                source, output, f"sha256:{root_digest}", "arm64"
            ), capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("platform does not match", failed.stderr)

    def test_modern_oci_rejects_corrupt_nested_index_blob(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker-modern.tar"
            output = root / "oci.tar"
            root_digest, _, _ = self.write_modern_oci_save(
                source, corrupt_nested_index=True
            )
            failed = subprocess.run(self.modern_command(
                source, output, f"sha256:{root_digest}"
            ), capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("digest does not match blob", failed.stderr)

    def test_modern_oci_ignores_absent_other_platform_blob(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            source = root / "docker-modern.tar"
            output = root / "oci.tar"
            root_digest, image_manifest_digest, selected_blobs = \
                self.write_modern_oci_save(source, absent_arm64_descriptor=True)

            completed = subprocess.run(
                self.modern_command(source, output, f"sha256:{root_digest}"),
                check=True, capture_output=True, text=True,
            )
            self.assertEqual(
                completed.stdout, f"sha256:{image_manifest_digest}\n"
            )

            with tarfile.open(output) as archive:
                index = json.loads(archive.extractfile("index.json").read())
                self.assertEqual(
                    index["manifests"][0]["digest"],
                    f"sha256:{image_manifest_digest}",
                )
                self.assertNotIn("blobs/sha256/" + "a" * 64, archive.getnames())
                self.assertEqual(
                    set(archive.getnames()),
                    {"oci-layout", "index.json"} |
                    {f"blobs/sha256/{digest}" for digest in selected_blobs},
                )

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
            completed = subprocess.run(
                command, check=True, capture_output=True, text=True
            )
            self.assertEqual(completed.stdout, f"sha256:{config_digest}\n")
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
