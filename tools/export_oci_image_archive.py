#!/usr/bin/env python3

"""Convert one Docker image-save archive into a deterministic OCI archive."""

import argparse
import hashlib
import io
import json
import tarfile


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def compact_json(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def add_bytes(archive: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(data)
    info.mode = 0o644
    info.mtime = 0
    info.uid = 0
    info.gid = 0
    archive.addfile(info, io.BytesIO(data))


def read_member(archive: tarfile.TarFile, name: str) -> bytes:
    member = archive.getmember(name)
    if not member.isfile():
        raise ValueError(f"Docker archive member is not a file: {name}")
    stream = archive.extractfile(member)
    if stream is None:
        raise ValueError(f"Docker archive member is unreadable: {name}")
    return stream.read()


def hash_member(archive: tarfile.TarFile, name: str) -> tuple[str, int]:
    member = archive.getmember(name)
    if not member.isfile():
        raise ValueError(f"Docker archive member is not a file: {name}")
    stream = archive.extractfile(member)
    if stream is None:
        raise ValueError(f"Docker archive member is unreadable: {name}")
    digest = hashlib.sha256()
    while chunk := stream.read(1024 * 1024):
        digest.update(chunk)
    return digest.hexdigest(), member.size


def copy_member(
        source: tarfile.TarFile, destination: tarfile.TarFile,
        source_name: str, destination_name: str, size: int) -> None:
    member = source.getmember(source_name)
    stream = source.extractfile(member)
    if stream is None:
        raise ValueError(f"Docker archive member is unreadable: {source_name}")
    info = tarfile.TarInfo(destination_name)
    info.size = size
    info.mode = 0o644
    info.mtime = 0
    info.uid = 0
    info.gid = 0
    destination.addfile(info, stream)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--docker-archive", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--os", required=True)
    parser.add_argument("--architecture", required=True)
    parser.add_argument("--reference", required=True)
    args = parser.parse_args()

    recorded_image_digest = args.image_id.removeprefix("sha256:")
    if len(recorded_image_digest) != 64:
        raise ValueError("image ID must be a sha256 digest")

    with tarfile.open(args.docker_archive, "r:*") as docker_archive:
        docker_manifest = json.loads(
            read_member(docker_archive, "manifest.json")
        )
        if not isinstance(docker_manifest, list) or len(docker_manifest) != 1:
            raise ValueError("Docker archive must contain exactly one image")
        image = docker_manifest[0]
        config = read_member(docker_archive, image["Config"])
        config_digest = sha256(config)
        config_json = json.loads(config)
        if config_json.get("os") != args.os or \
                config_json.get("architecture") != args.architecture:
            raise ValueError("Docker archive platform does not match the request")

        layer_members = []
        layer_descriptors = []
        for layer_name in image["Layers"]:
            digest, size = hash_member(docker_archive, layer_name)
            layer_members.append((layer_name, digest, size))
            layer_descriptors.append({
                "digest": f"sha256:{digest}",
                "mediaType": "application/vnd.oci.image.layer.v1.tar",
                "size": size,
            })

    manifest = compact_json({
        "config": {
            "digest": f"sha256:{config_digest}",
            "mediaType": "application/vnd.oci.image.config.v1+json",
            "size": len(config),
        },
        "layers": layer_descriptors,
        "mediaType": "application/vnd.oci.image.manifest.v1+json",
        "schemaVersion": 2,
    })
    manifest_digest = sha256(manifest)
    if recorded_image_digest not in (config_digest, manifest_digest):
        raise ValueError(
            "recorded image ID matches neither the OCI config nor manifest"
        )
    index = compact_json({
        "manifests": [{
            "annotations": {
                "org.opencontainers.image.ref.name": args.reference,
                "org.mentor-pi.source-image-id": args.image_id,
            },
            "digest": f"sha256:{manifest_digest}",
            "mediaType": "application/vnd.oci.image.manifest.v1+json",
            "platform": {
                "architecture": args.architecture,
                "os": args.os,
            },
            "size": len(manifest),
        }],
        "schemaVersion": 2,
    })

    with tarfile.open(args.docker_archive, "r:*") as docker_archive, \
            tarfile.open(args.output, "w", format=tarfile.PAX_FORMAT) as oci:
        add_bytes(oci, "oci-layout", compact_json({"imageLayoutVersion": "1.0.0"}))
        add_bytes(oci, "index.json", index)
        add_bytes(oci, f"blobs/sha256/{config_digest}", config)
        for layer_name, digest, size in layer_members:
            copy_member(
                docker_archive, oci, layer_name,
                f"blobs/sha256/{digest}", size
            )
        add_bytes(oci, f"blobs/sha256/{manifest_digest}", manifest)


if __name__ == "__main__":
    main()
