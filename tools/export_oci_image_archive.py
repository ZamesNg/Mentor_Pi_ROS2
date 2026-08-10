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


def descriptor_blob_name(descriptor: dict) -> str:
    """Return the OCI blob member name after minimally validating a descriptor."""
    digest = descriptor.get("digest")
    if not isinstance(digest, str) or not digest.startswith("sha256:") or \
            len(digest) != len("sha256:") + 64:
        raise ValueError("OCI descriptor must use a sha256 digest")
    try:
        int(digest.removeprefix("sha256:"), 16)
    except ValueError as error:
        raise ValueError("OCI descriptor must use a sha256 digest") from error
    if not isinstance(descriptor.get("size"), int) or descriptor["size"] < 0:
        raise ValueError("OCI descriptor must have a non-negative size")
    return f"blobs/sha256/{digest.removeprefix('sha256:')}"


def verify_descriptor(archive: tarfile.TarFile, descriptor: dict) -> bytes:
    """Verify an OCI descriptor's target bytes, digest, and recorded size."""
    name = descriptor_blob_name(descriptor)
    data = read_member(archive, name)
    if len(data) != descriptor["size"]:
        raise ValueError(f"OCI descriptor size does not match blob: {name}")
    if sha256(data) != descriptor["digest"].removeprefix("sha256:"):
        raise ValueError(f"OCI descriptor digest does not match blob: {name}")
    return data


def export_modern_oci(
        archive: tarfile.TarFile, output: str, recorded_image_digest: str,
        os_name: str, architecture: str, reference: str) -> str:
    """Repack one selected platform from Docker's OCI-aware save archive."""
    source_index = json.loads(read_member(archive, "index.json"))
    descriptors = source_index.get("manifests")
    if not isinstance(descriptors, list) or len(descriptors) != 1:
        raise ValueError("OCI Docker archive must contain exactly one root descriptor")
    source_descriptor = descriptors[0]
    if not isinstance(source_descriptor, dict):
        raise ValueError("OCI root descriptor is invalid")
    if source_descriptor.get("digest") != f"sha256:{recorded_image_digest}":
        raise ValueError("recorded image ID does not match the OCI root descriptor")
    nested_index_bytes = verify_descriptor(archive, source_descriptor)
    if source_descriptor.get("mediaType") != \
            "application/vnd.oci.image.index.v1+json":
        raise ValueError("OCI root descriptor is not an image index")
    nested_index = json.loads(nested_index_bytes)
    manifests = nested_index.get("manifests")
    if not isinstance(manifests, list):
        raise ValueError("OCI nested index has no manifests")

    selected = None
    for descriptor in manifests:
        if not isinstance(descriptor, dict):
            raise ValueError("OCI nested index descriptor is invalid")
        platform = descriptor.get("platform")
        if isinstance(platform, dict) and platform.get("architecture") == architecture \
                and platform.get("os") == os_name:
            if selected is not None:
                raise ValueError("OCI nested index has multiple requested platforms")
            selected = descriptor
    if selected is None:
        raise ValueError("OCI Docker archive platform does not match the request")
    if selected.get("mediaType") != \
            "application/vnd.oci.image.manifest.v1+json":
        raise ValueError("OCI platform descriptor is not an image manifest")
    image_manifest = json.loads(verify_descriptor(archive, selected))
    config_descriptor = image_manifest.get("config")
    if not isinstance(config_descriptor, dict):
        raise ValueError("OCI image manifest has no config descriptor")
    config = json.loads(verify_descriptor(archive, config_descriptor))
    if config.get("os") != os_name or config.get("architecture") != architecture:
        raise ValueError("OCI image config platform does not match the request")
    layers = image_manifest.get("layers")
    if not isinstance(layers, list):
        raise ValueError("OCI image manifest has invalid layers")
    for layer in layers:
        if not isinstance(layer, dict):
            raise ValueError("OCI image manifest has invalid layer descriptor")
        verify_descriptor(archive, layer)

    root_descriptor = {
        "annotations": {
            "org.opencontainers.image.ref.name": reference,
            "org.mentor-pi.source-image-id": f"sha256:{recorded_image_digest}",
        },
        "digest": selected["digest"],
        "mediaType": selected["mediaType"],
        "platform": selected["platform"],
        "size": selected["size"],
    }
    index = compact_json({"manifests": [root_descriptor], "schemaVersion": 2})
    with tarfile.open(output, "w", format=tarfile.PAX_FORMAT) as oci:
        add_bytes(oci, "oci-layout", compact_json({"imageLayoutVersion": "1.0.0"}))
        add_bytes(oci, "index.json", index)
        copy_member(
            archive, oci, descriptor_blob_name(selected),
            descriptor_blob_name(selected), selected["size"]
        )
        copy_member(
            archive, oci, descriptor_blob_name(config_descriptor),
            descriptor_blob_name(config_descriptor), config_descriptor["size"]
        )
        for layer in layers:
            copy_member(
                archive, oci, descriptor_blob_name(layer),
                descriptor_blob_name(layer), layer["size"]
            )
    # Docker loads this flattened archive as the selected platform manifest,
    # rather than as the source archive's nested index.
    return selected["digest"]


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
    try:
        int(recorded_image_digest, 16)
    except ValueError as error:
        raise ValueError("image ID must be a sha256 digest") from error

    with tarfile.open(args.docker_archive, "r:*") as docker_archive:
        try:
            docker_archive.getmember("oci-layout")
        except KeyError:
            pass
        else:
            runtime_image_id = export_modern_oci(
                docker_archive, args.output, recorded_image_digest,
                args.os, args.architecture, args.reference
            )
            print(runtime_image_id)
            return
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
    # Legacy Docker saves preserve their recorded image identity after load.
    print(args.image_id)


if __name__ == "__main__":
    main()
