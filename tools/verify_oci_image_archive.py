#!/usr/bin/env python3

"""Verify a flattened single-platform OCI image archive."""

import argparse
import collections
import hashlib
import json
import re
import tarfile

from export_oci_image_archive import read_member, verify_descriptor


OCI_MANIFEST = "application/vnd.oci.image.manifest.v1+json"
OCI_CONFIG = "application/vnd.oci.image.config.v1+json"


def require_object(value: object, description: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{description} is invalid")
    return value


def verify_archive(
        archive_path: str, image_id: str, os_name: str,
        architecture: str) -> None:
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id):
        raise ValueError("runtime image ID must be a sha256 digest")

    with tarfile.open(archive_path, "r:*") as archive:
        members_by_name = collections.defaultdict(list)
        for member in archive.getmembers():
            members_by_name[member.name].append(member)
        for name, members in members_by_name.items():
            if len(members) < 2:
                continue
            identities = set()
            for member in members:
                stream = archive.extractfile(member) if member.isfile() else None
                if stream is None:
                    raise ValueError(
                        f"duplicate OCI archive member is not a file: {name}"
                    )
                identities.add((member.size, hashlib.sha256(stream.read()).digest()))
            if len(identities) != 1:
                raise ValueError(
                    f"duplicate OCI archive members differ: {name}"
                )

        layout = require_object(
            json.loads(read_member(archive, "oci-layout")), "OCI layout"
        )
        if layout.get("imageLayoutVersion") != "1.0.0":
            raise ValueError("OCI archive has an unsupported layout version")

        index = require_object(
            json.loads(read_member(archive, "index.json")), "OCI index"
        )
        descriptors = index.get("manifests")
        if not isinstance(descriptors, list) or len(descriptors) != 1:
            raise ValueError("OCI archive must contain exactly one root descriptor")
        descriptor = require_object(descriptors[0], "OCI root descriptor")
        if descriptor.get("digest") != image_id:
            raise ValueError("runtime image ID does not match the OCI root descriptor")
        if descriptor.get("mediaType") != OCI_MANIFEST:
            raise ValueError("OCI root descriptor is not an image manifest")
        platform = require_object(
            descriptor.get("platform"), "OCI root descriptor platform"
        )
        if platform.get("os") != os_name or \
                platform.get("architecture") != architecture:
            raise ValueError("OCI root descriptor platform does not match the request")

        manifest = require_object(
            json.loads(verify_descriptor(archive, descriptor)),
            "OCI image manifest",
        )
        if manifest.get("schemaVersion") != 2 or \
                manifest.get("mediaType") != OCI_MANIFEST:
            raise ValueError("OCI image manifest identity is invalid")
        config_descriptor = require_object(
            manifest.get("config"), "OCI image config descriptor"
        )
        if config_descriptor.get("mediaType") != OCI_CONFIG:
            raise ValueError("OCI image config descriptor has the wrong media type")
        config = require_object(
            json.loads(verify_descriptor(archive, config_descriptor)),
            "OCI image config",
        )
        if config.get("os") != os_name or \
                config.get("architecture") != architecture:
            raise ValueError("OCI image config platform does not match the request")

        layers = manifest.get("layers")
        if not isinstance(layers, list):
            raise ValueError("OCI image manifest has invalid layers")
        for layer in layers:
            verify_descriptor(
                archive, require_object(layer, "OCI image layer descriptor")
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--os", required=True)
    parser.add_argument("--architecture", required=True)
    args = parser.parse_args()
    verify_archive(args.archive, args.image_id, args.os, args.architecture)
    print(f"{args.image_id} {args.os}/{args.architecture}")


if __name__ == "__main__":
    main()
