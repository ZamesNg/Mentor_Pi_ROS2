#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Pinned build-image selection failed: $*" >&2
  exit 1
}

[[ "$#" == "2" ]] || \
  Fail "usage: select_pinned_build_image.sh microros|ubuntu amd64|arm64"
readonly COMPONENT="$1"
readonly ARCHITECTURE="$2"
[[ "${ARCHITECTURE}" == "amd64" || "${ARCHITECTURE}" == "arm64" ]] || \
  Fail "architecture must be amd64 or arm64"

# These are architecture-specific child manifests of the reviewed immutable
# multi-platform indexes. Child digests avoid Docker Desktop's inability to
# bind one index-digest reference to two local platform images at once.
case "${COMPONENT}:${ARCHITECTURE}" in
  microros:amd64)
    echo 'microros/micro_ros_static_library_builder:humble@sha256:8dbeecd73df7a36327259321596755eebda27c1c760eded49720745bf909516a'
    ;;
  microros:arm64)
    echo 'microros/micro_ros_static_library_builder:humble@sha256:460b3ea2cd41d6256e5f09f9e7bf543f63a04890719abcc10d58acad12f33fa7'
    ;;
  ubuntu:amd64)
    echo 'ubuntu:24.04@sha256:019e8eb29a85e74d64925745884f2ec79aa27e3feab36353d24656f4d6b89467'
    ;;
  ubuntu:arm64)
    echo 'ubuntu:24.04@sha256:b17516cd982bf06bdd5d5600253d12a8de017b9eb831cc052b532a0363d294f9'
    ;;
  *)
    Fail "component must be microros or ubuntu"
    ;;
esac
