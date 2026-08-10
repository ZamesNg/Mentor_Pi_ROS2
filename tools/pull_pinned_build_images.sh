#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly PROFILE_DETECTOR="${SCRIPT_DIR}/detect_host_profile.sh"
readonly BUILD_IMAGE_PREPARER="${SCRIPT_DIR}/prepare_build_images.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly -a ORIGINAL_ARGUMENTS=("$@")

architecture=""
dry_run=0
profile=""

Fail() {
  echo "RRCLite pinned-image setup failed: $*" >&2
  exit 1
}

PullImage() {
  local image="$1"
  local source="$2"
  [[ "${image}" =~ ^[^[:space:]@]+@sha256:[0-9a-f]{64}$ ]] || \
    Fail "${source} does not identify an image pinned by a sha256 digest"
  if ((dry_run == 1)); then
    printf 'docker pull --platform %q %q # %s\n' \
      "linux/${architecture}" "${image}" "${source}"
  else
    echo "Pulling ${image} for linux/${architecture} (${source})."
    docker pull --platform "linux/${architecture}" "${image}"
  fi
}

while (($# > 0)); do
  case "$1" in
    --architecture) architecture="${2:-}"; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    --profile) profile="${2:-}"; shift 2 ;;
    *) Fail "usage: pull_pinned_build_images.sh --architecture amd64|arm64 [--profile rdk-x5|normal] [--dry-run]" ;;
  esac
done
[[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
  Fail "architecture must be amd64 or arm64"
[[ -n "${profile}" ]] || profile="$(${PROFILE_DETECTOR} | sed -n 's/^profile=//p')"
[[ "${profile}" == rdk-x5 || "${profile}" == normal ]] || \
  Fail "profile must be rdk-x5 or normal"
[[ "${profile}" != rdk-x5 || "${architecture}" == arm64 ]] || \
  Fail "the RDK X5 profile requires arm64"

if ((dry_run == 0)); then
  if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
    exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "${ORIGINAL_ARGUMENTS[@]}"
  fi
  case "$(uname -m)" in
    x86_64 | amd64) native_architecture=amd64 ;;
    aarch64 | arm64) native_architecture=arm64 ;;
    *) Fail "unsupported native architecture: $(uname -m)" ;;
  esac
  [[ "${architecture}" == "${native_architecture}" ]] || \
    Fail "cross-architecture pulls/builds are forbidden; requested ${architecture} on ${native_architecture}"
  command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
  docker info >/dev/null 2>&1 || Fail "Docker is not running or accessible"
fi

PullImage "$("${IMAGE_SELECTOR}" microros "${architecture}")" \
  "unified Humble project image"
if [[ "${profile}" == normal ]]; then
  PullImage "$("${IMAGE_SELECTOR}" ubuntu "${architecture}")" \
    "normal-computer quality image"
fi

prepare_arguments=(--architecture "${architecture}")
[[ "${profile}" != normal ]] || prepare_arguments+=(--include-quality)
if ((dry_run == 1)); then
  printf './tools/prepare_build_images.sh'
  printf ' %q' "${prepare_arguments[@]}"
  printf ' # TARGETARCH=%q\n' "${architecture}"
  echo "Pinned RRCLite ${profile} image pull plan is valid for linux/${architecture}."
else
  "${BUILD_IMAGE_PREPARER}" "${prepare_arguments[@]}"
  echo "Pinned RRCLite ${profile} images are present locally for linux/${architecture}."
fi
