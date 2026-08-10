#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() {
  echo "Firmware fingerprint error: $*" >&2
  exit 1
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    Fail "neither sha256sum nor shasum is installed"
  fi
}

Usage() {
  cat <<'EOF'
Usage: ./tools/firmware_source_fingerprint.sh [--manifest] [firmware|interfaces] [PROJECT_ROOT]

Print a deterministic SHA-256 over project-owned inputs to the selected
artifact. PROJECT_ROOT is intended for the verifier's isolated tests; normal
builds omit it. --manifest prints the canonical input manifest instead of its
SHA-256 so evidence-producing tools can reuse the same source selection.
EOF
}

PRINT_MANIFEST=0
if [[ "${1:-}" == "--manifest" ]]; then
  PRINT_MANIFEST=1
  shift
fi
[[ "$#" -le 2 ]] || {
  Usage >&2
  exit 2
}
readonly MODE="${1:-firmware}"
readonly PROJECT_ROOT="${2:-${DEFAULT_PROJECT_ROOT}}"
[[ -d "${PROJECT_ROOT}" ]] || Fail "project root does not exist: ${PROJECT_ROOT}"

case "${MODE}" in
  firmware | interfaces)
    ;;
  -h | --help)
    Usage
    exit 0
    ;;
  *)
    Usage >&2
    Fail "mode must be firmware or interfaces"
    ;;
esac

readonly MANIFEST="$(mktemp)"
readonly PATHS="$(mktemp)"
trap 'rm -f "${MANIFEST}" "${PATHS}"' EXIT

AppendFile() {
  local path="$1"
  [[ -f "${path}" ]] || Fail "required source is missing: ${path}"
  [[ ! -L "${path}" ]] || Fail "source symlink is unsupported: ${path}"
  printf '%s\n' "${path}" >>"${PATHS}"
}

AppendDirectory() {
  local path="$1"
  [[ -d "${path}" ]] || Fail "required source directory is missing: ${path}"
  [[ ! -L "${path}" ]] || Fail "source directory symlink is unsupported: ${path}"
  local symlink
  symlink="$(find "${path}" -type l -print -quit)"
  [[ -z "${symlink}" ]] || Fail "source symlink is unsupported: ${symlink}"
  find "${path}" -type f \
    ! -name '*.pyc' \
    ! -name '.DS_Store' \
    ! -path '*/__pycache__/*' \
    -print >>"${PATHS}"
}

readonly INTERFACE_ROOT="${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces"
AppendFile "${INTERFACE_ROOT}/CMakeLists.txt"
AppendFile "${INTERFACE_ROOT}/package.xml"
AppendDirectory "${INTERFACE_ROOT}/include"
AppendDirectory "${INTERFACE_ROOT}/msg"
AppendDirectory "${INTERFACE_ROOT}/srv"
if [[ "${MODE}" == "firmware" ]]; then
  readonly FIRMWARE_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu"
  AppendFile "${FIRMWARE_ROOT}/CMakeLists.txt"
  AppendDirectory "${FIRMWARE_ROOT}/app"
  AppendDirectory "${FIRMWARE_ROOT}/config"
  AppendDirectory "${FIRMWARE_ROOT}/drivers"
  AppendDirectory "${FIRMWARE_ROOT}/include"
  AppendDirectory "${FIRMWARE_ROOT}/linker"
  AppendDirectory "${FIRMWARE_ROOT}/platform"
  AppendDirectory "${FIRMWARE_ROOT}/src"
  AppendDirectory "${FIRMWARE_ROOT}/target/stm32"
  AppendFile "${PROJECT_ROOT}/tools/apply_microros_source_lock.sh"
  AppendFile "${PROJECT_ROOT}/tools/bootstrap_firmware_dependencies.sh"
  AppendFile "${PROJECT_ROOT}/tools/build_firmware.sh"
  AppendFile "${PROJECT_ROOT}/tools/run_with_build_lock.sh"
  AppendFile "${PROJECT_ROOT}/tools/check_firmware_memory.sh"
  AppendFile "${PROJECT_ROOT}/tools/build_microros_library.sh"
  AppendFile "${PROJECT_ROOT}/tools/prepare_build_images.sh"
  AppendFile "${PROJECT_ROOT}/tools/select_build_jobs.sh"
  AppendFile "${PROJECT_ROOT}/tools/verify_microros_cache.sh"
  AppendFile "${PROJECT_ROOT}/tools/docker/rrclite.Dockerfile"
  AppendFile "${PROJECT_ROOT}/tools/docker/ros-humble-packages.lock"
  AppendFile "${PROJECT_ROOT}/tools/docker_image_source_fingerprint.sh"
  AppendFile "${PROJECT_ROOT}/tools/firmware_source_fingerprint.sh"
  AppendFile "${PROJECT_ROOT}/tools/microros_artifact_fingerprint.sh"
fi

LC_ALL=C sort -u "${PATHS}" | while IFS= read -r source; do
  case "${source}" in
    *$'\n'*)
      Fail "newline in source path is unsupported"
      ;;
  esac
  relative="${source#"${PROJECT_ROOT}/"}"
  [[ "${relative}" != "${source}" ]] || \
    Fail "source is outside project root: ${source}"
  printf '%s  %s\n' "$(Sha256 "${source}")" "${relative}"
done >"${MANIFEST}"

if [[ "${PRINT_MANIFEST}" == "1" ]]; then
  cat "${MANIFEST}"
else
  Sha256 "${MANIFEST}"
fi
