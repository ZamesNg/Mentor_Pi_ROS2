#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly BUILD_SCRIPT="${PROJECT_ROOT}/tools/build_firmware.sh"
readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly BUILD_ROOT_RELATIVE="firmware/mentor_pi_mcu/build/stm32"
readonly PID_RELEASE_DIRECTORY="firmware-pid-release"
readonly -a ARTIFACT_EXTENSIONS=(elf hex bin map)

STAGING_ROOT=""

Usage() {
  cat <<'EOF'
Usage: ./tools/package_board_handoff.sh [OUTPUT_DIRECTORY]

Build and package the authoritative normal default PID firmware that is the
default for make firmware/flash/start. The artifact directory and the package
root contain a SHA256SUMS manifest.

If OUTPUT_DIRECTORY is relative, it is resolved from the repository root. If
it is omitted, a UTC-stamped directory is created under build/board-handoff/.
The destination must not already exist.
EOF
}

Fail() {
  echo "Board-handoff packaging error: $*" >&2
  exit 1
}

RemoveBuildRoot() {
  [[ "${BUILD_ROOT}" == \
      "${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32" && \
      "${BUILD_ROOT}" != "/" ]] || {
    echo "Refusing unsafe authoritative-build cleanup: ${BUILD_ROOT}" >&2
    return 1
  }
  rm -rf -- "${BUILD_ROOT}"
}

RemoveStagingRoot() {
  [[ -n "${STAGING_ROOT}" ]] || return 0
  case "${STAGING_ROOT}" in
    "${OUTPUT_PARENT}"/.rrclite-board-handoff.*)
      rm -rf -- "${STAGING_ROOT}"
      ;;
    *)
      echo "Refusing unsafe handoff-staging cleanup: ${STAGING_ROOT}" >&2
      return 1
      ;;
  esac
}

ReadMetadataValue() {
  local metadata="$1"
  local key="$2"
  local line
  line="$(grep -E "^${key}=" "${metadata}" || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] || {
    echo "Build metadata must contain exactly one ${key} entry." >&2
    return 1
  }
  printf '%s' "${line#*=}"
}

VerifyBuildMode() {
  local cache="${BUILD_ROOT}/CMakeCache.txt"

  [[ -f "${cache}" ]] || {
    echo "Missing authoritative build cache: ${cache}" >&2
    return 1
  }
  if ! grep -Fqx "CMAKE_BUILD_TYPE:STRING=MinSizeRel" "${cache}"; then
    echo "Authoritative build type is not MinSizeRel." >&2
    return 1
  fi
  local metadata="${BUILD_ROOT}/rrclite-build-metadata.txt"
  [[ -f "${metadata}" ]] || {
    echo "Missing verified build metadata: ${metadata}" >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" motor_mode)" == \
      "PID" ]] || {
    echo "Build metadata mode does not match the CMake cache." >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" artifact_mode)" == "NORMAL" ]] || {
    echo "Only NORMAL firmware artifacts can enter a host handoff." >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" schema)" == \
      "rrclite-firmware-build-v2" ]] || {
    echo "Unsupported build metadata schema." >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" target)" == "STM32F407VET6" ]] || {
    echo "Build metadata targets a different MCU." >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" ros_distro)" == "humble" ]] || {
    echo "Build metadata targets a different ROS distribution." >&2
    return 1
  }
  case "$(ReadMetadataValue "${metadata}" builder_mode)" in
    docker-pinned | native-ubuntu-22.04) ;;
    *)
      echo "Build metadata has an unsupported builder mode." >&2
      return 1
      ;;
  esac
  [[ "$(ReadMetadataValue "${metadata}" release_qualified)" == "0" ]] || {
    echo "Build metadata must leave release qualification pending HIL." >&2
    return 1
  }

  local provenance_key
  for provenance_key in source_sha256 interfaces_sha256 \
      microros_archive_sha256 microros_tree_sha256; do
    local provenance_hash
    provenance_hash="$(ReadMetadataValue "${metadata}" "${provenance_key}")"
    [[ "${provenance_hash}" =~ ^[0-9a-f]{64}$ ]] || {
      echo "Malformed ${provenance_key} in build metadata." >&2
      return 1
    }
  done

  local extension
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    [[ -s "${BUILD_ROOT}/mentor_pi_mcu.${extension}" ]] || {
      echo "Missing firmware artifact: mentor_pi_mcu.${extension}" >&2
      return 1
    }
    local recorded_hash
    recorded_hash="$(ReadMetadataValue "${metadata}" \
      "${extension}_sha256")"
    [[ "${recorded_hash}" =~ ^[0-9a-f]{64}$ ]] || {
      echo "Malformed ${extension}_sha256 in build metadata." >&2
      return 1
    }
    [[ "$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.${extension}")" == \
        "${recorded_hash}" ]] || {
      echo "Firmware ${extension} does not match build metadata." >&2
      return 1
    }
  done
}

BuildMode() {
  echo "Building PID firmware from a clean authoritative build directory."
  if ! RemoveBuildRoot; then
    echo "Could not clean ${BUILD_ROOT}." >&2
    return 1
  fi
  if ! "${BUILD_SCRIPT}"; then
    echo "PID firmware build failed." >&2
    RemoveBuildRoot || true
    return 1
  fi

  if ! VerifyBuildMode; then
    echo "PID firmware build did not pass mode verification." >&2
    RemoveBuildRoot || true
    return 1
  fi
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "Neither sha256sum nor shasum is installed." >&2
    return 1
  fi
}

AppendManifestEntry() {
  local manifest="$1"
  local root="$2"
  local relative_path="$3"
  local hash

  hash="$(Sha256 "${root}/${relative_path}")"
  [[ "${hash}" =~ ^[0-9a-f]{64}$ ]] || {
    echo "Malformed SHA-256 for ${relative_path}." >&2
    return 1
  }
  printf '%s  %s\n' "${hash}" "${relative_path}" >>"${manifest}"
}

VerifyManifest() {
  local directory="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum --check SHA256SUMS)
  else
    (cd "${directory}" && shasum -a 256 --check SHA256SUMS)
  fi
}

PackageModeArtifacts() {
  local destination="${STAGING_ROOT}/${PID_RELEASE_DIRECTORY}"
  mkdir -p -- "${destination}"

  local extension
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    local source="${BUILD_ROOT}/mentor_pi_mcu.${extension}"
    local packaged_name="mentor_pi_mcu-firmware-pid-release.${extension}"
    cp -- "${source}" "${destination}/${packaged_name}"
    cmp "${source}" "${destination}/${packaged_name}"
  done
  cp -- "${BUILD_ROOT}/rrclite-build-metadata.txt" \
    "${destination}/BUILD-METADATA.txt"
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    local packaged_name="mentor_pi_mcu-firmware-pid-release.${extension}"
    local recorded_hash
    recorded_hash="$(ReadMetadataValue "${destination}/BUILD-METADATA.txt" \
      "${extension}_sha256")"
    [[ "$(Sha256 "${destination}/${packaged_name}")" == \
        "${recorded_hash}" ]] || {
      echo "Packaged ${extension} does not match build metadata." >&2
      return 1
    }
  done

  printf '%s\n' \
    'target=STM32F407VET6' \
    'motor_mode=PID' \
    'control_mode=CLOSED_LOOP' \
    'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
    >"${destination}/BUILD-MODE.txt"

  local manifest="${destination}/SHA256SUMS"
  : >"${manifest}"
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    AppendManifestEntry "${manifest}" "${destination}" \
      "mentor_pi_mcu-firmware-pid-release.${extension}"
  done
  AppendManifestEntry "${manifest}" "${destination}" BUILD-MODE.txt
  AppendManifestEntry "${manifest}" "${destination}" BUILD-METADATA.txt
  VerifyManifest "${destination}"
}

WritePackageMetadata() {
  local created_utc="$1"

  printf '%s\n' \
    'package_format=rrclite-board-handoff-v1' \
    'target=STM32F407VET6' \
    "created_utc=${created_utc}" \
    "firmware_pid_release_artifact_directory=${PID_RELEASE_DIRECTORY}" \
    "authoritative_build_directory=${BUILD_ROOT_RELATIVE}" \
    'authoritative_build_mode_after_packaging=PID' \
    >"${STAGING_ROOT}/HANDOFF.txt"
}

WritePackageManifest() {
  local manifest="${STAGING_ROOT}/SHA256SUMS"
  : >"${manifest}"

  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" HANDOFF.txt

  local extension
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
      "${PID_RELEASE_DIRECTORY}/mentor_pi_mcu-firmware-pid-release.${extension}"
  done
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${PID_RELEASE_DIRECTORY}/BUILD-MODE.txt"
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${PID_RELEASE_DIRECTORY}/BUILD-METADATA.txt"
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${PID_RELEASE_DIRECTORY}/SHA256SUMS"
  VerifyManifest "${STAGING_ROOT}"
}

OnExit() {
  local status=$?
  trap - EXIT

  if [[ -n "${STAGING_ROOT}" && -d "${STAGING_ROOT}" ]]; then
    RemoveStagingRoot || status=1
  fi
  exit "${status}"
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  [[ "$#" == "1" ]] || Fail "--help does not accept other arguments"
  Usage
  exit 0
fi
[[ "$#" -le 1 ]] || {
  Usage >&2
  Fail "expected at most one output directory"
}

command -v cmp >/dev/null 2>&1 || Fail "cmp is not installed"
command -v cp >/dev/null 2>&1 || Fail "cp is not installed"
command -v mkdir >/dev/null 2>&1 || Fail "mkdir is not installed"
command -v rm >/dev/null 2>&1 || Fail "rm is not installed"
[[ -x "${BUILD_SCRIPT}" ]] || Fail "${BUILD_SCRIPT} is not executable"
if ! command -v sha256sum >/dev/null 2>&1 && \
    ! command -v shasum >/dev/null 2>&1; then
  Fail "neither sha256sum nor shasum is installed"
fi

CREATED_UTC="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
readonly CREATED_UTC
if [[ "$#" == "1" ]]; then
  OUTPUT_REQUEST="$1"
  [[ -n "${OUTPUT_REQUEST}" ]] || Fail "output directory must not be empty"
else
  OUTPUT_STAMP="$(date -u '+%Y%m%dT%H%M%SZ')"
  readonly OUTPUT_STAMP
  OUTPUT_REQUEST="build/board-handoff/${OUTPUT_STAMP}"
fi
readonly OUTPUT_REQUEST

case "${OUTPUT_REQUEST}" in
  /*)
    OUTPUT_CANDIDATE="${OUTPUT_REQUEST}"
    ;;
  *)
    OUTPUT_CANDIDATE="${PROJECT_ROOT}/${OUTPUT_REQUEST}"
    ;;
esac
OUTPUT_PARENT="$(dirname "${OUTPUT_CANDIDATE}")"
OUTPUT_NAME="$(basename "${OUTPUT_CANDIDATE}")"
[[ "${OUTPUT_NAME}" != "." && "${OUTPUT_NAME}" != ".." ]] || \
  Fail "output directory must name a new child directory"
mkdir -p -- "${OUTPUT_PARENT}"
OUTPUT_PARENT="$(cd "${OUTPUT_PARENT}" && pwd -P)"
readonly OUTPUT_PARENT
OUTPUT_ROOT="${OUTPUT_PARENT}/${OUTPUT_NAME}"
readonly OUTPUT_ROOT

case "${OUTPUT_ROOT}/" in
  "${BUILD_ROOT}/"*)
    Fail "output directory must not be inside ${BUILD_ROOT}"
    ;;
esac
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "output directory already exists: ${OUTPUT_ROOT}"

STAGING_ROOT="$(mktemp -d \
  "${OUTPUT_PARENT}/.rrclite-board-handoff.XXXXXX")"
trap OnExit EXIT

BuildMode || Fail "could not build the default PID firmware"
PackageModeArtifacts

VerifyBuildMode || Fail "authoritative build is not PID at handoff"

WritePackageMetadata "${CREATED_UTC}"
WritePackageManifest

[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "output directory appeared while packaging: ${OUTPUT_ROOT}"
mv "${STAGING_ROOT}" "${OUTPUT_ROOT}"
STAGING_ROOT=""

echo "Board-handoff package: ${OUTPUT_ROOT}"
echo "Authoritative firmware build: ${BUILD_ROOT} (PID)"
echo "PID release firmware: ${OUTPUT_ROOT}/${PID_RELEASE_DIRECTORY}"
