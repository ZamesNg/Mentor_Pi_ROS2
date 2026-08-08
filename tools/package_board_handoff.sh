#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly BUILD_SCRIPT="${PROJECT_ROOT}/tools/build_firmware.sh"
readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly BUILD_ROOT_RELATIVE="firmware/mentor_pi_mcu/build/stm32"
readonly LOCKED_DIRECTORY="locked"
readonly COMMISSIONING_DIRECTORY="commissioning-nonrelease"
readonly -a ARTIFACT_EXTENSIONS=(elf hex bin map)

RESTORE_LOCKED_REQUIRED=0
STAGING_ROOT=""

Usage() {
  cat <<'EOF'
Usage: ./tools/package_board_handoff.sh [OUTPUT_DIRECTORY]

Build and package the default motor-locked firmware. If, and only if, both
commissioning acknowledgements are present, also package the non-release
commissioning firmware:

  RRCLITE_MOTOR_COMMISSIONING=1 \
  RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
    ./tools/package_board_handoff.sh [OUTPUT_DIRECTORY]

With commissioning disabled or unset, only locked/ is produced. With both
exact acknowledgements, commissioning-nonrelease/ is also produced. Each
artifact directory and the package root contain a SHA256SUMS manifest.

If OUTPUT_DIRECTORY is relative, it is resolved from the repository root. If
it is omitted, a UTC-stamped directory is created under build/board-handoff/.
The destination must not already exist.

On every successful run, and after any interrupted or failed commissioning
build when restoration remains possible, the authoritative STM32 build
directory is rebuilt and verified as motor-locked.
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
  local expected_option="$1"
  local expected_ack="$2"
  local cache="${BUILD_ROOT}/CMakeCache.txt"

  [[ -f "${cache}" ]] || {
    echo "Missing authoritative build cache: ${cache}" >&2
    return 1
  }
  grep -Fqx "RRCLITE_MOTOR_COMMISSIONING:BOOL=${expected_option}" \
    "${cache}" || {
    echo "Authoritative build mode is not ${expected_option}." >&2
    return 1
  }
  grep -Fqx "RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=${expected_ack}" \
    "${cache}" || {
    echo "Authoritative build acknowledgement is not the expected value." >&2
    return 1
  }
  local metadata="${BUILD_ROOT}/rrclite-build-metadata.txt"
  [[ -f "${metadata}" ]] || {
    echo "Missing verified build metadata: ${metadata}" >&2
    return 1
  }
  local expected_mode="LOCKED"
  if [[ "${expected_option}" == "ON" ]]; then
    expected_mode="COMMISSIONING"
  fi
  [[ "$(ReadMetadataValue "${metadata}" motor_mode)" == \
      "${expected_mode}" ]] || {
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
  [[ "$(ReadMetadataValue "${metadata}" commissioning_ack)" == \
      "${expected_ack}" ]] || {
    echo "Build metadata commissioning acknowledgement mismatch." >&2
    return 1
  }
  [[ "$(ReadMetadataValue "${metadata}" release_qualified)" == "0" ]] || {
    echo "Unexpected build release classification." >&2
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
  local mode="$1"
  local option
  local ack

  case "${mode}" in
    LOCKED)
      option="0"
      ack=""
      ;;
    COMMISSIONING)
      option="1"
      ack="MOTORS_RAISED"
      ;;
    *)
      echo "Internal error: unknown build mode ${mode}." >&2
      return 1
      ;;
  esac

  echo "Building ${mode} firmware from a clean authoritative build directory."
  if ! RemoveBuildRoot; then
    echo "Could not clean ${BUILD_ROOT}." >&2
    return 1
  fi
  if ! RRCLITE_MOTOR_COMMISSIONING="${option}" \
      RRCLITE_MOTOR_COMMISSIONING_ACK="${ack}" \
      "${BUILD_SCRIPT}"; then
    echo "${mode} firmware build failed." >&2
    RemoveBuildRoot || true
    return 1
  fi

  local expected_option
  if [[ "${mode}" == "LOCKED" ]]; then
    expected_option="OFF"
  else
    expected_option="ON"
  fi
  if ! VerifyBuildMode "${expected_option}" "${ack}"; then
    echo "${mode} firmware build did not pass mode verification." >&2
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
  local mode="$1"
  local directory_name
  local file_suffix

  case "${mode}" in
    LOCKED)
      directory_name="${LOCKED_DIRECTORY}"
      file_suffix="locked"
      ;;
    COMMISSIONING)
      directory_name="${COMMISSIONING_DIRECTORY}"
      file_suffix="commissioning-nonrelease"
      ;;
    *)
      echo "Internal error: unknown package mode ${mode}." >&2
      return 1
      ;;
  esac

  local destination="${STAGING_ROOT}/${directory_name}"
  mkdir -p -- "${destination}"

  local extension
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    local source="${BUILD_ROOT}/mentor_pi_mcu.${extension}"
    local packaged_name="mentor_pi_mcu-${file_suffix}.${extension}"
    cp -- "${source}" "${destination}/${packaged_name}"
    cmp "${source}" "${destination}/${packaged_name}"
  done
  cp -- "${BUILD_ROOT}/rrclite-build-metadata.txt" \
    "${destination}/BUILD-METADATA.txt"
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    local packaged_name="mentor_pi_mcu-${file_suffix}.${extension}"
    local recorded_hash
    recorded_hash="$(ReadMetadataValue "${destination}/BUILD-METADATA.txt" \
      "${extension}_sha256")"
    [[ "$(Sha256 "${destination}/${packaged_name}")" == \
        "${recorded_hash}" ]] || {
      echo "Packaged ${extension} does not match build metadata." >&2
      return 1
    }
  done

  if [[ "${mode}" == "LOCKED" ]]; then
    printf '%s\n' \
      'target=STM32F407VET6' \
      'motor_mode=LOCKED' \
      'classification=NORMAL_MOTOR_LOCKED' \
      'commissioning_enabled=0' \
      >"${destination}/BUILD-MODE.txt"
  else
    printf '%s\n' \
      'target=STM32F407VET6' \
      'motor_mode=COMMISSIONING' \
      'classification=NON_RELEASE_HIL_ONLY' \
      'commissioning_enabled=1' \
      'commissioning_ack=MOTORS_RAISED' \
      >"${destination}/BUILD-MODE.txt"
  fi

  local manifest="${destination}/SHA256SUMS"
  : >"${manifest}"
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    AppendManifestEntry "${manifest}" "${destination}" \
      "mentor_pi_mcu-${file_suffix}.${extension}"
  done
  AppendManifestEntry "${manifest}" "${destination}" BUILD-MODE.txt
  AppendManifestEntry "${manifest}" "${destination}" BUILD-METADATA.txt
  VerifyManifest "${destination}"
}

WritePackageMetadata() {
  local created_utc="$1"
  local commissioning_included="$2"
  local commissioning_path

  if [[ "${commissioning_included}" == "yes" ]]; then
    commissioning_path="${COMMISSIONING_DIRECTORY}"
  else
    commissioning_path="NOT_INCLUDED"
  fi

  printf '%s\n' \
    'package_format=rrclite-board-handoff-v1' \
    'target=STM32F407VET6' \
    "created_utc=${created_utc}" \
    "locked_artifact_directory=${LOCKED_DIRECTORY}" \
    "commissioning_artifact_directory=${commissioning_path}" \
    "commissioning_included=${commissioning_included}" \
    "authoritative_build_directory=${BUILD_ROOT_RELATIVE}" \
    'authoritative_build_mode_after_packaging=LOCKED' \
    >"${STAGING_ROOT}/HANDOFF.txt"
}

WritePackageManifest() {
  local commissioning_included="$1"
  local manifest="${STAGING_ROOT}/SHA256SUMS"
  : >"${manifest}"

  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" HANDOFF.txt

  local extension
  for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
    AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
      "${LOCKED_DIRECTORY}/mentor_pi_mcu-locked.${extension}"
  done
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${LOCKED_DIRECTORY}/BUILD-MODE.txt"
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${LOCKED_DIRECTORY}/BUILD-METADATA.txt"
  AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
    "${LOCKED_DIRECTORY}/SHA256SUMS"

  if [[ "${commissioning_included}" == "yes" ]]; then
    for extension in "${ARTIFACT_EXTENSIONS[@]}"; do
      AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
        "${COMMISSIONING_DIRECTORY}/mentor_pi_mcu-commissioning-nonrelease.${extension}"
    done
    AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
      "${COMMISSIONING_DIRECTORY}/BUILD-MODE.txt"
    AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
      "${COMMISSIONING_DIRECTORY}/BUILD-METADATA.txt"
    AppendManifestEntry "${manifest}" "${STAGING_ROOT}" \
      "${COMMISSIONING_DIRECTORY}/SHA256SUMS"
  fi
  VerifyManifest "${STAGING_ROOT}"
}

OnExit() {
  local status=$?
  trap - EXIT

  if [[ "${RESTORE_LOCKED_REQUIRED}" == "1" ]]; then
    echo "Restoring the authoritative build directory to LOCKED mode." >&2
    if BuildMode LOCKED; then
      RESTORE_LOCKED_REQUIRED=0
    else
      echo "Locked restoration failed; removing the authoritative build " \
        "directory so commissioning artifacts cannot be mistaken for the " \
        "default image." >&2
      RemoveBuildRoot || true
      status=1
    fi
  fi

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

readonly REQUESTED_COMMISSIONING="${RRCLITE_MOTOR_COMMISSIONING:-0}"
readonly REQUESTED_ACK="${RRCLITE_MOTOR_COMMISSIONING_ACK:-}"
INCLUDE_COMMISSIONING=0
case "${REQUESTED_COMMISSIONING}" in
  0)
    ;;
  1)
    [[ "${REQUESTED_ACK}" == "MOTORS_RAISED" ]] || \
      Fail "commissioning requires RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED"
    INCLUDE_COMMISSIONING=1
    ;;
  *)
    Fail "RRCLITE_MOTOR_COMMISSIONING must be 0 or 1"
    ;;
esac
readonly INCLUDE_COMMISSIONING

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

# Establish and verify the safe default before any optional commissioning
# build. The final locked build is packaged after commissioning so locked/
# always matches the authoritative build directory left for flashing or SWD.
BuildMode LOCKED || Fail "could not establish the initial locked build"

if [[ "${INCLUDE_COMMISSIONING}" == "1" ]]; then
  RESTORE_LOCKED_REQUIRED=1
  BuildMode COMMISSIONING || Fail "could not build commissioning firmware"
  PackageModeArtifacts COMMISSIONING
  BuildMode LOCKED || Fail "could not restore the final locked build"
  RESTORE_LOCKED_REQUIRED=0
fi

PackageModeArtifacts LOCKED
VerifyBuildMode OFF "" || Fail "authoritative build is not locked at handoff"

if [[ "${INCLUDE_COMMISSIONING}" == "1" ]]; then
  WritePackageMetadata "${CREATED_UTC}" yes
  WritePackageManifest yes
else
  WritePackageMetadata "${CREATED_UTC}" no
  WritePackageManifest no
fi

[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "output directory appeared while packaging: ${OUTPUT_ROOT}"
mv "${STAGING_ROOT}" "${OUTPUT_ROOT}"
STAGING_ROOT=""

echo "Board-handoff package: ${OUTPUT_ROOT}"
echo "Authoritative firmware build: ${BUILD_ROOT} (LOCKED)"
if [[ "${INCLUDE_COMMISSIONING}" == "1" ]]; then
  echo "Commissioning artifacts: ${OUTPUT_ROOT}/${COMMISSIONING_DIRECTORY} " \
    "(NON-RELEASE)"
else
  echo "Commissioning artifacts were not requested and were not built."
fi
