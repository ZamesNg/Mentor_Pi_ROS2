#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly INSTALL_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/.deps/arm-gnu-toolchain-13.2.rel1"
readonly DOWNLOAD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/.deps/downloads"
readonly TOOLCHAIN_BASE_URL="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.2.rel1/binrel"

Fail() {
  echo "Native Arm toolchain setup error: $*" >&2
  exit 1
}

case "${1:-}" in
  "") ;;
  --print-bin) ;;
  --print-plan) ;;
  *) Fail "usage: ./tools/bootstrap_native_arm_toolchain.sh [--print-bin|--print-plan]" ;;
esac

case "$(uname -m)" in
  x86_64 | amd64)
    readonly TOOLCHAIN_HOST="x86_64"
    readonly TOOLCHAIN_SHA256="6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb"
    ;;
  aarch64 | arm64)
    readonly TOOLCHAIN_HOST="aarch64"
    readonly TOOLCHAIN_SHA256="8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a"
    ;;
  *) Fail "only amd64 and arm64 native hosts are supported" ;;
esac

readonly ARCHIVE_NAME="arm-gnu-toolchain-13.2.rel1-${TOOLCHAIN_HOST}-arm-none-eabi.tar.xz"
readonly ARCHIVE_PATH="${DOWNLOAD_ROOT}/${ARCHIVE_NAME}"
readonly BIN_ROOT="${INSTALL_ROOT}/bin"

if [[ "${1:-}" == "--print-plan" ]]; then
  printf '%s\n' \
    "host=${TOOLCHAIN_HOST}" \
    "archive=${ARCHIVE_NAME}" \
    "sha256=${TOOLCHAIN_SHA256}" \
    "install_root=${INSTALL_ROOT}" \
    "version=13.2.1"
  exit 0
fi

VerifyToolchain() {
  [[ -x "${BIN_ROOT}/arm-none-eabi-gcc" &&
     -x "${BIN_ROOT}/arm-none-eabi-g++" &&
     -x "${BIN_ROOT}/arm-none-eabi-objcopy" ]] || return 1
  [[ "$("${BIN_ROOT}/arm-none-eabi-gcc" -dumpfullversion)" == "13.2.1" &&
     "$("${BIN_ROOT}/arm-none-eabi-g++" -dumpfullversion)" == "13.2.1" ]]
}

if ! VerifyToolchain; then
  command -v curl >/dev/null 2>&1 || Fail "curl is not installed"
  command -v sha256sum >/dev/null 2>&1 || Fail "sha256sum is not installed"
  command -v tar >/dev/null 2>&1 || Fail "tar is not installed"
  mkdir -p -- "${DOWNLOAD_ROOT}"
  if [[ -f "${ARCHIVE_PATH}" ]]; then
    echo "${TOOLCHAIN_SHA256}  ${ARCHIVE_PATH}" | sha256sum --check --strict - \
      >/dev/null || rm -f -- "${ARCHIVE_PATH}"
  fi
  if [[ ! -f "${ARCHIVE_PATH}" ]]; then
    curl -fL --retry 5 --retry-all-errors --retry-delay 2 \
      --continue-at - --connect-timeout 30 --max-time 1800 \
      "${TOOLCHAIN_BASE_URL}/${ARCHIVE_NAME}" -o "${ARCHIVE_PATH}"
  fi
  echo "${TOOLCHAIN_SHA256}  ${ARCHIVE_PATH}" | \
    sha256sum --check --strict - >/dev/null

  readonly STAGING_ROOT="${INSTALL_ROOT}.staging.$$"
  [[ ! -e "${STAGING_ROOT}" && ! -L "${STAGING_ROOT}" ]] || \
    Fail "unexpected staging path already exists: ${STAGING_ROOT}"
  mkdir -p -- "${STAGING_ROOT}"
  trap 'rm -rf -- "${STAGING_ROOT}"' EXIT
  tar -xJf "${ARCHIVE_PATH}" -C "${STAGING_ROOT}" --strip-components=1
  [[ "${INSTALL_ROOT}" == "${PROJECT_ROOT}/firmware/mentor_pi_mcu/.deps/arm-gnu-toolchain-13.2.rel1" ]] || \
    Fail "refusing to replace an unexpected toolchain path"
  rm -rf -- "${INSTALL_ROOT}"
  mv -- "${STAGING_ROOT}" "${INSTALL_ROOT}"
  trap - EXIT
  VerifyToolchain || Fail "the extracted toolchain is incomplete or has the wrong version"
fi

if [[ "${1:-}" == "--print-bin" ]]; then
  printf '%s\n' "${BIN_ROOT}"
else
  echo "Verified native Arm GNU 13.2.1 toolchain: ${BIN_ROOT}"
fi
