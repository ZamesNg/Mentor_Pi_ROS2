#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ARCHIVE="${PROJECT_ROOT}/thirdpart/stm32cubeprogrammer_2.23.0_arm64.deb.zip"
readonly ARCHIVE_SHA256="99d2a1bfd8948f713ccae814b3038528d6a4e76e9d9d101857692a4d8da5de6f"
readonly DEB_NAME="stm32cubeprogrammer_2.23.0_arm64.deb"

Fail() {
  echo "Onboard STM32CubeProgrammer setup error: $*" >&2
  exit 1
}

[[ "$#" == 1 ]] || \
  Fail "usage: ./tools/install_onboard_stm32cubeprogrammer.sh --verify-archive|--install"
case "$1" in
  --verify-archive | --install) ;;
  *) Fail "usage: ./tools/install_onboard_stm32cubeprogrammer.sh --verify-archive|--install" ;;
esac

for required_tool in sha256sum unzip dpkg-deb; do
  command -v "${required_tool}" >/dev/null 2>&1 || \
    Fail "required tool is missing: ${required_tool}"
done
[[ -f "${ARCHIVE}" && ! -L "${ARCHIVE}" ]] || \
  Fail "the repository arm64 package archive is missing or symbolic: ${ARCHIVE}"
echo "${ARCHIVE_SHA256}  ${ARCHIVE}" | sha256sum --check --strict - \
  >/dev/null || Fail "the STM32CubeProgrammer archive checksum is invalid"

archive_listing="$(unzip -Z1 "${ARCHIVE}")"
[[ "${archive_listing}" == "${DEB_NAME}" ]] || \
  Fail "the STM32CubeProgrammer archive must contain only ${DEB_NAME}"

temporary_root="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-cubeprogrammer.XXXXXX")"
Cleanup() {
  [[ -d "${temporary_root}" ]] || return
  rm -rf -- "${temporary_root}"
}
trap Cleanup EXIT
unzip -q "${ARCHIVE}" "${DEB_NAME}" -d "${temporary_root}"
readonly DEB_PATH="${temporary_root}/${DEB_NAME}"

[[ "$(dpkg-deb -f "${DEB_PATH}" Package)" == "stm32cubeprogrammer" ]] || \
  Fail "the Debian package has an unexpected name"
[[ "$(dpkg-deb -f "${DEB_PATH}" Version)" == "2.23.0" ]] || \
  Fail "the Debian package has an unexpected version"
[[ "$(dpkg-deb -f "${DEB_PATH}" Architecture)" == "arm64" ]] || \
  Fail "the Debian package has an unexpected architecture"
dpkg-deb -c "${DEB_PATH}" | \
  awk '{print $6}' | grep -Fqx './usr/bin/STM32_Programmer_CLI' || \
  Fail "the Debian package does not contain STM32_Programmer_CLI"

if [[ "$1" == "--verify-archive" ]]; then
  echo "Verified repository STM32CubeProgrammer 2.23.0 arm64 package archive."
  exit 0
fi

[[ "$(id -u)" == 0 ]] || Fail "run --install as root"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "the onboard host must be Ubuntu"
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  Fail "the onboard host must run Ubuntu 22.04"
[[ "$(dpkg --print-architecture)" == "arm64" ]] || \
  Fail "the repository STM32CubeProgrammer package requires an arm64 host"

installed_identity="$(
  dpkg-query -W -f='${Version} ${Architecture}' stm32cubeprogrammer \
    2>/dev/null || true
)"
if [[ "${installed_identity}" == "2.23.0 arm64" && \
    -x /usr/bin/STM32_Programmer_CLI ]]; then
  echo "Reusing installed STM32CubeProgrammer 2.23.0 arm64."
  exit 0
fi

echo "STM32CubeProgrammer requires acceptance of ST's displayed license terms."
echo "The installer will stop without installing if you do not accept them."
dpkg -i "${DEB_PATH}"

[[ "$(dpkg-query -W -f='${Version} ${Architecture}' stm32cubeprogrammer)" == \
    "2.23.0 arm64" ]] || Fail "STM32CubeProgrammer package verification failed"
[[ -x /usr/bin/STM32_Programmer_CLI ]] || \
  Fail "STM32_Programmer_CLI is not executable after installation"
echo "Installed STM32CubeProgrammer 2.23.0 arm64: /usr/bin/STM32_Programmer_CLI"
