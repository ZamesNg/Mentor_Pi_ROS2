#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly CAPTURE_SOURCE="${PROJECT_ROOT}/ros2_ws/src/mentor_pi_bringup/scripts/capture_board_diagnostics"
readonly INSTALL_ROOT="/opt/mentor_pi/tools"
readonly EVIDENCE_ROOT="/var/log/mentor-pi/actions"

Fail() { echo "Evidence tool installation error: $*" >&2; exit 1; }

[[ "$(id -u)" == 0 ]] || {
  echo "Run this target through sudo." >&2
  exit 1
}
[[ ! -f /.dockerenv ]] || \
  Fail "production evidence tools require native Ubuntu 22.04"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "production evidence tools require native Ubuntu 22.04"
[[ -x "${CAPTURE_SOURCE}" ]] || {
  echo "Diagnostic capture source is missing or not executable." >&2
  exit 1
}

owner="${SUDO_USER:-root}"
group="$(id -gn "${owner}")"
install -d -o root -g root -m 0755 "${INSTALL_ROOT}"
install -o root -g root -m 0755 "${CAPTURE_SOURCE}" \
  "${INSTALL_ROOT}/capture_board_diagnostics"
install -d -o "${owner}" -g "${group}" -m 0750 "${EVIDENCE_ROOT}"

echo "Installed native diagnostic capture tooling in ${INSTALL_ROOT}."
echo "Production evidence will be written below ${EVIDENCE_ROOT}."
