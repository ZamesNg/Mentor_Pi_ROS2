#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SELECTOR="${SCRIPT_DIR}/select_rdk_handoff.sh"
readonly PACKAGED_FLASHER="${SCRIPT_DIR}/flash_packaged_firmware.sh"
readonly DEFAULT_SEARCH_ROOT="${PROJECT_ROOT}/build/received-handoffs"

port=""
handoff=""

Fail() {
  echo "Production firmware flash error: $*" >&2
  exit 1
}

Usage() {
  echo "Usage: flash_production_firmware.sh --device /dev/mentor_pi_mcu [--handoff ABSOLUTE_RDK_HANDOFF]" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --device) port="${2:-}"; shift 2 ;;
    --handoff) handoff="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done
[[ -n "${port}" ]] || Usage
[[ -x "${SELECTOR}" && -x "${PACKAGED_FLASHER}" ]] || \
  Fail "production handoff helpers are unavailable"

case "$(uname -m)" in
  aarch64 | arm64) ;;
  *) Fail "make flash-production must run on the arm64 RDK X5" ;;
esac
"${SCRIPT_DIR}/detect_host_profile.sh" | grep -Fqx 'profile=rdk-x5' || \
  Fail "make flash-production requires the detected RDK X5 profile"

recorded_handoff="$("${SELECTOR}" --recorded-under "${DEFAULT_SEARCH_ROOT}")"
if [[ -n "${handoff}" ]]; then
  [[ "${handoff}" == /* ]] || Usage
  handoff="$(cd "${handoff}" 2>/dev/null && pwd -P)" || \
    Fail "explicit RDK handoff is missing"
  [[ "${handoff}" == "${recorded_handoff}" ]] || \
    Fail "explicit handoff was not received; run make rdk-receive RDK_HANDOFF=${handoff}"
else
  handoff="${recorded_handoff}"
fi

command -v systemctl >/dev/null 2>&1 || Fail "systemctl is unavailable"
if systemctl is-active --quiet mentor-pi-controller.target; then
  command -v sudo >/dev/null 2>&1 || \
    Fail "sudo is required to stop the active production target"
  echo "Stopping the active Mentor Pi production target before flashing."
  sudo systemctl stop mentor-pi-controller.target
fi
if command -v docker >/dev/null 2>&1 && \
    [[ "$(docker container inspect mentor-pi-production \
      --format '{{.State.Running}}' 2>/dev/null || true)" == true ]]; then
  Fail "stop the mentor-pi-production container before flashing"
fi

echo "Selected production handoff: ${handoff}"
exec "${PACKAGED_FLASHER}" "${port}" "${handoff}/board-handoff"
