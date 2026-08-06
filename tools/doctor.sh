#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DEFAULT_MINIMUM_FREE_KIB=10485760

Fail() {
  echo "RRCLite development environment check failed: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || Fail "usage: ./tools/doctor.sh"

for command_name in git make docker; do
  command -v "${command_name}" >/dev/null 2>&1 || \
    Fail "${command_name} is not installed"
done

[[ "$(git -C "${PROJECT_ROOT}" rev-parse --is-inside-work-tree 2>/dev/null)" == \
  "true" ]] || Fail "the project is not inside a Git worktree"
readonly GIT_ROOT="$(git -C "${PROJECT_ROOT}" rev-parse --show-toplevel)"
[[ "${GIT_ROOT}" == "${PROJECT_ROOT}" ]] || \
  Fail "the Git worktree root differs from the project root: ${GIT_ROOT}"

case "$(uname -m)" in
  x86_64 | amd64)
    readonly ARCHITECTURE="amd64"
    ;;
  aarch64 | arm64)
    readonly ARCHITECTURE="arm64"
    ;;
  *)
    Fail "unsupported architecture: $(uname -m)"
    ;;
esac

docker info >/dev/null 2>&1 || \
  Fail "Docker Desktop/Engine is not running or is not accessible"

readonly MINIMUM_FREE_KIB="${RRCLITE_MIN_FREE_KIB:-${DEFAULT_MINIMUM_FREE_KIB}}"
[[ "${MINIMUM_FREE_KIB}" =~ ^[0-9]+$ ]] || \
  Fail "RRCLITE_MIN_FREE_KIB must be a non-negative integer"
readonly AVAILABLE_KIB="$(
  df -Pk "${PROJECT_ROOT}" | awk 'NR == 2 {print $4}'
)"
[[ "${AVAILABLE_KIB}" =~ ^[0-9]+$ ]] || \
  Fail "could not determine available workspace disk space"
((AVAILABLE_KIB >= MINIMUM_FREE_KIB)) || \
  Fail "workspace has ${AVAILABLE_KIB} KiB free; ${MINIMUM_FREE_KIB} KiB is required"

programmer="${STM32_CUBE_PROGRAMMER_CLI:-}"
if [[ -n "${programmer}" ]]; then
  [[ -x "${programmer}" ]] || \
    Fail "STM32_CUBE_PROGRAMMER_CLI is not executable: ${programmer}"
elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  programmer="$(command -v STM32_Programmer_CLI)"
else
  readonly -a PROGRAMMER_CANDIDATES=(
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
    "/opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI"
  )
  for candidate in "${PROGRAMMER_CANDIDATES[@]}"; do
    if [[ -x "${candidate}" ]]; then
      programmer="${candidate}"
      break
    fi
  done
fi

echo "Git worktree: ${PROJECT_ROOT}"
echo "Build architecture: ${ARCHITECTURE}"
echo "Available workspace space: ${AVAILABLE_KIB} KiB"
echo "Docker: available"
if [[ -n "${programmer}" ]]; then
  echo "STM32CubeProgrammer: ${programmer}"
else
  echo "STM32CubeProgrammer: not found (optional until make flash)"
fi
echo "RRCLite development environment check passed."
