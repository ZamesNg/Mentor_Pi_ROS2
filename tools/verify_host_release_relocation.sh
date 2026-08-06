#!/usr/bin/env bash

set -euo pipefail

prefix=""
work_directory=""
hidden_prefix=""
relocation_root=""

Usage() {
  cat >&2 <<'EOF'
Usage: verify_host_release_relocation.sh --prefix ABSOLUTE_PATH
  --work-directory ABSOLUTE_PATH
EOF
  exit 2
}

Fail() {
  echo "Host release relocation error: $*" >&2
  exit 1
}

RestoreAndCleanup() {
  local status=$?
  if [[ -n "${hidden_prefix}" && -e "${hidden_prefix}" ]]; then
    if [[ -e "${prefix}" || -L "${prefix}" ]]; then
      echo "Cannot restore hidden host prefix because its path reappeared." >&2
      status=1
    elif ! mv "${hidden_prefix}" "${prefix}"; then
      echo "Could not restore hidden host prefix ${hidden_prefix}." >&2
      status=1
    fi
  fi
  if [[ -n "${relocation_root}" && -d "${relocation_root}" ]]; then
    rm -rf -- "${relocation_root}"
  fi
  return "${status}"
}
trap RestoreAndCleanup EXIT

while (($# > 0)); do
  case "$1" in
    --prefix) prefix="${2:-}"; shift 2 ;;
    --work-directory) work_directory="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${prefix}" == /* && -d "${prefix}" ]] || Usage
[[ "${work_directory}" == /* && -d "${work_directory}" ]] || Usage
[[ -x "${prefix}/lib/mentor_pi_bringup/promote_host_release" ]] ||
  Fail "release promoter is missing"

relocation_root="$(mktemp -d "${work_directory}/relocation.XXXXXX")"
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${relocation_root}" \
  "${prefix}/lib/mentor_pi_bringup/promote_host_release" \
    --staged-prefix "${prefix}" --release-id relocation-smoke
readonly ACTIVE_PREFIX="${relocation_root}/opt/mentor_pi/host"
readonly RESOLVED_PREFIX="$(cd "${ACTIVE_PREFIX}" && pwd -P)"

hidden_prefix="${prefix}.relocation-hidden.$$"
[[ ! -e "${hidden_prefix}" && ! -L "${hidden_prefix}" ]] ||
  Fail "temporary relocation path already exists: ${hidden_prefix}"
mv "${prefix}" "${hidden_prefix}"

smoke_script='set -eo pipefail
set +u
source "$1/setup.bash"
set -u
reported="$(ros2 pkg prefix mentor_pi_bringup)"
[[ "${reported}" == "$1" || "${reported}" == "$2" ]] || {
  echo "relocated prefix resolved to unexpected path: ${reported}" >&2
  exit 1
}
[[ ":${AMENT_PREFIX_PATH:-}:" != *":$3:"* ]] || {
  echo "relocated environment retained staging prefix: $3" >&2
  exit 1
}
ros2 interface show mentor_pi_interfaces/msg/MotorCommand >/dev/null
for executable in configuration_supervisor qualification_campaign \
    qualification_monitor motor_commissioning; do
  ldd_output="$(ldd "$1/lib/mentor_pi_bringup/${executable}")"
  if grep -Fq "not found" <<<"${ldd_output}"; then
    echo "unresolved library for ${executable}" >&2
    exit 1
  fi
  if grep -Fq "$3" <<<"${ldd_output}"; then
    echo "loader output retained staging prefix for ${executable}" >&2
    exit 1
  fi
done'
env -i HOME="${HOME:-/tmp}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash -c "${smoke_script}" mentor-pi-relocation-smoke \
    "${ACTIVE_PREFIX}" "${RESOLVED_PREFIX}" "${prefix}"

RestoreAndCleanup
hidden_prefix=""
relocation_root=""
trap - EXIT
echo "Verified host release from a copied prefix with staging unavailable."
