#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_PREFIX="${COMPONENT_ROOT}/build/native/install"
readonly METADATA="${BUILD_PREFIX}/AGENT-BUILD-METADATA.txt"
readonly EXECUTABLE="${BUILD_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
readonly LAUNCHER="${BUILD_PREFIX}/bin/mentor-pi-agent"
readonly SERIAL_ACCESS_HELPER="${COMPONENT_ROOT}/tools/configure_serial_access.sh"
DEVICE="${DEVICE:-}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ID_SERIAL_SHORT="${ID_SERIAL_SHORT:-}"
ID_PATH="${ID_PATH:-}"

Fail() {
  echo "Agent service installation error: $*" >&2
  exit 1
}

ReadMetadata() {
  local key="$1"
  local value
  value="$(sed -n "s/^${key}=//p" "${METADATA}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "metadata lacks one ${key} value"
  printf '%s' "${value}"
}

VerifyExistingRelease() {
  local release_path="$1"
  local expected_metadata="$2"
  local expected_executable_sha="$3"
  local expected_launcher_sha="$4"
  local installed_metadata="${release_path}/AGENT-BUILD-METADATA.txt"
  local installed_executable="${release_path}/lib/micro_ros_agent/micro_ros_agent"
  local installed_launcher="${release_path}/bin/mentor-pi-agent"

  [[ -d "${release_path}" && ! -L "${release_path}" ]] || \
    Fail "existing Agent release is not a real directory: ${release_path}"
  [[ -f "${installed_metadata}" && ! -L "${installed_metadata}" && \
     -f "${installed_executable}" && ! -L "${installed_executable}" && \
     -x "${installed_executable}" && -f "${installed_launcher}" && \
     ! -L "${installed_launcher}" && -x "${installed_launcher}" ]] || \
    Fail "existing Agent release is malformed: ${release_path}"
  cmp -s "${installed_metadata}" "${expected_metadata}" || \
    Fail "existing Agent release metadata does not match this build"
  [[ "$(sha256sum "${installed_executable}" | awk '{print $1}')" == \
     "${expected_executable_sha}" ]] || \
    Fail "existing Agent release executable does not match this build"
  [[ "$(sha256sum "${installed_launcher}" | awk '{print $1}')" == \
     "${expected_launcher_sha}" ]] || \
    Fail "existing Agent release launcher does not match this build"
}

Main() {
  [[ "$(id -u)" == 0 ]] || Fail "run this target through sudo"
  [[ ! -f /.dockerenv ]] || \
    Fail "service installation requires native Ubuntu 22.04, not a container"
  "${SCRIPT_DIR}/check_environment.sh" >/dev/null
  [[ -f "${EXECUTABLE}" && ! -L "${EXECUTABLE}" && -x "${EXECUTABLE}" && \
     -f "${LAUNCHER}" && ! -L "${LAUNCHER}" && -x "${LAUNCHER}" && \
     -f "${METADATA}" && ! -L "${METADATA}" ]] || \
    Fail "run make build before installing the service"
  [[ -x "${SERIAL_ACCESS_HELPER}" ]] || Fail "serial-access helper is unavailable"
  [[ "${ROS_DOMAIN_ID}" =~ ^(0|[1-9][0-9]{0,2})$ ]] && \
    ((ROS_DOMAIN_ID <= 232)) || Fail "ROS_DOMAIN_ID must be in [0,232]"
  [[ -z "${ID_SERIAL_SHORT}" || -z "${ID_PATH}" ]] || \
    Fail "set at most one of ID_SERIAL_SHORT or ID_PATH"
  local -a serial_access_arguments=(--user mentor-pi)
  [[ -z "${DEVICE}" ]] || serial_access_arguments+=(--device "${DEVICE}")
  [[ -z "${ID_SERIAL_SHORT}" ]] || serial_access_arguments+=(--id-serial-short "${ID_SERIAL_SHORT}")
  [[ -z "${ID_PATH}" ]] || serial_access_arguments+=(--id-path "${ID_PATH}")

  [[ "$(ReadMetadata ubuntu)" == 22.04 && \
     "$(ReadMetadata ros_distro)" == humble && \
     "$(ReadMetadata architecture)" == "$(dpkg --print-architecture)" ]] || \
    Fail "Agent build metadata does not match this host"
  local executable_sha launcher_sha
  executable_sha="$(sha256sum "${EXECUTABLE}" | awk '{print $1}')"
  [[ "${executable_sha}" == "$(ReadMetadata executable_sha256)" ]] || \
    Fail "Agent executable changed after its build"
  launcher_sha="$(sha256sum "${LAUNCHER}" | awk '{print $1}')"
  [[ "${launcher_sha}" == "$(ReadMetadata launcher_sha256)" ]] || \
    Fail "Agent launcher changed after its build"

  local release_id="${RELEASE_ID:-${executable_sha:0:16}}"
  [[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] || \
    Fail "RELEASE_ID is invalid"
  local release_root="/opt/mentor_pi/agent/releases"
  local release_path="${release_root}/${release_id}"

  "${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}" --preflight
  if ! id mentor-pi >/dev/null 2>&1; then
    useradd --system --user-group --home-dir /nonexistent \
      --shell /usr/sbin/nologin mentor-pi
  fi
  "${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}"

  install -d -o root -g root -m 0755 "${release_root}" /etc/mentor-pi
  if [[ -e "${release_path}" || -L "${release_path}" ]]; then
    VerifyExistingRelease "${release_path}" "${METADATA}" \
      "${executable_sha}" "${launcher_sha}"
  else
    local temporary_release="${release_root}/.${release_id}.tmp.$$"
    trap 'rm -rf -- "${temporary_release}"' EXIT
    install -d -o root -g root -m 0755 "${temporary_release}"
    cp -a "${BUILD_PREFIX}/." "${temporary_release}/"
    chown -R root:root "${temporary_release}"
    chmod -R go-w "${temporary_release}"
    mv "${temporary_release}" "${release_path}"
    trap - EXIT
  fi

  local current_link="/opt/mentor_pi/agent/current"
  [[ ! -e "${current_link}" || -L "${current_link}" ]] || \
    Fail "refusing to replace non-symbolic ${current_link}"
  local temporary_link="/opt/mentor_pi/agent/.current.${release_id}.$$"
  ln -s "${release_path}" "${temporary_link}"
  mv -Tf "${temporary_link}" "${current_link}"

  printf 'ROS_DOMAIN_ID=%s\nMENTOR_PI_RRCLITE_AUTORESET=1\n' \
    "${ROS_DOMAIN_ID}" >/etc/mentor-pi/agent.env
  chown root:root /etc/mentor-pi/agent.env
  chmod 0644 /etc/mentor-pi/agent.env
  install -o root -g root -m 0644 \
    "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service" \
    /etc/systemd/system/mentor-pi-agent.service

  systemctl daemon-reload
  systemctl enable mentor-pi-agent.service
  systemctl restart mentor-pi-agent.service
  echo "Installed and enabled Agent release ${release_id}."
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  Main "$@"
fi
