#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_PREFIX="${COMPONENT_ROOT}/build/native/install"
readonly METADATA="${BUILD_PREFIX}/AGENT-BUILD-METADATA.txt"
readonly EXECUTABLE="${BUILD_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
readonly LAUNCHER="${BUILD_PREFIX}/bin/mentor-pi-agent"
readonly SERIAL_ACCESS_HELPER="${COMPONENT_ROOT}/tools/configure_serial_access.sh"
readonly SERVICE_TEMPLATE="${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
readonly RELEASE_ROOT="/opt/mentor_pi/agent/releases"
DEVICE="${DEVICE:-}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"
ID_SERIAL_SHORT="${ID_SERIAL_SHORT:-}"
ID_PATH="${ID_PATH:-}"
temporary_release=""
temporary_release_prefix=""

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

TreeDigest() {
  local tree="$1"
  [[ -d "${tree}" && ! -L "${tree}" ]] || \
    Fail "release tree is not a real directory: ${tree}"

  {
    printf 'root\0'
    while IFS= read -r -d '' relative_path; do
      local entry="${tree}/${relative_path}" entry_type entry_content
      local entry_mode raw_mode
      if [[ -L "${entry}" ]]; then
        entry_type=symlink
        entry_mode=-
        printf '%s\0%s\0%s\0' "${entry_type}" "${relative_path}" \
          "${entry_mode}"
        readlink --zero -- "${entry}"
        continue
      elif [[ -d "${entry}" ]]; then
        entry_type=directory
        entry_content='-'
      elif [[ -f "${entry}" ]]; then
        entry_type=file
        entry_content="$(sha256sum -- "${entry}")"
        entry_content="${entry_content%% *}"
      else
        Fail "release contains unsupported entry type: ${relative_path}"
      fi
      raw_mode="$(stat -c '%a' -- "${entry}")"
      printf -v entry_mode '%04o' "$((8#${raw_mode} & 8#7755))"
      printf '%s\0%s\0%s\0%s\0' "${entry_type}" "${relative_path}" \
        "${entry_mode}" "${entry_content}"
    done < <(find -P "${tree}" -mindepth 1 -printf '%P\0' | LC_ALL=C sort -z)
  } | sha256sum | awk '{print $1}'
}

VerifySecureEntry() {
  local entry="$1" expected_uid="$2" expected_gid="$3" description="$4"
  local owner mode
  owner="$(stat -c '%u:%g' -- "${entry}")" || Fail "cannot inspect ${description}: ${entry}"
  [[ "${owner}" == "${expected_uid}:${expected_gid}" ]] || \
    Fail "${description} is not owned by ${expected_uid}:${expected_gid}: ${entry}"
  if [[ ! -L "${entry}" ]]; then
    mode="$(stat -c '%a' -- "${entry}")" || Fail "cannot inspect mode for ${description}: ${entry}"
    (( (8#${mode} & 8#22) == 0 )) || \
      Fail "${description} is group/world writable: ${entry}"
    if [[ -d "${entry}" ]]; then
      (( (8#${mode} & 8#1) != 0 )) || \
        Fail "${description} is not traversable by the service user: ${entry}"
    elif [[ -f "${entry}" ]]; then
      (( (8#${mode} & 8#4) != 0 )) || \
        Fail "${description} is not readable by the service user: ${entry}"
    fi
  fi
}

EnsureTrustedDirectory() {
  local directory
  for directory in "$@"; do
    if [[ -e "${directory}" || -L "${directory}" ]]; then
      [[ -d "${directory}" && ! -L "${directory}" ]] || \
        Fail "trusted installation directory is not a real directory: ${directory}"
    else
      install -d -o root -g root -m 0755 "${directory}"
    fi
    VerifySecureEntry "${directory}" 0 0 "trusted installation directory"
  done
}

RenderServiceUnit() {
  local destination="$1" domain_id="$2"
  [[ -f "${SERVICE_TEMPLATE}" && ! -L "${SERVICE_TEMPLATE}" ]] || \
    Fail "Agent service template is missing or symbolic"
  [[ "$(grep -o '@ROS_DOMAIN_ID@' "${SERVICE_TEMPLATE}" | wc -l)" -eq 1 ]] || \
    Fail "Agent service template must contain one ROS domain marker"
  sed "s/@ROS_DOMAIN_ID@/${domain_id}/" "${SERVICE_TEMPLATE}" >"${destination}"
  ! grep -Fq '@ROS_DOMAIN_ID@' "${destination}" || \
    Fail "Agent service domain rendering failed"
}

RemoveLegacyAgentEnvironment() {
  local config_directory=/etc/mentor-pi
  local environment_file="${config_directory}/agent.env"
  if [[ ! -e "${config_directory}" && ! -L "${config_directory}" ]]; then
    return
  fi
  [[ -d "${config_directory}" && ! -L "${config_directory}" ]] || \
    Fail "legacy Agent configuration directory is not a real directory"
  VerifySecureEntry "${config_directory}" 0 0 \
    "legacy Agent configuration directory"
  if [[ -e "${environment_file}" || -L "${environment_file}" ]]; then
    [[ ! -d "${environment_file}" ]] || \
      Fail "legacy Agent environment path is a directory"
    rm -f -- "${environment_file}"
  fi
  rmdir --ignore-fail-on-non-empty -- "${config_directory}"
}

VerifyExistingRelease() {
  local release_path="$1"
  local expected_metadata="$2"
  local expected_executable_sha="$3"
  local expected_launcher_sha="$4"
  local expected_tree_sha="$5"
  local expected_uid="${6:-0}"
  local expected_gid="${7:-0}"
  local installed_metadata="${release_path}/AGENT-BUILD-METADATA.txt"
  local installed_executable="${release_path}/lib/micro_ros_agent/micro_ros_agent"
  local installed_launcher="${release_path}/bin/mentor-pi-agent"

  [[ -d "${release_path}" && ! -L "${release_path}" ]] || \
    Fail "existing Agent release is not a real directory: ${release_path}"
  VerifySecureEntry "${release_path}" "${expected_uid}" "${expected_gid}" "release root"
  while IFS= read -r -d '' relative_path; do
    VerifySecureEntry "${release_path}/${relative_path}" "${expected_uid}" \
      "${expected_gid}" "release entry"
  done < <(find -P "${release_path}" -mindepth 1 -printf '%P\0' | LC_ALL=C sort -z)
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
  [[ "$(TreeDigest "${release_path}")" == "${expected_tree_sha}" ]] || \
    Fail "existing Agent release tree does not match this build"
}

CleanupTemporaryRelease() {
  [[ -n "${temporary_release}" ]] || return
  if [[ -n "${temporary_release_prefix}" && \
        "${temporary_release_prefix}" == "${RELEASE_ROOT}/."*.tmp. && \
        "${temporary_release}" == "${temporary_release_prefix}"* && \
        "$(dirname -- "${temporary_release}")" == "${RELEASE_ROOT}" ]]; then
    rm -rf -- "${temporary_release}"
    return
  fi
  echo "Refusing unsafe release staging cleanup: ${temporary_release}" >&2
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
  [[ -n "${ROS_DOMAIN_ID}" ]] || \
    Fail "ROS_DOMAIN_ID must be exported before service installation"
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
  local release_path="${RELEASE_ROOT}/${release_id}"
  local build_tree_sha
  build_tree_sha="$(TreeDigest "${BUILD_PREFIX}")"

  EnsureTrustedDirectory /run/mentor-pi
  local installer_lock=/run/mentor-pi/agent-install.lock
  [[ ! -e "${installer_lock}" || \
     (-f "${installer_lock}" && ! -L "${installer_lock}") ]] || \
    Fail "Agent installer lock is not a regular file"
  exec 9>"${installer_lock}"
  flock -n 9 || Fail "another Agent service installation is running"
  VerifySecureEntry "${installer_lock}" 0 0 "Agent installer lock"

  "${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}" --preflight
  local release_exists=0
  EnsureTrustedDirectory /opt/mentor_pi /opt/mentor_pi/agent "${RELEASE_ROOT}"
  if [[ -e "${release_path}" || -L "${release_path}" ]]; then
    VerifyExistingRelease "${release_path}" "${METADATA}" \
      "${executable_sha}" "${launcher_sha}" "${build_tree_sha}"
    release_exists=1
  else
    temporary_release_prefix="${RELEASE_ROOT}/.${release_id}.tmp."
    temporary_release="$(mktemp -d "${temporary_release_prefix}XXXXXX")"
    trap CleanupTemporaryRelease EXIT
    cp -a "${BUILD_PREFIX}/." "${temporary_release}/"
    chown -hR root:root "${temporary_release}"
    find -P "${temporary_release}" \( -type f -o -type d \) -exec chmod go-w {} +
    VerifyExistingRelease "${temporary_release}" "${METADATA}" \
      "${executable_sha}" "${launcher_sha}" "${build_tree_sha}"
  fi
  EnsureTrustedDirectory /etc/systemd/system

  if ! id mentor-pi >/dev/null 2>&1; then
    useradd --system --user-group --home-dir /nonexistent \
      --shell /usr/sbin/nologin mentor-pi
  fi
  "${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}"

  if ((release_exists == 0)); then
    VerifyExistingRelease "${temporary_release}" "${METADATA}" \
      "${executable_sha}" "${launcher_sha}" "${build_tree_sha}"
    mv -T "${temporary_release}" "${release_path}"
    temporary_release=""
    trap - EXIT
  fi

  local current_link="/opt/mentor_pi/agent/current"
  [[ ! -e "${current_link}" || -L "${current_link}" ]] || \
    Fail "refusing to replace non-symbolic ${current_link}"
  local temporary_link="/opt/mentor_pi/agent/.current.${release_id}.$$"
  ln -s "${release_path}" "${temporary_link}"
  mv -Tf "${temporary_link}" "${current_link}"

  local service_path=/etc/systemd/system/mentor-pi-agent.service
  if [[ -e "${service_path}" || -L "${service_path}" ]]; then
    [[ -f "${service_path}" && ! -L "${service_path}" ]] || \
      Fail "installed Agent service path is not a regular file"
  fi
  local temporary_service
  temporary_service="$(mktemp \
    /etc/systemd/system/.mentor-pi-agent.service.XXXXXX)"
  RenderServiceUnit "${temporary_service}" "${ROS_DOMAIN_ID}"
  chown root:root "${temporary_service}"
  chmod 0644 "${temporary_service}"
  mv -Tf "${temporary_service}" "${service_path}"
  RemoveLegacyAgentEnvironment

  systemctl daemon-reload
  systemctl enable mentor-pi-agent.service
  systemctl restart mentor-pi-agent.service
  echo "Installed and enabled Agent release ${release_id}."
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  Main "$@"
fi
