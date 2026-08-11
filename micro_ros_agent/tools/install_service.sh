#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_PREFIX="${COMPONENT_ROOT}/build/native/install"
readonly METADATA="${BUILD_PREFIX}/AGENT-BUILD-METADATA.txt"
readonly EXECUTABLE="${BUILD_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
readonly LAUNCHER="${BUILD_PREFIX}/bin/mentor-pi-agent"
readonly DEVICE_FINDER="${COMPONENT_ROOT}/tools/find_device.sh"
DEVICE="${DEVICE:-}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ID_SERIAL_SHORT="${ID_SERIAL_SHORT:-}"
ID_PATH="${ID_PATH:-}"

Fail() {
  echo "Agent service installation error: $*" >&2
  exit 1
}

[[ "$(id -u)" == 0 ]] || Fail "run this target through sudo"
[[ ! -f /.dockerenv ]] || \
  Fail "service installation requires native Ubuntu 22.04, not a container"
"${SCRIPT_DIR}/check_environment.sh" >/dev/null
[[ -x "${EXECUTABLE}" && -x "${LAUNCHER}" && -f "${METADATA}" && \
   ! -L "${METADATA}" ]] || \
  Fail "run make build before installing the service"
[[ -x "${DEVICE_FINDER}" ]] || Fail "device discovery helper is unavailable"
[[ "${ROS_DOMAIN_ID}" =~ ^(0|[1-9][0-9]{0,2})$ ]] && \
  ((ROS_DOMAIN_ID <= 232)) || Fail "ROS_DOMAIN_ID must be in [0,232]"
[[ -z "${ID_SERIAL_SHORT}" || -z "${ID_PATH}" ]] || \
  Fail "set at most one of ID_SERIAL_SHORT or ID_PATH"
if [[ -z "${DEVICE}" ]]; then
  discovery_arguments=(--path)
  if [[ -n "${ID_SERIAL_SHORT}" ]]; then
    discovery_arguments+=(--id-serial-short "${ID_SERIAL_SHORT}")
  elif [[ -n "${ID_PATH}" ]]; then
    discovery_arguments+=(--id-path "${ID_PATH}")
  fi
  DEVICE="$("${DEVICE_FINDER}" "${discovery_arguments[@]}")" || \
    Fail "automatic CH9102F discovery did not select one device"
fi
readonly DEVICE
[[ "${DEVICE}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && -c "${DEVICE}" ]] || \
  Fail "DEVICE must be a connected character device"

properties="$(udevadm info --query=property --name="${DEVICE}")" || \
  Fail "udevadm could not inspect ${DEVICE}"
grep -Fqx 'ID_VENDOR_ID=1a86' <<<"${properties}" && \
  grep -Fqx 'ID_MODEL_ID=55d4' <<<"${properties}" || \
  Fail "${DEVICE} is not the Mentor Pi CH9102F"

if [[ -z "${ID_SERIAL_SHORT}" && -z "${ID_PATH}" ]]; then
  ID_SERIAL_SHORT="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"${properties}" | head -n 1)"
  if [[ -z "${ID_SERIAL_SHORT}" ]]; then
    ID_PATH="$(sed -n 's/^ID_PATH=//p' <<<"${properties}" | head -n 1)"
  fi
fi
readonly ID_SERIAL_SHORT ID_PATH
if [[ -n "${ID_SERIAL_SHORT}" ]]; then
  identity_kind=ID_SERIAL_SHORT
  identity_value="${ID_SERIAL_SHORT}"
  selector="ATTRS{serial}==\"${ID_SERIAL_SHORT}\""
elif [[ -n "${ID_PATH}" ]]; then
  identity_kind=ID_PATH
  identity_value="${ID_PATH}"
  selector="ENV{ID_PATH}==\"${ID_PATH}\""
else
  Fail "the selected device has no stable ID_SERIAL_SHORT or ID_PATH"
fi
readonly identity_kind identity_value selector
[[ "${identity_value}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || \
  Fail "device identity contains unsupported characters"

grep -Fqx "${identity_kind}=${identity_value}" <<<"${properties}" || \
  Fail "${DEVICE} does not match the requested stable identity"

ReadMetadata() {
  local key="$1"
  local value
  value="$(sed -n "s/^${key}=//p" "${METADATA}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "metadata lacks one ${key} value"
  printf '%s' "${value}"
}

[[ "$(ReadMetadata ubuntu)" == 22.04 && \
   "$(ReadMetadata ros_distro)" == humble && \
   "$(ReadMetadata architecture)" == "$(dpkg --print-architecture)" ]] || \
  Fail "Agent build metadata does not match this host"
executable_sha="$(sha256sum "${EXECUTABLE}" | awk '{print $1}')"
[[ "${executable_sha}" == "$(ReadMetadata executable_sha256)" ]] || \
  Fail "Agent executable changed after its build"
launcher_sha="$(sha256sum "${LAUNCHER}" | awk '{print $1}')"
[[ "${launcher_sha}" == "$(ReadMetadata launcher_sha256)" ]] || \
  Fail "Agent launcher changed after its build"

release_id="${RELEASE_ID:-${executable_sha:0:16}}"
[[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] || \
  Fail "RELEASE_ID is invalid"
readonly release_id
readonly RELEASE_ROOT="/opt/mentor_pi/agent/releases"
readonly RELEASE_PATH="${RELEASE_ROOT}/${release_id}"
[[ ! -e "${RELEASE_PATH}" && ! -L "${RELEASE_PATH}" ]] || \
  Fail "Agent release ${release_id} already exists"

getent group mentor-pi-serial >/dev/null || groupadd --system mentor-pi-serial
if ! id mentor-pi >/dev/null 2>&1; then
  useradd --system --user-group --home-dir /nonexistent \
    --shell /usr/sbin/nologin mentor-pi
fi
usermod --append --groups mentor-pi-serial mentor-pi

install -d -o root -g root -m 0755 "${RELEASE_ROOT}" /etc/mentor-pi
temporary_release="${RELEASE_ROOT}/.${release_id}.tmp.$$"
trap 'rm -rf -- "${temporary_release}"' EXIT
install -d -o root -g root -m 0755 "${temporary_release}"
cp -a "${BUILD_PREFIX}/." "${temporary_release}/"
chown -R root:root "${temporary_release}"
chmod -R go-w "${temporary_release}"
mv "${temporary_release}" "${RELEASE_PATH}"
trap - EXIT
readonly CURRENT_LINK="/opt/mentor_pi/agent/current"
[[ ! -e "${CURRENT_LINK}" || -L "${CURRENT_LINK}" ]] || \
  Fail "refusing to replace non-symbolic ${CURRENT_LINK}"
temporary_link="/opt/mentor_pi/agent/.current.${release_id}.$$"
ln -s "${RELEASE_PATH}" "${temporary_link}"
mv -Tf "${temporary_link}" "${CURRENT_LINK}"

temporary_rule="$(mktemp)"
trap 'rm -f -- "${temporary_rule}"' EXIT
sed "s|@MENTOR_PI_DEVICE_IDENTITY@|${selector}|" \
  "${COMPONENT_ROOT}/udev/99-mentor-pi-mcu.rules.in" >"${temporary_rule}"
install -o root -g root -m 0644 "${temporary_rule}" \
  /etc/udev/rules.d/99-mentor-pi-mcu.rules
rm -f -- "${temporary_rule}"
trap - EXIT
printf 'ROS_DOMAIN_ID=%s\nMENTOR_PI_RRCLITE_AUTORESET=1\n' \
  "${ROS_DOMAIN_ID}" >/etc/mentor-pi/agent.env
chown root:root /etc/mentor-pi/agent.env
chmod 0644 /etc/mentor-pi/agent.env
install -o root -g root -m 0644 \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service" \
  /etc/systemd/system/mentor-pi-agent.service

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
systemctl daemon-reload
systemctl enable --now mentor-pi-agent.service
echo "Installed and enabled Agent release ${release_id}."
