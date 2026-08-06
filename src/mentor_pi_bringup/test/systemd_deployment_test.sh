#!/usr/bin/env bash

set -euo pipefail

readonly SOURCE_ROOT="${1:?source root argument is required}"
readonly SYSTEMD_ROOT="${SOURCE_ROOT}/systemd"
readonly AGENT_UNIT="${SYSTEMD_ROOT}/mentor-pi-agent.service"
readonly SUPERVISOR_UNIT="${SYSTEMD_ROOT}/mentor-pi-configuration-supervisor.service"
readonly RUNTIME_UNIT="${SYSTEMD_ROOT}/mentor-pi-runtime.service"
readonly CONTROLLER_TARGET="${SYSTEMD_ROOT}/mentor-pi-controller.target"
readonly SHARED_DEFAULT="${SYSTEMD_ROOT}/mentor-pi.default"
readonly SUPERVISOR_DEFAULT="${SYSTEMD_ROOT}/mentor-pi-configuration-supervisor.default"
readonly SUPERVISOR_LAUNCHER="${SYSTEMD_ROOT}/run_configuration_supervisor"
readonly ASSET_INSTALLER="${SYSTEMD_ROOT}/install_production_assets"
readonly RELEASE_PROMOTER="${SYSTEMD_ROOT}/promote_host_release"
readonly INSTALL_IDLE_GUARD="${SYSTEMD_ROOT}/require_controller_target_inactive"
readonly AGENT_WRAPPER="${SOURCE_ROOT}/scripts/run_micro_ros_agent"
readonly DIAGNOSTIC_CAPTURE="${SOURCE_ROOT}/scripts/capture_board_diagnostics"
readonly UDEV_TEMPLATE="${SOURCE_ROOT}/udev/99-mentor-pi-mcu.rules.in"

Fail() {
  echo "systemd deployment test failure: $*" >&2
  exit 1
}

RequireFile() {
  [[ -f "$1" ]] || Fail "missing file $1"
}

RequireLine() {
  local file="$1"
  local line="$2"
  grep -Fqx "${line}" "${file}" || \
    Fail "${file} is missing exact line: ${line}"
}

RequireBefore() {
  local file="$1"
  local first="$2"
  local second="$3"
  local first_line=""
  local second_line=""
  first_line="$(grep -nF "${first}" "${file}" | head -n 1 | cut -d: -f1)"
  second_line="$(grep -nF "${second}" "${file}" | head -n 1 | cut -d: -f1)"
  [[ -n "${first_line}" && -n "${second_line}" &&
     "${first_line}" -lt "${second_line}" ]] || \
    Fail "${file} does not order '${first}' before '${second}'"
}

RejectPattern() {
  local file="$1"
  local pattern="$2"
  if grep -Eq -- "${pattern}" "${file}"; then
    Fail "${file} contains forbidden pattern: ${pattern}"
  fi
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

for required_file in "${AGENT_UNIT}" "${SUPERVISOR_UNIT}" \
    "${RUNTIME_UNIT}" "${CONTROLLER_TARGET}" "${SHARED_DEFAULT}" \
    "${SUPERVISOR_DEFAULT}" "${SUPERVISOR_LAUNCHER}" \
    "${ASSET_INSTALLER}" "${RELEASE_PROMOTER}" "${INSTALL_IDLE_GUARD}" \
    "${AGENT_WRAPPER}" \
    "${DIAGNOSTIC_CAPTURE}" "${UDEV_TEMPLATE}"; do
  RequireFile "${required_file}"
done
RejectPattern "${AGENT_UNIT}" '^Environment=ROS_DOMAIN_ID='
RejectPattern "${SUPERVISOR_UNIT}" '^Environment=ROS_DOMAIN_ID='
RejectPattern "${AGENT_UNIT}" \
  '^EnvironmentFile=-/etc/default/mentor-pi$'
RejectPattern "${SUPERVISOR_UNIT}" \
  '^EnvironmentFile=-/etc/default/mentor-pi$'
RequireBefore "${SUPERVISOR_UNIT}" \
  'EnvironmentFile=-/etc/default/mentor-pi-configuration-supervisor' \
  'EnvironmentFile=/etc/default/mentor-pi'
bash -n "${SUPERVISOR_LAUNCHER}"
bash -n "${ASSET_INSTALLER}"
bash -n "${RELEASE_PROMOTER}"
bash -n "${INSTALL_IDLE_GUARD}"
bash -n "${AGENT_WRAPPER}"
bash -n "${DIAGNOSTIC_CAPTURE}"

for shared_line in \
    'Environment=ROS_LOG_DIR=/var/log/mentor-pi' \
    'Environment=XDG_RUNTIME_DIR=/run/mentor-pi' \
    'EnvironmentFile=/etc/default/mentor-pi' \
    'ReadWritePaths=/run/mentor-pi /var/log/mentor-pi'; do
  RequireLine "${AGENT_UNIT}" "${shared_line}"
  RequireLine "${SUPERVISOR_UNIT}" "${shared_line}"
done

RequireLine "${SHARED_DEFAULT}" 'ROS_DOMAIN_ID=0'
RequireLine "${SUPERVISOR_DEFAULT}" \
  'MENTOR_PI_CONTROLLER_CONFIG=/etc/mentor-pi/controller.yaml'
RequireLine "${SUPERVISOR_DEFAULT}" \
  'MENTOR_PI_ROS_SETUP=/opt/mentor_pi/host/setup.bash'
RequireLine "${SUPERVISOR_DEFAULT}" \
  'MENTOR_PI_SUPERVISOR_EXECUTABLE=/opt/mentor_pi/host/lib/mentor_pi_bringup/configuration_supervisor'
RejectPattern "${SHARED_DEFAULT}" 'MENTOR_PI_SERIAL_DEVICE|/dev/'
RejectPattern "${SUPERVISOR_DEFAULT}" 'MENTOR_PI_SERIAL_DEVICE|/dev/'
RejectPattern "${SUPERVISOR_DEFAULT}" '^ROS_DOMAIN_ID='
shared_value_count="$(grep -Evc '^[[:space:]]*(#|$)' "${SHARED_DEFAULT}")"
[[ "${shared_value_count}" == "1" ]] || \
  Fail "${SHARED_DEFAULT} must contain only the shared ROS_DOMAIN_ID value"

RequireLine "${AGENT_UNIT}" 'PartOf=mentor-pi-controller.target'
RequireLine "${AGENT_UNIT}" 'Requires=mentor-pi-runtime.service'
RequireLine "${AGENT_UNIT}" \
  'After=network.target mentor-pi-runtime.service'
RequireLine "${AGENT_UNIT}" 'SupplementaryGroups=mentor-pi-serial'
RejectPattern "${AGENT_UNIT}" 'dialout|MENTOR_PI_SERIAL_DEVICE'
RequireLine "${AGENT_UNIT}" \
  'ExecStartPre=/usr/bin/test -c /dev/mentor_pi_mcu'
RequireLine "${AGENT_UNIT}" \
  'ExecStart=/opt/mentor_pi/bin/mentor_pi_micro_ros_agent serial --dev /dev/mentor_pi_mcu --baudrate 1000000 -v4'
RequireLine "${AGENT_UNIT}" 'Restart=always'
RequireLine "${AGENT_UNIT}" 'RestartSec=1'
RequireLine "${AGENT_UNIT}" 'DevicePolicy=closed'
RequireLine "${AGENT_UNIT}" 'DeviceAllow=/dev/mentor_pi_mcu rw'

RequireLine "${SUPERVISOR_UNIT}" 'PartOf=mentor-pi-controller.target'
RequireLine "${SUPERVISOR_UNIT}" 'Requires=mentor-pi-runtime.service'
RequireLine "${SUPERVISOR_UNIT}" 'Wants=mentor-pi-agent.service'
RequireLine "${SUPERVISOR_UNIT}" \
  'After=network.target mentor-pi-runtime.service mentor-pi-agent.service'
RejectPattern "${SUPERVISOR_UNIT}" \
  '^(Requires|BindsTo|PartOf)=.*mentor-pi-agent\.service'
RequireLine "${SUPERVISOR_UNIT}" \
  'ExecStart=/opt/mentor_pi/host/lib/mentor_pi_bringup/run_configuration_supervisor'
RequireLine "${SUPERVISOR_UNIT}" \
  'EnvironmentFile=-/etc/default/mentor-pi-configuration-supervisor'
RequireLine "${SUPERVISOR_UNIT}" 'Restart=always'
RequireLine "${SUPERVISOR_UNIT}" 'RestartSec=2'
RequireLine "${SUPERVISOR_UNIT}" 'DevicePolicy=closed'
RejectPattern "${SUPERVISOR_UNIT}" '^DeviceAllow='
RejectPattern "${SUPERVISOR_UNIT}" \
  'MENTOR_PI_SERIAL_DEVICE|/dev/|--dev([[:space:]]|=)|serial[[:space:]]'
RejectPattern "${SUPERVISOR_LAUNCHER}" \
  'MENTOR_PI_SERIAL_DEVICE|/dev/|--dev([[:space:]]|=)|ros2[[:space:]]+run'
RequireLine "${AGENT_WRAPPER}" \
  'readonly SERIAL_LOCK="/run/mentor-pi/serial.lock"'
RequireLine "${AGENT_WRAPPER}" \
  '  if ! /usr/bin/flock -n "${serial_lock_fd}"; then'
RequireLine "${SUPERVISOR_LAUNCHER}" 'ValidateRosIdentity'
RequireLine "${AGENT_WRAPPER}" 'ValidateRosIdentity "$@"'
RejectPattern "${SUPERVISOR_LAUNCHER}" \
  'MENTOR_PI_REQUIRE_ROS_IDENTITY'
RejectPattern "${AGENT_WRAPPER}" 'MENTOR_PI_REQUIRE_ROS_IDENTITY'
RejectPattern "${AGENT_UNIT}" 'MENTOR_PI_REQUIRE_ROS_IDENTITY'
RejectPattern "${SUPERVISOR_UNIT}" 'MENTOR_PI_REQUIRE_ROS_IDENTITY'
RequireLine "${ASSET_INSTALLER}" \
  '  exec 8>/run/lock/mentor-pi-production-install.lock'
RequireLine "${ASSET_INSTALLER}" '  if ! /usr/bin/flock -n 8; then'
RequireLine "${RELEASE_PROMOTER}" \
  '  exec 9>"${OPT_ROOT}/.host-promotion.lock"'
RequireLine "${RELEASE_PROMOTER}" '  if ! /usr/bin/flock -n 9; then'
RequireLine "${ASSET_INSTALLER}" '  "${INSTALL_IDLE_GUARD}"'
RequireLine "${RELEASE_PROMOTER}" '  "${INSTALL_IDLE_GUARD}"'

RequireLine "${UDEV_TEMPLATE}" \
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", @MENTOR_PI_DEVICE_IDENTITY@, SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"'
RejectPattern "${UDEV_TEMPLATE}" 'GROUP="dialout"|MODE="0?777"'

RequireLine "${RUNTIME_UNIT}" 'RuntimeDirectory=mentor-pi'
RequireLine "${RUNTIME_UNIT}" 'RuntimeDirectoryMode=0700'
RequireLine "${RUNTIME_UNIT}" 'LogsDirectory=mentor-pi'
RequireLine "${RUNTIME_UNIT}" 'LogsDirectoryMode=0750'
RequireLine "${RUNTIME_UNIT}" 'RemainAfterExit=yes'
RequireLine "${RUNTIME_UNIT}" 'PartOf=mentor-pi-controller.target'
RequireLine "${CONTROLLER_TARGET}" \
  'Requires=mentor-pi-runtime.service'
RequireLine "${CONTROLLER_TARGET}" \
  'Wants=mentor-pi-agent.service mentor-pi-configuration-supervisor.service'
RequireLine "${CONTROLLER_TARGET}" 'WantedBy=multi-user.target'

TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-systemd.XXXXXX")"
readonly TEST_ROOT
Cleanup() {
  cmake -E remove_directory "${TEST_ROOT}"
}
trap Cleanup EXIT

cat >"${TEST_ROOT}/fake_systemctl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ "$#" == "4" ]]
[[ "$1" == "show" ]]
[[ "$2" == "--property=LoadState" ]]
[[ "$3" == "--property=ActiveState" ]]
[[ "$4" == "mentor-pi-controller.target" ]]
case "${MENTOR_PI_TEST_TARGET_STATE:-inactive}" in
  inactive)
    printf '%s\n' 'LoadState=loaded' 'ActiveState=inactive'
    ;;
  not-found)
    printf '%s\n' 'LoadState=not-found' 'ActiveState=inactive'
    ;;
  active)
    printf '%s\n' 'LoadState=loaded' 'ActiveState=active'
    ;;
  activating)
    printf '%s\n' 'LoadState=loaded' 'ActiveState=activating'
    ;;
  failed)
    printf '%s\n' 'LoadState=loaded' 'ActiveState=failed'
    ;;
  malformed)
    printf '%s\n' 'LoadState=loaded'
    ;;
  error)
    exit 5
    ;;
  *)
    printf '%s\n' 'LoadState=masked' 'ActiveState=inactive'
    ;;
esac
EOF
chmod +x "${TEST_ROOT}/fake_systemctl"
MENTOR_PI_TEST_TARGET_STATE=inactive \
  "${INSTALL_IDLE_GUARD}" --systemctl "${TEST_ROOT}/fake_systemctl" \
  >/dev/null
MENTOR_PI_TEST_TARGET_STATE=not-found \
  "${INSTALL_IDLE_GUARD}" --systemctl "${TEST_ROOT}/fake_systemctl" \
  >/dev/null
for unsafe_state in active activating failed malformed error unexpected; do
  ExpectFailure env MENTOR_PI_TEST_TARGET_STATE="${unsafe_state}" \
    "${INSTALL_IDLE_GUARD}" --systemctl "${TEST_ROOT}/fake_systemctl"
done

cat >"${TEST_ROOT}/setup.bash" <<'EOF'
export MENTOR_PI_TEST_SETUP_SOURCED=1
EOF
cat >"${TEST_ROOT}/fake_supervisor" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
{
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-}"
  printf 'ROS_LOG_DIR=%s\n' "${ROS_LOG_DIR:-}"
  printf 'XDG_RUNTIME_DIR=%s\n' "${XDG_RUNTIME_DIR:-}"
  printf 'SETUP_SOURCED=%s\n' "${MENTOR_PI_TEST_SETUP_SOURCED:-}"
  for argument in "$@"; do
    printf 'ARG=%s\n' "${argument}"
  done
} >"${MENTOR_PI_TEST_OUTPUT:?}"
EOF
chmod +x "${TEST_ROOT}/fake_supervisor"
printf '%s\n' 'test configuration' >"${TEST_ROOT}/controller.yaml"

ROS_DOMAIN_ID=37 \
ROS_LOG_DIR=/var/log/mentor-pi \
XDG_RUNTIME_DIR=/run/mentor-pi \
MENTOR_PI_SERIAL_DEVICE=/dev/must-not-be-forwarded \
MENTOR_PI_ROS_SETUP="${TEST_ROOT}/setup.bash" \
MENTOR_PI_SUPERVISOR_EXECUTABLE="${TEST_ROOT}/fake_supervisor" \
MENTOR_PI_CONTROLLER_CONFIG="${TEST_ROOT}/controller.yaml" \
MENTOR_PI_TEST_OUTPUT="${TEST_ROOT}/arguments.txt" \
  "${SUPERVISOR_LAUNCHER}"

RequireLine "${TEST_ROOT}/arguments.txt" 'ROS_DOMAIN_ID=37'
RequireLine "${TEST_ROOT}/arguments.txt" \
  'ROS_LOG_DIR=/var/log/mentor-pi'
RequireLine "${TEST_ROOT}/arguments.txt" \
  'XDG_RUNTIME_DIR=/run/mentor-pi'
RequireLine "${TEST_ROOT}/arguments.txt" 'SETUP_SOURCED=1'
RequireLine "${TEST_ROOT}/arguments.txt" 'ARG=--ros-args'
RequireLine "${TEST_ROOT}/arguments.txt" 'ARG=--params-file'
RequireLine "${TEST_ROOT}/arguments.txt" \
  "ARG=${TEST_ROOT}/controller.yaml"
RejectPattern "${TEST_ROOT}/arguments.txt" \
  'must-not-be-forwarded|MENTOR_PI_SERIAL_DEVICE|--dev([[:space:]]|=)'

ExpectFailure env \
  ROS_DOMAIN_ID=999 \
  ROS_LOG_DIR=/var/log/mentor-pi \
  XDG_RUNTIME_DIR=/run/mentor-pi \
  MENTOR_PI_ROS_SETUP="${TEST_ROOT}/setup.bash" \
  MENTOR_PI_SUPERVISOR_EXECUTABLE="${TEST_ROOT}/fake_supervisor" \
  MENTOR_PI_CONTROLLER_CONFIG="${TEST_ROOT}/controller.yaml" \
  MENTOR_PI_TEST_OUTPUT="${TEST_ROOT}/invalid-domain.txt" \
  "${SUPERVISOR_LAUNCHER}"
ExpectFailure env -u ROS_DOMAIN_ID \
  ROS_LOG_DIR=/var/log/mentor-pi \
  XDG_RUNTIME_DIR=/run/mentor-pi \
  MENTOR_PI_ROS_SETUP="${TEST_ROOT}/setup.bash" \
  MENTOR_PI_SUPERVISOR_EXECUTABLE="${TEST_ROOT}/fake_supervisor" \
  MENTOR_PI_CONTROLLER_CONFIG="${TEST_ROOT}/controller.yaml" \
  MENTOR_PI_TEST_OUTPUT="${TEST_ROOT}/missing-domain.txt" \
  "${SUPERVISOR_LAUNCHER}"

mkdir -p "${TEST_ROOT}/sys/class/tty/ttyUSB0" \
  "${TEST_ROOT}/sys/class/tty/ttyUSB1"
cat >"${TEST_ROOT}/fake_udevadm" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
node=""
for argument in "$@"; do
  case "${argument}" in
    --name=*) node="${argument#--name=}" ;;
  esac
done
case "$(basename "${node}")" in
  ttyUSB0)
    printf '%s\n' \
      'ID_VENDOR_ID=1a86' \
      'ID_MODEL_ID=55d4' \
      'ID_SERIAL_SHORT=RRCLITE-A' \
      'ID_PATH=pci-0000:00:14.0-usb-0:1:1.0'
    ;;
  ttyUSB1)
    serial="RRCLITE-B"
    if [[ "${MENTOR_PI_TEST_DUPLICATE_IDENTITY:-0}" == "1" ]]; then
      serial="RRCLITE-A"
    fi
    printf '%s\n' \
      'ID_VENDOR_ID=1a86' \
      'ID_MODEL_ID=55d4' \
      "ID_SERIAL_SHORT=${serial}" \
      'ID_PATH=pci-0000:00:14.0-usb-0:2:1.0'
    ;;
  *) exit 1 ;;
esac
EOF
chmod +x "${TEST_ROOT}/fake_udevadm"

readonly INSTALL_ROOT="${TEST_ROOT}/installed"
RunAssetInstaller() {
  MENTOR_PI_DEPLOYMENT_TEST_ROOT="${INSTALL_ROOT}" \
  MENTOR_PI_UDEVADM="${TEST_ROOT}/fake_udevadm" \
  MENTOR_PI_SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty" \
    "${ASSET_INSTALLER}" "$@" \
      --ros-domain-id 37 \
      --identity-kind serial \
      --identity-value RRCLITE-A \
      --device /dev/ttyUSB0 \
      --share-dir "${SOURCE_ROOT}"
}

RunAssetInstaller --mode first-install
readonly INSTALLED_CONTROLLER="${INSTALL_ROOT}/etc/mentor-pi/controller.yaml"
readonly INSTALLED_SHARED="${INSTALL_ROOT}/etc/default/mentor-pi"
readonly INSTALLED_SUPERVISOR_DEFAULT="${INSTALL_ROOT}/etc/default/mentor-pi-configuration-supervisor"
readonly INSTALLED_RULE="${INSTALL_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules"
RequireLine "${INSTALLED_SHARED}" 'ROS_DOMAIN_ID=37'
RequireLine "${INSTALLED_RULE}" \
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="RRCLITE-A", SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"'
RejectPattern "${INSTALLED_RULE}" '@MENTOR_PI|GROUP="dialout"'
[[ ! -e "${INSTALL_ROOT}/etc/default/mentor-pi-agent" ]] || \
  Fail "obsolete Agent-specific defaults were installed"

printf '%s\n' '# SITE_REVIEWED_VALUE' >>"${INSTALLED_CONTROLLER}"
printf '%s\n' '# SITE_PATH_OVERRIDE' >>"${INSTALLED_SUPERVISOR_DEFAULT}"
readonly controller_digest="$(cksum "${INSTALLED_CONTROLLER}")"
readonly supervisor_default_digest="$(cksum "${INSTALLED_SUPERVISOR_DEFAULT}")"
ExpectFailure RunAssetInstaller --mode first-install
RunAssetInstaller --mode upgrade
[[ "$(cksum "${INSTALLED_CONTROLLER}")" == "${controller_digest}" ]] || \
  Fail "upgrade overwrote reviewed controller configuration"
[[ "$(cksum "${INSTALLED_SUPERVISOR_DEFAULT}")" == \
    "${supervisor_default_digest}" ]] || \
  Fail "upgrade overwrote reviewed supervisor defaults"

cp "${INSTALLED_SUPERVISOR_DEFAULT}" \
  "${TEST_ROOT}/supervisor-default.backup"
printf '%s\n' 'ROS_DOMAIN_ID=37' >>"${INSTALLED_SUPERVISOR_DEFAULT}"
ExpectFailure RunAssetInstaller --mode upgrade
mv "${TEST_ROOT}/supervisor-default.backup" \
  "${INSTALLED_SUPERVISOR_DEFAULT}"

ExpectFailure env \
  MENTOR_PI_DEPLOYMENT_TEST_ROOT="${INSTALL_ROOT}" \
  MENTOR_PI_UDEVADM="${TEST_ROOT}/fake_udevadm" \
  MENTOR_PI_SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty" \
  "${ASSET_INSTALLER}" --mode upgrade \
    --ros-domain-id 38 \
    --identity-kind serial \
    --identity-value RRCLITE-A \
    --device /dev/ttyUSB0 \
    --share-dir "${SOURCE_ROOT}"
ExpectFailure env \
  MENTOR_PI_TEST_DUPLICATE_IDENTITY=1 \
  MENTOR_PI_DEPLOYMENT_TEST_ROOT="${INSTALL_ROOT}" \
  MENTOR_PI_UDEVADM="${TEST_ROOT}/fake_udevadm" \
  MENTOR_PI_SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty" \
  "${ASSET_INSTALLER}" --mode upgrade \
    --ros-domain-id 37 \
    --identity-kind serial \
    --identity-value RRCLITE-A \
    --device /dev/ttyUSB0 \
    --share-dir "${SOURCE_ROOT}"

readonly ID_PATH_ROOT="${TEST_ROOT}/installed-id-path"
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${ID_PATH_ROOT}" \
MENTOR_PI_UDEVADM="${TEST_ROOT}/fake_udevadm" \
MENTOR_PI_SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty" \
  "${ASSET_INSTALLER}" --mode first-install \
    --ros-domain-id 0 \
    --identity-kind id-path \
    --identity-value pci-0000:00:14.0-usb-0:2:1.0 \
    --device /dev/ttyUSB1 \
    --share-dir "${SOURCE_ROOT}"
RequireLine \
  "${ID_PATH_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules" \
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:2:1.0", SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"'

MakeStagedRelease() {
  local stage="$1"
  local marker="$2"
  mkdir -p "${stage}/lib/mentor_pi_bringup" \
    "${stage}/share/ament_index/resource_index/packages" \
    "${stage}/share/mentor_pi_interfaces" \
    "${stage}/share/ros_package_schema" \
    "${stage}/share/mentor_pi_bringup/config" \
    "${stage}/share/mentor_pi_bringup/systemd" \
    "${stage}/share/mentor_pi_bringup/udev"
  printf '%s\n' '# staged ROS setup' >"${stage}/setup.bash"
  printf '%s\n' 'mentor_pi_bringup' \
    >"${stage}/share/ament_index/resource_index/packages/mentor_pi_bringup"
  printf '%s\n' 'mentor_pi_interfaces' \
    >"${stage}/share/ament_index/resource_index/packages/mentor_pi_interfaces"
  printf '%s\n' \
    '<package format="3"><name>mentor_pi_interfaces</name></package>' \
    >"${stage}/share/mentor_pi_interfaces/package.xml"
  cp "${SOURCE_ROOT}/../ros_package_schema/package_common.xsd" \
    "${SOURCE_ROOT}/../ros_package_schema/package_format3.xsd" \
    "${stage}/share/ros_package_schema/"
  for library in \
      libmentor_pi_interfaces__rosidl_generator_c.so \
      libmentor_pi_interfaces__rosidl_typesupport_cpp.so \
      libmentor_pi_interfaces__rosidl_typesupport_fastrtps_cpp.so; do
    printf '%s\n' "${library} fixture" >"${stage}/lib/${library}"
  done
  cat >"${stage}/lib/mentor_pi_bringup/configuration_supervisor" <<EOF
#!/usr/bin/env bash
echo "${marker}"
EOF
  chmod +x "${stage}/lib/mentor_pi_bringup/configuration_supervisor"
  cp "${SUPERVISOR_LAUNCHER}" \
    "${stage}/lib/mentor_pi_bringup/run_configuration_supervisor"
  cp "${DIAGNOSTIC_CAPTURE}" \
    "${stage}/lib/mentor_pi_bringup/capture_board_diagnostics"
  cp "${ASSET_INSTALLER}" \
    "${stage}/lib/mentor_pi_bringup/install_production_assets"
  cp "${RELEASE_PROMOTER}" \
    "${stage}/lib/mentor_pi_bringup/promote_host_release"
  cp "${INSTALL_IDLE_GUARD}" \
    "${stage}/lib/mentor_pi_bringup/require_controller_target_inactive"
  chmod +x "${stage}/lib/mentor_pi_bringup/"*
  cp "${SOURCE_ROOT}/config/controller.yaml" \
    "${stage}/share/mentor_pi_bringup/config/controller.yaml"
  cp "${AGENT_UNIT}" \
    "${stage}/share/mentor_pi_bringup/systemd/mentor-pi-agent.service"
  cp "${SUPERVISOR_DEFAULT}" \
    "${stage}/share/mentor_pi_bringup/systemd/mentor-pi-configuration-supervisor.default"
  cp "${RUNTIME_UNIT}" \
    "${stage}/share/mentor_pi_bringup/systemd/mentor-pi-runtime.service"
  cp "${SUPERVISOR_UNIT}" \
    "${stage}/share/mentor_pi_bringup/systemd/mentor-pi-configuration-supervisor.service"
  cp "${CONTROLLER_TARGET}" \
    "${stage}/share/mentor_pi_bringup/systemd/mentor-pi-controller.target"
  cp "${UDEV_TEMPLATE}" \
    "${stage}/share/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in"
}

readonly PROMOTION_ROOT="${TEST_ROOT}/promotion"
MakeStagedRelease "${TEST_ROOT}/stage-r1" r1
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" \
    --staged-prefix "${TEST_ROOT}/stage-r1" --release-id r1
readonly ACTIVE_HOST="${PROMOTION_ROOT}/opt/mentor_pi/host"
[[ -L "${ACTIVE_HOST}" ]] || Fail "host activation is not a symbolic link"
[[ "$(readlink "${ACTIVE_HOST}")" == \
    "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r1" ]] || \
  Fail "r1 was not activated"

MakeStagedRelease "${TEST_ROOT}/stage-r2" r2
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" \
    --staged-prefix "${TEST_ROOT}/stage-r2" --release-id r2
[[ -d "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r1" ]] || \
  Fail "promotion removed rollback release r1"
[[ "$(readlink "${ACTIVE_HOST}")" == \
    "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r2" ]] || \
  Fail "r2 was not atomically activated"
ExpectFailure env MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" \
    --staged-prefix "${TEST_ROOT}/stage-r2" --release-id r2
MakeStagedRelease "${TEST_ROOT}/stage-incomplete" incomplete
cmake -E remove \
  "${TEST_ROOT}/stage-incomplete/share/mentor_pi_bringup/systemd/mentor-pi-controller.target"
ExpectFailure env MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" \
    --staged-prefix "${TEST_ROOT}/stage-incomplete" \
    --release-id incomplete

readonly -a REQUIRED_INTERFACE_RUNTIME=(
  'share/ament_index/resource_index/packages/mentor_pi_bringup'
  'share/ament_index/resource_index/packages/mentor_pi_interfaces'
  'share/mentor_pi_interfaces/package.xml'
  'share/ros_package_schema/package_common.xsd'
  'share/ros_package_schema/package_format3.xsd'
  'lib/libmentor_pi_interfaces__rosidl_generator_c.so'
  'lib/libmentor_pi_interfaces__rosidl_typesupport_cpp.so'
  'lib/libmentor_pi_interfaces__rosidl_typesupport_fastrtps_cpp.so'
)
for missing_index in "${!REQUIRED_INTERFACE_RUNTIME[@]}"; do
  missing_path="${REQUIRED_INTERFACE_RUNTIME[missing_index]}"
  stage_name="stage-missing-runtime-${missing_index}"
  release_name="missing-runtime-${missing_index}"
  MakeStagedRelease "${TEST_ROOT}/${stage_name}" "${release_name}"
  cmake -E remove "${TEST_ROOT}/${stage_name}/${missing_path}"
  ExpectFailure env MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
    "${RELEASE_PROMOTER}" \
      --staged-prefix "${TEST_ROOT}/${stage_name}" \
      --release-id "${release_name}"
done
readonly -a REQUIRED_OPERATOR_TOOLS=(
  'lib/mentor_pi_bringup/capture_board_diagnostics'
  'lib/mentor_pi_bringup/require_controller_target_inactive'
)
for missing_index in "${!REQUIRED_OPERATOR_TOOLS[@]}"; do
  missing_path="${REQUIRED_OPERATOR_TOOLS[missing_index]}"
  stage_name="stage-missing-operator-${missing_index}"
  release_name="missing-operator-${missing_index}"
  MakeStagedRelease "${TEST_ROOT}/${stage_name}" "${release_name}"
  cmake -E remove "${TEST_ROOT}/${stage_name}/${missing_path}"
  ExpectFailure env MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
    "${RELEASE_PROMOTER}" \
      --staged-prefix "${TEST_ROOT}/${stage_name}" \
      --release-id "${release_name}"
done
MakeStagedRelease "${TEST_ROOT}/stage-external-link" linked
ln -s "${TEST_ROOT}/stage-r1/setup.bash" \
  "${TEST_ROOT}/stage-external-link/external-setup-link"
ExpectFailure env MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" \
    --staged-prefix "${TEST_ROOT}/stage-external-link" \
    --release-id external-link
# Operator-only tools were not present in releases installed before this
# hardening. The current promoter's guard has already run, so preserve the
# documented emergency rollback path to an otherwise-complete legacy release.
cmake -E remove \
  "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r1/lib/mentor_pi_bringup/capture_board_diagnostics" \
  "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r1/lib/mentor_pi_bringup/require_controller_target_inactive"
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${PROMOTION_ROOT}" \
  "${RELEASE_PROMOTER}" --activate-release r1
[[ "$(readlink "${ACTIVE_HOST}")" == \
    "${PROMOTION_ROOT}/opt/mentor_pi/releases/host/r1" ]] || \
  Fail "rollback did not reactivate r1"

RequireLine "${SOURCE_ROOT}/CMakeLists.txt" \
  '    scripts/capture_board_diagnostics'
RequireLine "${SOURCE_ROOT}/CMakeLists.txt" \
  '    systemd/install_production_assets'
RequireLine "${SOURCE_ROOT}/CMakeLists.txt" \
  '    systemd/promote_host_release'
RequireLine "${SOURCE_ROOT}/CMakeLists.txt" \
  '    systemd/require_controller_target_inactive'
RequireLine "${SOURCE_ROOT}/CMakeLists.txt" \
  '    systemd/run_configuration_supervisor'

echo "systemd deployment tests passed"
