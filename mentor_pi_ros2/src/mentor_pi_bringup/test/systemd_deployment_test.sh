#!/usr/bin/env bash

set -euo pipefail

readonly SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly SYSTEMD_ROOT="${SOURCE_ROOT}/systemd"
readonly RUNTIME_UNIT="${SYSTEMD_ROOT}/mentor-pi-runtime.service"
readonly TARGET_UNIT="${SYSTEMD_ROOT}/mentor-pi-controller.target"
readonly LAUNCHER="${SYSTEMD_ROOT}/run_production_container"
readonly INSTALLER="${SYSTEMD_ROOT}/install_production_assets"
readonly PROMOTER="${SYSTEMD_ROOT}/promote_host_release"

Fail() {
  echo "systemd deployment contract failure: $*" >&2
  exit 1
}

RequireLiteral() {
  local path="$1"
  local text="$2"
  grep -Fq -- "${text}" "${path}" || Fail "${path} is missing: ${text}"
}

RequireRegex() {
  local path="$1"
  local pattern="$2"
  grep -Eq -- "${pattern}" "${path}" || Fail "${path} is missing pattern: ${pattern}"
}

for path in "${RUNTIME_UNIT}" "${TARGET_UNIT}" "${LAUNCHER}" \
    "${INSTALLER}" "${PROMOTER}"; do
  [[ -f "${path}" ]] || Fail "required deployment asset is missing: ${path}"
done
[[ -x "${LAUNCHER}" && -x "${INSTALLER}" && -x "${PROMOTER}" ]] || \
  Fail "deployment helpers must be executable"

for removed in mentor-pi-agent.service \
    mentor-pi-configuration-supervisor.service \
    mentor-pi-configuration-supervisor.default; do
  [[ ! -e "${SYSTEMD_ROOT}/${removed}" ]] || \
    Fail "obsolete native deployment asset remains: ${removed}"
done

RequireLiteral "${TARGET_UNIT}" 'Requires=mentor-pi-runtime.service'
if grep -Eq 'mentor-pi-(agent|configuration-supervisor)[.]service' \
    "${TARGET_UNIT}"; then
  Fail "controller target still depends on a separate native process"
fi

for text in \
    'Requires=docker.service' \
    'ExecStart=/opt/mentor_pi/host/lib/mentor_pi_bringup/run_production_container' \
    'ExecStop=-/usr/bin/docker stop --time 5 mentor-pi-production' \
    'Restart=always' \
    'ProtectSystem=strict'; do
  RequireLiteral "${RUNTIME_UNIT}" "${text}"
done

for text in \
    '--network host' \
    '--platform "linux/${native_architecture}"' \
    '--device "${DEVICE}:${DEVICE}:rwm"' \
    '--volume /run/udev:/run/udev:ro' \
    '--cap-drop ALL' \
    '--security-opt no-new-privileges' \
    '--read-only' \
    '--tmpfs /tmp:rw,nosuid,nodev,mode=1777,size=256m' \
    '--entrypoint /bin/bash' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/mentor_pi/micro_ros_agent/local_setup.bash' \
    'source /opt/mentor_pi/host/setup.bash' \
    'exec ros2 launch mentor_pi_bringup controller.launch.py'; do
  RequireLiteral "${LAUNCHER}" "${text}"
done
RequireRegex "${LAUNCHER}" 'sha256:\[0-9a-f\]\{64\}'
for text in \
    "dpkg --print-architecture" \
    "--format '{{.Os}}/{{.Architecture}}'" \
    'install -d -o "${service_uid}" -g "${service_gid}" -m 0750' \
    "stat -Lc '%u:%g:%a' /var/log/mentor-pi"; do
  RequireLiteral "${LAUNCHER}" "${text}"
done

RequireLiteral "${INSTALLER}" '--runtime-image sha256:HEX'
RequireLiteral "${INSTALLER}" 'dpkg --print-architecture'
RequireLiteral "${INSTALLER}" "--format '{{.Os}}/{{.Architecture}}'"
RequireLiteral "${INSTALLER}" 'mentor-pi-runtime.service'
RequireLiteral "${INSTALLER}" 'mentor-pi-controller.target'
RequireLiteral "${INSTALLER}" 'for obsolete_unit in mentor-pi-agent.service'
RequireLiteral "${PROMOTER}" 'command -v docker'
RequireLiteral "${PROMOTER}" 'run_production_container'

if grep -Eq 'mentor-pi-(agent|configuration-supervisor)[.]service' \
    "${SOURCE_ROOT}/CMakeLists.txt"; then
  Fail "CMake still installs obsolete native service units"
fi
RequireLiteral "${SOURCE_ROOT}/CMakeLists.txt" 'systemd/run_production_container'

bash -n "${LAUNCHER}" "${INSTALLER}" "${PROMOTER}"
echo "Docker production systemd contract tests passed."
