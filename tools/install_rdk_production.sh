#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="${MENTOR_PI_PRODUCTION_TEST_ROOT:-}"
if [[ -n "${TEST_ROOT}" && ("${TEST_ROOT}" != /* || "${TEST_ROOT}" == /) ]]; then
  echo "MENTOR_PI_PRODUCTION_TEST_ROOT must be an absolute non-root path" >&2
  exit 2
fi
if [[ -n "${MENTOR_PI_RDK_RECEIVER:-}" && -z "${TEST_ROOT}" ]]; then
  echo "MENTOR_PI_RDK_RECEIVER requires MENTOR_PI_PRODUCTION_TEST_ROOT" >&2
  exit 2
fi
readonly RECEIVER="${MENTOR_PI_RDK_RECEIVER:-${SCRIPT_DIR}/receive_rdk_handoff.sh}"
readonly OPT_ROOT="${TEST_ROOT}/opt/mentor_pi"
readonly ETC_SYSTEMD="${TEST_ROOT}/etc/systemd/system"

mode=""
device=""
ros_domain_id=""
identity_kind=""
identity_value=""
handoff=""

Fail() {
  echo "RDK production installation error: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: install_rdk_production.sh --mode first-install|upgrade \
  --device /dev/DEVICE --ros-domain-id 0..232 \
  --identity-kind serial|id-path --identity-value VALUE \
  [--handoff ABSOLUTE_RDK_HANDOFF]
EOF
  exit 2
}

ReadSingleValue() {
  local file="$1" key="$2" description="$3"
  local count value
  count="$(grep -Ec "^${key}=" "${file}" || true)"
  [[ "${count}" == 1 ]] || \
    Fail "${description} must contain exactly one ${key}= entry"
  value="$(sed -n "s/^${key}=//p" "${file}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "${description} contains an invalid ${key}= entry"
  printf '%s' "${value}"
}

while (($# > 0)); do
  case "$1" in
    --mode) mode="${2:-}"; shift 2 ;;
    --device) device="${2:-}"; shift 2 ;;
    --ros-domain-id) ros_domain_id="${2:-}"; shift 2 ;;
    --identity-kind) identity_kind="${2:-}"; shift 2 ;;
    --identity-value) identity_value="${2:-}"; shift 2 ;;
    --handoff) handoff="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done
[[ "${mode}" == first-install || "${mode}" == upgrade ]] || Usage
[[ "${device}" == /dev/* ]] || Usage
[[ "${ros_domain_id}" =~ ^(0|[1-9][0-9]{0,2})$ ]] && \
  ((ros_domain_id <= 232)) || Usage
[[ "${identity_kind}" == serial || "${identity_kind}" == id-path ]] || Usage
[[ "${identity_value}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || Usage
[[ -x "${RECEIVER}" ]] || Fail "RDK handoff receiver is unavailable"
for command in docker realpath sha256sum sudo systemctl systemd-analyze; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is required"
done

receiver_args=()
if [[ -n "${handoff}" ]]; then
  receiver_args+=(--handoff "${handoff}")
fi
readonly bundle="$("${RECEIVER}" "${receiver_args[@]}")"
readonly bundle_name="$(basename "${bundle}")"
readonly host_handoff="${bundle}/host-handoff"
readonly host_metadata="${host_handoff}/HOST-HANDOFF.txt"
[[ -d "${host_handoff}" && ! -L "${host_handoff}" && \
   -f "${host_metadata}" && ! -L "${host_metadata}" ]] || \
  Fail "host handoff is missing or symbolic"
(cd "${host_handoff}" && sha256sum --check --strict SHA256SUMS >/dev/null) || \
  Fail "host handoff checksum verification failed"
readonly symlink_manifest="${host_handoff}/SYMLINKS.txt"
[[ -f "${symlink_manifest}" && ! -L "${symlink_manifest}" ]] || \
  Fail "host handoff symlink manifest is missing or symbolic"
while IFS=$'\t' read -r relative_path expected_target extra; do
  [[ -z "${extra:-}" && -n "${relative_path}" && -n "${expected_target}" && \
     "${relative_path}" != /* && "/${relative_path}/" != *'/../'* && \
     "${expected_target}" != /* ]] || \
    Fail "host handoff symlink manifest is malformed"
  resolved_target="$(realpath -m \
    "$(dirname "${host_handoff}/${relative_path}")/${expected_target}")"
  [[ "${resolved_target}" == "${host_handoff}/"* && \
     -e "${resolved_target}" ]] || \
    Fail "host handoff symlink escapes its verified bundle: ${relative_path}"
  [[ -L "${host_handoff}/${relative_path}" && \
     "$(readlink -- "${host_handoff}/${relative_path}")" == \
       "${expected_target}" ]] || \
    Fail "host handoff symlink verification failed for ${relative_path}"
done <"${symlink_manifest}"

readonly release_id="$(ReadSingleValue \
  "${host_metadata}" release_id "host handoff metadata")"
readonly runtime_image_id="$(ReadSingleValue \
  "${host_metadata}" runtime_image_id "host handoff metadata")"
[[ "${release_id}" == "${bundle_name}" ]] || \
  Fail "host handoff release ID does not match ${bundle_name}"
[[ "${runtime_image_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "host handoff runtime image ID is malformed"
readonly runtime_archive="${host_handoff}/runtime-image/mentor-pi-runtime.tar"
[[ -f "${runtime_archive}" && ! -L "${runtime_archive}" ]] || \
  Fail "runtime image archive is missing or symbolic"

if systemctl is-active --quiet mentor-pi-controller.target; then
  echo "Stopping the active Mentor Pi production target before installation."
  sudo systemctl stop mentor-pi-controller.target
fi

docker load --input "${runtime_archive}"
[[ "$(docker image inspect "${runtime_image_id}" --format '{{.Id}}')" == \
   "${runtime_image_id}" ]] || Fail "loaded runtime image ID does not match"
[[ "$(docker image inspect "${runtime_image_id}" \
   --format '{{.Os}}/{{.Architecture}}')" == linux/arm64 ]] || \
  Fail "loaded runtime image is not linux/arm64"

readonly agent_source="${host_handoff}/agent"
readonly agent_metadata="${agent_source}/AGENT-BUILD-METADATA.txt"
readonly agent_sha="$(ReadSingleValue \
  "${agent_metadata}" executable_sha256 "Agent metadata")"
[[ "${agent_sha}" =~ ^[0-9a-f]{64}$ ]] || Fail "Agent hash is malformed"
readonly agent_destination="${OPT_ROOT}/releases/agent/${release_id}"
readonly agent_executable="${agent_destination}/lib/micro_ros_agent/micro_ros_agent"
if ! sudo test -d "${agent_destination}"; then
  sudo install -d "${agent_destination}"
  sudo cp -a "${agent_source}/." "${agent_destination}/"
fi
printf '%s  %s\n' "${agent_sha}" "${agent_executable}" | \
  sudo sha256sum --check --strict - >/dev/null
readonly active_agent="${OPT_ROOT}/micro_ros_agent"
if sudo test -e "${active_agent}" && ! sudo test -L "${active_agent}"; then
  Fail "${active_agent} exists and is not a managed symbolic link"
fi
sudo ln -sfn "${agent_destination}" "${active_agent}"

readonly promoter="${host_handoff}/host/lib/mentor_pi_bringup/promote_host_release"
[[ -x "${promoter}" && ! -L "${promoter}" ]] || \
  Fail "host release promoter is missing or symbolic"
if sudo test -d "${OPT_ROOT}/releases/host/${release_id}"; then
  sudo "${promoter}" --activate-release "${release_id}"
else
  sudo "${promoter}" --staged-prefix "${host_handoff}/host" \
    --release-id "${release_id}"
fi

readonly installer="${OPT_ROOT}/host/lib/mentor_pi_bringup/install_production_assets"
[[ -x "${installer}" && ! -L "${installer}" ]] || \
  Fail "installed production asset helper is missing or symbolic"
sudo "${installer}" --mode "${mode}" --ros-domain-id "${ros_domain_id}" \
  --identity-kind "${identity_kind}" --identity-value "${identity_value}" \
  --device "${device}" --runtime-image "${runtime_image_id}"
sudo systemd-analyze verify \
  "${ETC_SYSTEMD}/mentor-pi-runtime.service" \
  "${ETC_SYSTEMD}/mentor-pi-controller.target"

echo "Installed Mentor Pi production release ${release_id}."
echo "Start it with: sudo systemctl enable --now mentor-pi-controller.target"
