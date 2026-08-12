#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${TEST_DIR}/.." && pwd)"

bash -n "${COMPONENT_ROOT}"/tools/*.sh
grep -Eq '^find-device:' "${COMPONENT_ROOT}/Makefile"
grep -Fq 'SERIAL_ACCESS_HELPER=' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
! grep -Fq 'ROS_DOMAIN_ID ?= 0' "${COMPONENT_ROOT}/Makefile"
grep -Fq 'sudo make install-service ROS_DOMAIN_ID=0' \
  "${COMPONENT_ROOT}/Makefile"
grep -Fq 'pass ROS_DOMAIN_ID=<0..232> to make install-service' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
grep -Fq 'RELEASE_ID:-${build_tree_sha:0:16}' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
! grep -Fq 'RELEASE_ID:-${executable_sha:0:16}' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
! grep -Fq 'ROS_DOMAIN_ID:-0' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
grep -Fq 'automatic CH9102F discovery' \
  "${COMPONENT_ROOT}/tools/configure_serial_access.sh"
grep -Fqx 'ros_distro=humble' "${COMPONENT_ROOT}/sources.lock"
grep -Fq 'MENTOR_PI_RRCLITE_AUTORESET' \
  "${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch"
normal_boot_sequence="$(tr '\n' ' ' <"${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch")"
[[ "${normal_boot_sequence}" =~ bits\ =\ TIOCM_RTS\;.*TIOCMBIS.*bits\ =\ TIOCM_DTR\;.*TIOCMBIC.*milliseconds\(100\).*bits\ =\ TIOCM_RTS\;.*TIOCMBIC.*milliseconds\(100\) ]] || {
  echo "micro-ROS Agent patch lost the separate normal-boot RTS/DTR sequence" >&2
  exit 1
}
grep -Fq 'User=mentor-pi' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fq '/opt/mentor_pi/agent/current/bin/mentor-pi-agent' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fq 'source "${ROS_SETUP}"' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent"
grep -Fq 'source "${LOCAL_SETUP}"' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent"
grep -Fqx 'Restart=always' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fqx 'StartLimitIntervalSec=0' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fqx 'ProtectClock=true' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fqx 'DeviceAllow=char-ttyACM rw' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in" || {
  echo "Agent service does not allow its ttyACM transport device" >&2
  exit 1
}
grep -Fqx 'Environment=FASTDDS_BUILTIN_TRANSPORTS=UDPv4' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in" || {
  echo "Agent service does not disable cross-user Fast DDS shared memory" >&2
  exit 1
}
grep -Fqx 'Environment=MENTOR_PI_RRCLITE_AUTORESET=1' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
! grep -Fq 'EnvironmentFile=' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service.in"
grep -Fq 'ATTRS{idVendor}=="1a86"' \
  "${COMPONENT_ROOT}/udev/99-mentor-pi-mcu.rules.in"
if grep -R -n -E 'docker (run|build|exec|pull|load)|mentor-pi-runtime|configuration_supervisor' \
    "${COMPONENT_ROOT}/Makefile" "${COMPONENT_ROOT}/tools" \
    "${COMPONENT_ROOT}/systemd"; then
  echo "Agent component contains forbidden runtime/container coupling" >&2
  exit 1
fi
grep -Fq '/.dockerenv' "${COMPONENT_ROOT}/tools/install_service.sh" || {
  echo "Agent service installation does not reject the Dev Container" >&2
  exit 1
}

installer="${COMPONENT_ROOT}/tools/install_service.sh"
release_test_root="$(mktemp -d)"
trap 'rm -rf -- "${release_test_root}"' EXIT
rendered_service="${release_test_root}/mentor-pi-agent.service"
bash -c 'source "$1"; RenderServiceUnit "$2" 37' bash \
  "${installer}" "${rendered_service}"
grep -Fqx 'Environment=ROS_DOMAIN_ID=37' "${rendered_service}"
grep -Fqx 'Environment=MENTOR_PI_RRCLITE_AUTORESET=1' "${rendered_service}"
! grep -Fq '@ROS_DOMAIN_ID@' "${rendered_service}"
! grep -Fq '/etc/mentor-pi/agent.env' "${rendered_service}"
expected_release="${release_test_root}/expected"
mkdir -p "${expected_release}/lib/micro_ros_agent" \
  "${expected_release}/share/micro_ros_agent/hook" "${expected_release}/bin"
printf 'build=verified\n' >"${expected_release}/AGENT-BUILD-METADATA.txt"
printf '#!/bin/sh\nexit 0\n' >"${expected_release}/lib/micro_ros_agent/micro_ros_agent"
cp "${expected_release}/lib/micro_ros_agent/micro_ros_agent" \
  "${expected_release}/bin/mentor-pi-agent"
printf 'runtime library\n' >"${expected_release}/lib/libmentor_pi_runtime.so"
printf 'hook\n' >"${expected_release}/share/micro_ros_agent/hook/runtime.dsv"
printf 'source this release\n' >"${expected_release}/local_setup.sh"
ln -s libmentor_pi_runtime.so "${expected_release}/lib/libmentor_pi_runtime.so.1"
chmod 0755 "${expected_release}/lib/micro_ros_agent/micro_ros_agent" \
  "${expected_release}/bin/mentor-pi-agent"
find -P "${expected_release}" \( -type f -o -type d \) -exec chmod go-w {} +
current_metadata="${expected_release}/AGENT-BUILD-METADATA.txt"
executable_sha="$(sha256sum "${expected_release}/lib/micro_ros_agent/micro_ros_agent" | awk '{print $1}')"
launcher_sha="$(sha256sum "${expected_release}/bin/mentor-pi-agent" | awk '{print $1}')"
tree_sha="$(bash -c 'source "$1"; TreeDigest "$2"' bash "${installer}" "${expected_release}")"
test_uid="$(id -u)"
test_gid="$(id -g)"

verify_release() {
  bash -c 'source "$1"; VerifyExistingRelease "$2" "$3" "$4" "$5" "$6" "$7" "$8"' bash \
    "${installer}" "$1" "${current_metadata}" "${executable_sha}" "${launcher_sha}" \
    "${tree_sha}" "${test_uid}" "${test_gid}"
}

expect_rejected_release() {
  if verify_release "$1" 2>/dev/null; then
    echo "unsafe Agent release was accepted: $1" >&2
    exit 1
  fi
}

release_path="${release_test_root}/release"
cp -a "${expected_release}" "${release_path}"
verify_release "${release_path}"

cp -a "${expected_release}" "${release_test_root}/metadata-mismatch"
printf 'build=mismatch\n' >"${release_test_root}/metadata-mismatch/AGENT-BUILD-METADATA.txt"
expect_rejected_release "${release_test_root}/metadata-mismatch"

cp -a "${expected_release}" "${release_test_root}/executable-mismatch"
printf 'mutated executable\n' >>"${release_test_root}/executable-mismatch/lib/micro_ros_agent/micro_ros_agent"
expect_rejected_release "${release_test_root}/executable-mismatch"

cp -a "${expected_release}" "${release_test_root}/launcher-mismatch"
printf 'mutated launcher\n' >>"${release_test_root}/launcher-mismatch/bin/mentor-pi-agent"
expect_rejected_release "${release_test_root}/launcher-mismatch"

cp -a "${expected_release}" "${release_test_root}/runtime-mismatch"
printf 'mutated runtime\n' >>"${release_test_root}/runtime-mismatch/share/micro_ros_agent/hook/runtime.dsv"
expect_rejected_release "${release_test_root}/runtime-mismatch"

cp -a "${expected_release}" "${release_test_root}/missing-local-setup"
rm "${release_test_root}/missing-local-setup/local_setup.sh"
expect_rejected_release "${release_test_root}/missing-local-setup"

cp -a "${expected_release}" "${release_test_root}/intermediate-symlink"
rm -rf "${release_test_root}/intermediate-symlink/share"
ln -s "${expected_release}/share" "${release_test_root}/intermediate-symlink/share"
expect_rejected_release "${release_test_root}/intermediate-symlink"

cp -a "${expected_release}" "${release_test_root}/unsafe-mode"
chmod g+w "${release_test_root}/unsafe-mode/lib/libmentor_pi_runtime.so"
expect_rejected_release "${release_test_root}/unsafe-mode"

cp -a "${expected_release}" "${release_test_root}/service-inaccessible-mode"
chmod 0700 "${release_test_root}/service-inaccessible-mode/bin/mentor-pi-agent"
expect_rejected_release "${release_test_root}/service-inaccessible-mode"

cp -a "${expected_release}" "${release_test_root}/newline-symlink-target"
rm "${release_test_root}/newline-symlink-target/lib/libmentor_pi_runtime.so.1"
ln -s $'libmentor_pi_runtime.so\n' \
  "${release_test_root}/newline-symlink-target/lib/libmentor_pi_runtime.so.1"
expect_rejected_release "${release_test_root}/newline-symlink-target"

cp -a "${expected_release}" "${release_test_root}/malformed"
rm "${release_test_root}/malformed/lib/micro_ros_agent/micro_ros_agent"
expect_rejected_release "${release_test_root}/malformed"

if bash -c 'source "$1"; VerifyExistingRelease "$2" "$3" "$4" "$5" "$6" 99999 99999' bash \
    "${installer}" "${release_path}" "${current_metadata}" "${executable_sha}" \
    "${launcher_sha}" "${tree_sha}" 2>/dev/null; then
  echo "release ownership verification cannot be exercised without root" >&2
  exit 1
fi

lock_line="$(grep -n 'flock -n 9' "${installer}" | cut -d: -f1)"
preflight_line="$(grep -n 'SERIAL_ACCESS_HELPER.*--preflight' "${installer}" | cut -d: -f1)"
publish_line="$(grep -n 'mv -T .*temporary_release.*release_path' "${installer}" | cut -d: -f1)"
existing_verify_line="$(grep -n 'VerifyExistingRelease "${release_path}"' "${installer}" | cut -d: -f1)"
useradd_line="$(grep -n 'useradd --system' "${installer}" | cut -d: -f1)"
destination_check_line="$(grep -n 'EnsureTrustedDirectory /etc/systemd/system' \
  "${installer}" | cut -d: -f1)"
current_link_line="$(grep -n 'local current_link=' "${installer}" | cut -d: -f1)"
[[ -n "${lock_line}" && -n "${preflight_line}" && -n "${publish_line}" && \
   -n "${existing_verify_line}" && -n "${useradd_line}" && \
   -n "${destination_check_line}" && -n "${current_link_line}" && \
   "${lock_line}" -lt "${preflight_line}" && "${lock_line}" -lt "${publish_line}" && \
   "${existing_verify_line}" -lt "${useradd_line}" && \
   "${destination_check_line}" -lt "${current_link_line}" ]] || {
  echo "Agent installer ordering does not protect release validation/publication" >&2
  exit 1
}
grep -Fq 'mktemp -d "${temporary_release_prefix}XXXXXX"' "${installer}"
grep -Fq 'EnsureTrustedDirectory /opt/mentor_pi /opt/mentor_pi/agent "${RELEASE_ROOT}"' \
  "${installer}"
grep -Fq 'EnsureTrustedDirectory /run/mentor-pi' "${installer}"
! grep -Fq '/run/lock/mentor-pi-agent-install.lock' "${installer}"
grep -Fq 'VerifySecureEntry "${installer_lock}" 0 0 "Agent installer lock"' \
  "${installer}"
grep -Fq 'VerifyExistingRelease "${temporary_release}"' "${installer}"
grep -Fq 'mv -T "${temporary_release}" "${release_path}"' "${installer}"
grep -Fq 'RemoveLegacyAgentEnvironment' "${installer}"
! grep -Fq '>/etc/mentor-pi/agent.env' "${installer}"
grep -Eq '^[[:space:]]*systemctl enable mentor-pi-agent\.service$' "${installer}"
grep -Eq '^[[:space:]]*systemctl restart mentor-pi-agent\.service$' "${installer}"
! grep -Fq 'systemctl enable --now mentor-pi-agent.service' "${installer}"

executable="${COMPONENT_ROOT}/build/native/install/lib/micro_ros_agent/micro_ros_agent"
launcher="${COMPONENT_ROOT}/build/native/install/bin/mentor-pi-agent"
metadata="${COMPONENT_ROOT}/build/native/install/AGENT-BUILD-METADATA.txt"
if [[ -e "${executable}" || -e "${launcher}" || -e "${metadata}" ]]; then
  [[ -x "${executable}" && -x "${launcher}" && -f "${metadata}" ]] || {
    echo "Agent build output is incomplete" >&2
    exit 1
  }
  expected="$(sed -n 's/^executable_sha256=//p' "${metadata}")"
  [[ "${expected}" == "$(sha256sum "${executable}" | awk '{print $1}')" ]] || {
    echo "Agent executable does not match metadata" >&2
    exit 1
  }
  expected_launcher="$(sed -n 's/^launcher_sha256=//p' "${metadata}")"
  [[ "${expected_launcher}" == "$(sha256sum "${launcher}" | awk '{print $1}')" ]] || {
    echo "Agent launcher does not match metadata" >&2
    exit 1
  }
  set +e
  launcher_help="$(env -i PATH=/usr/bin:/bin HOME=/tmp \
    "${launcher}" --help 2>&1 | LC_ALL=C tr -d '\000')"
  launcher_status=$?
  set -e
  [[ "${launcher_status}" -eq 1 && "${launcher_help}" == Usage:* ]] || {
    echo "Agent launcher does not load its ROS runtime in a clean environment" >&2
    exit 1
  }
fi
echo "micro-ROS Agent component contract passed."
