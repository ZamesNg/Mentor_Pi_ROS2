#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${TEST_DIR}/.." && pwd)"
readonly HELPER="${COMPONENT_ROOT}/tools/configure_serial_access.sh"
readonly TEST_ROOT="$(mktemp -d /tmp/mentor-pi-serial-access-test.XXXXXX)"
readonly SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty"
readonly FAKE_UDEVADM="${TEST_ROOT}/udevadm"
readonly TEST_USER="mentor-pi-test"

Cleanup() { rm -rf -- "${TEST_ROOT}"; }
trap Cleanup EXIT
Fail() { echo "Agent serial-access helper test failed: $*" >&2; exit 1; }

grep -Fq -- 'trigger --action=add --subsystem-match=tty' "${HELPER}" || Fail "helper does not replay an add event"
! grep -Fq -- 'trigger --action=change --subsystem-match=tty' "${HELPER}" || Fail "helper still uses a change event"
grep -Fq 'serial identity changed before permission update' "${HELPER}" || Fail "helper does not revalidate before repair"
grep -Fq 'chgrp -- "${SERIAL_GROUP}" "${resolved_device}"' "${HELPER}" || Fail "helper does not repair group"
grep -Fq 'chmod 0660 "${resolved_device}"' "${HELPER}" || Fail "helper does not repair mode"
grep -Fq 'newgrp ${SERIAL_GROUP}' "${HELPER}" || Fail "helper lacks interactive activation guidance"
grep -Fq 'SERIAL_ACCESS_HELPER=' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer does not delegate serial access"
grep -Fq '"${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}"' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer does not invoke serial helper"
preflight_line="$(grep -n -F '"${SERIAL_ACCESS_HELPER}" "${serial_access_arguments[@]}" --preflight' "${COMPONENT_ROOT}/tools/install_service.sh" | cut -d: -f1)"
useradd_line="$(grep -n -F 'useradd --system' "${COMPONENT_ROOT}/tools/install_service.sh" | cut -d: -f1)"
[[ -n "${preflight_line}" && -n "${useradd_line}" && \
   "${preflight_line}" -lt "${useradd_line}" ]] || \
  Fail "installer does not preflight serial access before creating the service user"
! grep -Fq 'DEVICE_FINDER=' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer still duplicates device discovery"
! grep -Fq '99-mentor-pi-mcu.rules.in' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer still renders the udev rule"
! grep -Fq '/etc/udev/rules.d' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer still installs a udev rule"
! grep -Fq 'udevadm control --reload-rules' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer still reloads udev rules"
! grep -Fq 'udevadm trigger' "${COMPONENT_ROOT}/tools/install_service.sh" || Fail "installer still triggers udev itself"
! grep -Fq 'configure_dev_serial_access' "${COMPONENT_ROOT}/../tools/tutorial_action.sh" || Fail "root tutorial action still calls removed helper"

ExpectFailure() { local expected="$1"; shift; local output; if output="$("$@" 2>&1)"; then Fail "command unexpectedly succeeded: $*"; fi; [[ "${output}" == *"${expected}"* ]] || Fail "expected '${expected}', got: ${output}"; }
RunHelper() { env MENTOR_PI_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" MENTOR_PI_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" MENTOR_PI_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" MENTOR_PI_DEVICE_UDEVADM="${FAKE_UDEVADM}" MENTOR_PI_DEVICE_SYS_CLASS_TTY="${SYS_CLASS_TTY}" "${HELPER}" "$@"; }

mkdir -p "${SYS_CLASS_TTY}/ttyACM0"
cat >"${FAKE_UDEVADM}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
node=""
for argument in "$@"; do case "${argument}" in --name=*) node="${argument#--name=}";; esac; done
case "$(basename "${node}")" in
  ttyACM0) vendor=1a86; [[ "${MENTOR_PI_TEST_WRONG_VENDOR:-0}" != 1 ]] || vendor=ffff; printf '%s\n' "ID_VENDOR_ID=${vendor}" 'ID_MODEL_ID=55d4' 'ID_SERIAL_SHORT=MENTOR-DEV-A' 'ID_PATH=pci-test-usb-1' ;;
  ttyACM1) serial=MENTOR-DEV-B; [[ "${MENTOR_PI_TEST_DUPLICATE:-0}" != 1 ]] || serial=MENTOR-DEV-A; printf '%s\n' 'ID_VENDOR_ID=1a86' 'ID_MODEL_ID=55d4' "ID_SERIAL_SHORT=${serial}" 'ID_PATH=pci-test-usb-2' ;;
  *) exit 1 ;;
esac
EOF
chmod +x "${FAKE_UDEVADM}"

auto_output="$(RunHelper --user "${TEST_USER}" --dry-run --interactive)"
[[ "${auto_output}" == *'Selected CH9102F device: /dev/ttyACM0'* && "${auto_output}" == *'ATTRS{serial}=="MENTOR-DEV-A"'* ]] || Fail "automatic identity discovery failed"
preflight_output="$(RunHelper --user "${TEST_USER}" --preflight)"
[[ "${preflight_output}" == *'Serial-access preflight passed without changing the system.'* ]] || \
  Fail "non-mutating preflight did not complete"
[[ ! -e "${TEST_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules" && \
   ! -e "${TEST_ROOT}/var/lib/mentor-pi-serial-access/membership-request.txt" ]] || \
  Fail "serial-access preflight changed the test system"
mkdir -p "${SYS_CLASS_TTY}/ttyACM1"
dry_output="$(RunHelper --device /dev/ttyACM0 --id-serial-short MENTOR-DEV-A --user "${TEST_USER}" --dry-run)"
[[ "${dry_output}" == *'Validated CH9102F serial identity: MENTOR-DEV-A'* ]] || Fail "explicit selector was not validated"
path_output="$(RunHelper --device /dev/ttyACM0 --id-path pci-test-usb-1 --user "${TEST_USER}" --dry-run)"
[[ "${path_output}" == *'Validated CH9102F id-path identity: pci-test-usb-1'* && \
   "${path_output}" == *'ENV{ID_PATH}=="pci-test-usb-1"'* ]] || \
  Fail "explicit physical-path selector was not preserved"
ExpectFailure 'at most one stable identity selector' RunHelper \
  --device /dev/ttyACM0 --id-serial-short MENTOR-DEV-A \
  --id-path pci-test-usb-1 --user "${TEST_USER}" --dry-run
RunHelper --device /dev/ttyACM0 --user "${TEST_USER}"
readonly INSTALLED_RULE="${TEST_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules"
readonly MEMBERSHIP_REQUEST="${TEST_ROOT}/var/lib/mentor-pi-serial-access/membership-request.txt"
grep -Fqx 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="MENTOR-DEV-A", SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"' "${INSTALLED_RULE}" || Fail "installed rule differs from policy"
! grep -Fq 'GROUP="dialout"' "${INSTALLED_RULE}" || Fail "installed rule grants broad dialout access"
grep -Fqx 'group=mentor-pi-serial' "${MEMBERSHIP_REQUEST}" && grep -Fqx "user=${TEST_USER}" "${MEMBERSHIP_REQUEST}" || Fail "membership request is wrong"
RunHelper --device /dev/ttyACM0 --user "${TEST_USER}"
{
  printf '%s\n' '# Reviewed legacy comment.'
  sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "${INSTALLED_RULE}"
} >"${INSTALLED_RULE}.legacy"
mv "${INSTALLED_RULE}.legacy" "${INSTALLED_RULE}"
legacy_output="$(RunHelper --device /dev/ttyACM0 --user "${TEST_USER}")"
[[ "${legacy_output}" == *'Keeping semantically identical existing udev rule'* ]] || Fail "comment-only legacy rule was not retained"
ExpectFailure 'valid local login name' RunHelper --device /dev/ttyACM0 --user bad/user --dry-run
ExpectFailure 'well-formed /dev path' RunHelper --device /tmp/ttyACM0 --user "${TEST_USER}" --dry-run
printf '%s\n' 'SUBSYSTEM=="tty", ENV{UNREVIEWED}="1"' >>"${INSTALLED_RULE}"
ExpectFailure 'refusing to overwrite a semantically different existing udev rule' RunHelper --device /dev/ttyACM0 --user "${TEST_USER}" --preflight
ExpectFailure 'does not match the requested stable identity' RunHelper --device /dev/ttyACM0 --id-serial-short WRONG --user "${TEST_USER}" --dry-run
ExpectFailure 'not the Mentor Pi CH9102F' env MENTOR_PI_TEST_WRONG_VENDOR=1 MENTOR_PI_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" MENTOR_PI_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" MENTOR_PI_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" "${HELPER}" --device /dev/ttyACM0 --user "${TEST_USER}"
ExpectFailure 'found 2' env MENTOR_PI_TEST_DUPLICATE=1 MENTOR_PI_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" MENTOR_PI_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" MENTOR_PI_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" "${HELPER}" --device /dev/ttyACM0 --user "${TEST_USER}"
echo "Agent serial-access helper tests passed."
