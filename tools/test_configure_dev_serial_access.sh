#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly HELPER="${SCRIPT_DIR}/configure_dev_serial_access.sh"
readonly TEST_ROOT="$(mktemp -d /tmp/rrclite-serial-access-test.XXXXXX)"
readonly SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty"
readonly FAKE_UDEVADM="${TEST_ROOT}/fake_udevadm"
readonly TEST_USER="rrclite-test"

Cleanup() {
  case "${TEST_ROOT}" in
    /tmp/rrclite-serial-access-test.*)
      rm -rf -- "${TEST_ROOT}"
      ;;
    *)
      echo "Refusing unsafe serial-access test cleanup: ${TEST_ROOT}" >&2
      ;;
  esac
}
trap Cleanup EXIT

Fail() {
  echo "Development serial-access helper test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected_text="$1"
  shift
  local output=""
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected_text}"* ]] || \
    Fail "failure did not contain '${expected_text}': ${output}"
}

RunHelper() {
  env \
    RRCLITE_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" \
    RRCLITE_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" \
    RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" \
    "${HELPER}" "$@"
}

mkdir -p "${SYS_CLASS_TTY}/ttyACM0" "${SYS_CLASS_TTY}/ttyACM1"
cat >"${FAKE_UDEVADM}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
node=""
for argument in "$@"; do
  case "${argument}" in
    --name=*) node="${argument#--name=}" ;;
  esac
done
case "$(basename "${node}")" in
  ttyACM0)
    vendor="1a86"
    [[ "${RRCLITE_TEST_WRONG_VENDOR:-0}" != "1" ]] || vendor="ffff"
    printf '%s\n' \
      "ID_VENDOR_ID=${vendor}" \
      'ID_MODEL_ID=55d4' \
      'ID_SERIAL_SHORT=RRCLITE-DEV-A' \
      'ID_PATH=pci-test-usb-1'
    ;;
  ttyACM1)
    serial="RRCLITE-DEV-B"
    [[ "${RRCLITE_TEST_DUPLICATE:-0}" != "1" ]] || serial="RRCLITE-DEV-A"
    printf '%s\n' \
      'ID_VENDOR_ID=1a86' \
      'ID_MODEL_ID=55d4' \
      "ID_SERIAL_SHORT=${serial}" \
      'ID_PATH=pci-test-usb-2'
    ;;
  *) exit 1 ;;
esac
EOF
chmod +x "${FAKE_UDEVADM}"

readonly DRY_OUTPUT="$(RunHelper \
  --device /dev/ttyACM0 --user "${TEST_USER}" --dry-run)"
[[ "${DRY_OUTPUT}" == *'Validated CH9102F serial identity: RRCLITE-DEV-A'* ]] || \
  Fail "dry run did not report the verified identity"
[[ "${DRY_OUTPUT}" == *'SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial"'* ]] || \
  Fail "dry run did not render the dedicated alias and group"
[[ ! -e "${TEST_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules" ]] || \
  Fail "dry run installed a udev rule"

RunHelper --device /dev/ttyACM0 --user "${TEST_USER}"
readonly INSTALLED_RULE="${TEST_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules"
readonly MEMBERSHIP_REQUEST="${TEST_ROOT}/var/lib/rrclite-dev-serial-access/membership-request.txt"
grep -Fqx \
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="RRCLITE-DEV-A", SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"' \
  "${INSTALLED_RULE}" || Fail "installed udev rule differs from policy"
! grep -Fq 'GROUP="dialout"' "${INSTALLED_RULE}" || \
  Fail "installed rule grants broad dialout access"
grep -Fqx 'group=mentor-pi-serial' "${MEMBERSHIP_REQUEST}" || \
  Fail "test membership request has the wrong group"
grep -Fqx "user=${TEST_USER}" "${MEMBERSHIP_REQUEST}" || \
  Fail "test membership request has the wrong user"

# An identical rerun is intentionally idempotent.
RunHelper --device /dev/ttyACM0 --user "${TEST_USER}"

ExpectFailure "valid local login name" RunHelper \
  --device /dev/ttyACM0 --user 'bad/user'
ExpectFailure "well-formed /dev path" RunHelper \
  --device /tmp/ttyACM0 --user "${TEST_USER}"
ExpectFailure "not the Mentor Pi CH9102F" env \
  RRCLITE_TEST_WRONG_VENDOR=1 \
  RRCLITE_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" \
  RRCLITE_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" \
  RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" \
  "${HELPER}" --device /dev/ttyACM0 --user "${TEST_USER}"
ExpectFailure "found 2" env \
  RRCLITE_TEST_DUPLICATE=1 \
  RRCLITE_SERIAL_ACCESS_TEST_ROOT="${TEST_ROOT}" \
  RRCLITE_SERIAL_ACCESS_UDEVADM="${FAKE_UDEVADM}" \
  RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY="${SYS_CLASS_TTY}" \
  "${HELPER}" --device /dev/ttyACM0 --user "${TEST_USER}"

printf '%s\n' '# unexpected local replacement' >>"${INSTALLED_RULE}"
ExpectFailure "refusing to overwrite a different existing udev rule" RunHelper \
  --device /dev/ttyACM0 --user "${TEST_USER}"

echo "Development serial-access helper tests passed."
