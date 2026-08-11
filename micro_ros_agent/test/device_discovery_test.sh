#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FINDER="${TEST_DIR}/../tools/find_device.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT
readonly SYS_CLASS_TTY="${TEST_ROOT}/sys/class/tty"
readonly FAKE_UDEVADM="${TEST_ROOT}/udevadm"

Fail() {
  echo "Mentor Pi device discovery test failed: $*" >&2
  exit 1
}

mkdir -p "${SYS_CLASS_TTY}/ttyUSB7" "${SYS_CLASS_TTY}/ttyS0"
cat >"${FAKE_UDEVADM}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
device=""
for argument in "$@"; do
  case "${argument}" in
    --name=*) device="${argument#--name=}" ;;
  esac
done
case "${device}" in
  /dev/ttyUSB7)
    printf '%s\n' \
      'ID_VENDOR_ID=1a86' \
      'ID_MODEL_ID=55d4' \
      'ID_SERIAL_SHORT=MENTOR-A' \
      'ID_PATH=pci-test-usb-7'
    ;;
  /dev/ttyUSB8)
    printf '%s\n' \
      'ID_VENDOR_ID=1A86' \
      'ID_MODEL_ID=55D4' \
      'ID_SERIAL_SHORT=MENTOR-B' \
      'ID_PATH=pci-test-usb-8'
    ;;
  /dev/ttyS0)
    printf '%s\n' 'ID_VENDOR_ID=ffff' 'ID_MODEL_ID=0001'
    ;;
  *) exit 1 ;;
esac
EOF
chmod 0755 "${FAKE_UDEVADM}"

RunFinder() {
  MENTOR_PI_DEVICE_UDEVADM="${FAKE_UDEVADM}" \
  MENTOR_PI_DEVICE_SYS_CLASS_TTY="${SYS_CLASS_TTY}" \
    "${FINDER}" "$@"
}

[[ "$(RunFinder --path)" == /dev/ttyUSB7 ]] || \
  Fail "single-device path discovery failed"
details="$(RunFinder --details)"
grep -Fqx 'device=/dev/ttyUSB7' <<<"${details}" || \
  Fail "details omit the selected device"
grep -Fqx 'ID_SERIAL_SHORT=MENTOR-A' <<<"${details}" || \
  Fail "details omit the stable serial"

mkdir -p "${SYS_CLASS_TTY}/ttyUSB8"
output=""
if output="$(RunFinder --path 2>&1)"; then
  Fail "ambiguous discovery unexpectedly succeeded"
fi
[[ "${output}" == *'found 2 matching CH9102F ttys'* && \
   "${output}" == *'/dev/ttyUSB7'* && "${output}" == *'/dev/ttyUSB8'* ]] || \
  Fail "ambiguous discovery did not list both candidates"
[[ "$(RunFinder --path --id-serial-short MENTOR-B)" == /dev/ttyUSB8 ]] || \
  Fail "serial selection failed"
[[ "$(RunFinder --path --id-path pci-test-usb-7)" == /dev/ttyUSB7 ]] || \
  Fail "physical-path selection failed"

rm -rf -- "${SYS_CLASS_TTY}/ttyUSB7" "${SYS_CLASS_TTY}/ttyUSB8"
if RunFinder --path >/dev/null 2>&1; then
  Fail "zero-device discovery unexpectedly succeeded"
fi

echo "Mentor Pi CH9102F device discovery tests passed."
