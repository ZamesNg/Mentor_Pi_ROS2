#!/usr/bin/env bash

set -euo pipefail

readonly UDEVADM="${MENTOR_PI_DEVICE_UDEVADM:-${RRCLITE_SERIAL_ACCESS_UDEVADM:-/usr/bin/udevadm}}"
readonly SYS_CLASS_TTY="${MENTOR_PI_DEVICE_SYS_CLASS_TTY:-${RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY:-/sys/class/tty}}"
readonly EXPECTED_VENDOR=1a86
readonly EXPECTED_PRODUCT=55d4

output_format=details
serial_selector=""
path_selector=""

Fail() {
  echo "Mentor Pi device discovery failed: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: find_device.sh [--path|--details]
                      [--id-serial-short VALUE | --id-path VALUE]

Finds the tty belonging to the Mentor Pi CH9102F USB adapter (1a86:55d4).
Discovery succeeds only when the optional stable selector identifies exactly
one connected tty.
EOF
  exit 2
}

PropertyValue() {
  local properties="$1"
  local name="$2"
  sed -n "s/^${name}=//p" <<<"${properties}" | head -n 1
}

while (($# > 0)); do
  case "$1" in
    --path)
      output_format=path
      shift
      ;;
    --details)
      output_format=details
      shift
      ;;
    --id-serial-short)
      (($# >= 2)) || Usage
      serial_selector="${2:-}"
      shift 2
      ;;
    --id-path)
      (($# >= 2)) || Usage
      path_selector="${2:-}"
      shift 2
      ;;
    *) Usage ;;
  esac
done

[[ -z "${serial_selector}" || -z "${path_selector}" ]] || \
  Fail "set at most one of ID_SERIAL_SHORT or ID_PATH"
for selector in "${serial_selector}" "${path_selector}"; do
  [[ -z "${selector}" || "${selector}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || \
    Fail "stable identity contains unsupported characters"
done
[[ -x "${UDEVADM}" ]] || Fail "udevadm is required at ${UDEVADM}"
[[ -d "${SYS_CLASS_TTY}" ]] || \
  Fail "Linux tty discovery path is unavailable: ${SYS_CLASS_TTY}"

declare -a matched_devices=()
declare -a matched_serials=()
declare -a matched_paths=()
shopt -s nullglob
for tty_path in "${SYS_CLASS_TTY}"/*; do
  tty_name="$(basename "${tty_path}")"
  [[ "${tty_name}" =~ ^[A-Za-z0-9._:+-]+$ ]] || continue
  device="/dev/${tty_name}"
  properties=""
  properties="$(${UDEVADM} info --query=property \
    --name="${device}" 2>/dev/null)" || continue
  vendor="$(PropertyValue "${properties}" ID_VENDOR_ID)"
  product="$(PropertyValue "${properties}" ID_MODEL_ID)"
  [[ "${vendor,,}" == "${EXPECTED_VENDOR}" && \
     "${product,,}" == "${EXPECTED_PRODUCT}" ]] || continue

  serial="$(PropertyValue "${properties}" ID_SERIAL_SHORT)"
  physical_path="$(PropertyValue "${properties}" ID_PATH)"
  [[ -z "${serial_selector}" || "${serial}" == "${serial_selector}" ]] || \
    continue
  [[ -z "${path_selector}" || "${physical_path}" == "${path_selector}" ]] || \
    continue
  matched_devices+=("${device}")
  matched_serials+=("${serial}")
  matched_paths+=("${physical_path}")
done
shopt -u nullglob

if ((${#matched_devices[@]} == 0)); then
  Fail "no connected CH9102F tty matches USB ${EXPECTED_VENDOR}:${EXPECTED_PRODUCT}; check the data cable, board power, and optional stable selector"
fi
if ((${#matched_devices[@]} != 1)); then
  {
    printf 'Mentor Pi device discovery failed: found %d matching CH9102F ttys:\n' \
      "${#matched_devices[@]}"
    for index in "${!matched_devices[@]}"; do
      printf '  %s  ID_SERIAL_SHORT=%s  ID_PATH=%s\n' \
        "${matched_devices[index]}" \
        "${matched_serials[index]:-unavailable}" \
        "${matched_paths[index]:-unavailable}"
    done
    echo "Select the intended board with ID_SERIAL_SHORT or ID_PATH; do not guess a tty number."
  } >&2
  exit 1
fi

if [[ "${output_format}" == path ]]; then
  printf '%s\n' "${matched_devices[0]}"
else
  printf '%s\n' \
    "device=${matched_devices[0]}" \
    "ID_VENDOR_ID=${EXPECTED_VENDOR}" \
    "ID_MODEL_ID=${EXPECTED_PRODUCT}" \
    "ID_SERIAL_SHORT=${matched_serials[0]:-unavailable}" \
    "ID_PATH=${matched_paths[0]:-unavailable}"
fi
