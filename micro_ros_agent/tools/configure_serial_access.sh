#!/usr/bin/env bash

set -euo pipefail

readonly SERIAL_GROUP="mentor-pi-serial"
readonly UDEV_RULE_NAME="99-mentor-pi-mcu.rules"
readonly DEVICE_ALIAS="/dev/mentor_pi_mcu"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly UDEV_TEMPLATE="${COMPONENT_ROOT}/udev/99-mentor-pi-mcu.rules.in"
readonly DEVICE_FINDER="${COMPONENT_ROOT}/tools/find_device.sh"
readonly UDEVADM="${MENTOR_PI_SERIAL_ACCESS_UDEVADM:-${RRCLITE_SERIAL_ACCESS_UDEVADM:-/usr/bin/udevadm}}"
readonly SYS_CLASS_TTY="${MENTOR_PI_SERIAL_ACCESS_SYS_CLASS_TTY:-${RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY:-/sys/class/tty}}"
readonly TEST_ROOT="${MENTOR_PI_SERIAL_ACCESS_TEST_ROOT:-${RRCLITE_SERIAL_ACCESS_TEST_ROOT:-}}"

selected_device=""
target_user=""
requested_serial=""
requested_path=""
dry_run=0
interactive=0
preflight=0
temporary_directory=""

Fail() { echo "Mentor Pi serial-access setup failed: $*" >&2; exit 1; }

Usage() {
  cat >&2 <<'EOF'
Usage: configure_serial_access.sh [--device /dev/EXPLICIT_DEVICE] --user USER
       [--id-serial-short SERIAL | --id-path PATH]
       [--interactive] [--preflight | --dry-run]
EOF
  exit 2
}

Cleanup() {
  [[ -n "${temporary_directory}" ]] || return
  case "${temporary_directory}" in
    "${TMPDIR:-/tmp}"/mentor-pi-serial-access.*) rm -rf -- "${temporary_directory}" ;;
    *) echo "Refusing unsafe temporary cleanup: ${temporary_directory}" >&2 ;;
  esac
}
trap Cleanup EXIT

PropertyValue() { sed -n "s/^$2=//p" <<<"$1" | head -n 1; }
RuleBody() { sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "$1"; }
Destination() { printf '%s%s' "${TEST_ROOT}" "$1"; }

IdentityMatches() {
  local properties="$1" expected_kind="$2" expected_value="$3" observed=""
  [[ "$(PropertyValue "${properties}" ID_VENDOR_ID)" == 1a86 &&
     "$(PropertyValue "${properties}" ID_MODEL_ID)" == 55d4 ]] || return 1
  if [[ "${expected_kind}" == serial ]]; then
    observed="$(PropertyValue "${properties}" ID_SERIAL_SHORT)"
  else
    observed="$(PropertyValue "${properties}" ID_PATH)"
  fi
  [[ "${observed}" == "${expected_value}" ]]
}

while (($# > 0)); do
  case "$1" in
    --device) (($# >= 2)) || Usage; selected_device="${2:-}"; shift 2 ;;
    --user) (($# >= 2)) || Usage; target_user="${2:-}"; shift 2 ;;
    --id-serial-short) (($# >= 2)) || Usage; requested_serial="${2:-}"; shift 2 ;;
    --id-path) (($# >= 2)) || Usage; requested_path="${2:-}"; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    --interactive) interactive=1; shift ;;
    --preflight) preflight=1; shift ;;
    *) Usage ;;
  esac
done

[[ -z "${selected_device}" || ("${selected_device}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ &&
  "${selected_device}" != *"/../"* && "${selected_device}" != */.. &&
  "${selected_device}" != *"/./"* && "${selected_device}" != */.) ]] ||
  Fail "device must be an explicit, well-formed /dev path"
[[ "${target_user}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || Fail "user must be a valid local login name"
[[ -z "${requested_serial}" || -z "${requested_path}" ]] || Fail "set at most one stable identity selector"
[[ "${dry_run}" != 1 || "${preflight}" != 1 ]] || \
  Fail "set at most one of --preflight or --dry-run"
[[ -z "${requested_serial}" || "${requested_serial}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || Fail "stable identity contains unsupported characters"
[[ -z "${requested_path}" || "${requested_path}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || Fail "stable identity contains unsupported characters"
[[ -x "${UDEVADM}" ]] || Fail "udevadm is required at ${UDEVADM}"
[[ -f "${UDEV_TEMPLATE}" && ! -L "${UDEV_TEMPLATE}" ]] || Fail "udev rule template is missing or symbolic: ${UDEV_TEMPLATE}"
[[ -x "${DEVICE_FINDER}" ]] || Fail "device discovery helper is unavailable"

if [[ -z "${selected_device}" ]]; then
  finder_arguments=(--path)
  [[ -z "${requested_serial}" ]] || finder_arguments+=(--id-serial-short "${requested_serial}")
  [[ -z "${requested_path}" ]] || finder_arguments+=(--id-path "${requested_path}")
  selected_device="$("${DEVICE_FINDER}" "${finder_arguments[@]}")" || Fail "automatic CH9102F discovery did not select one device"
fi

if [[ -n "${TEST_ROOT}" ]]; then
  case "${TEST_ROOT}" in /tmp/mentor-pi-serial-access-test.*|/tmp/rrclite-serial-access-test.*) ;; *) Fail "test root must be an isolated serial-access path below /tmp" ;; esac
else
  [[ "$(id -u)" == 0 ]] || Fail "run this helper as root"
  [[ -c "${selected_device}" ]] || Fail "device is not an existing character device: ${selected_device}"
  [[ -r /etc/os-release ]] || Fail "/etc/os-release is missing"
  host_id="$(sed -n 's/^ID="\{0,1\}\([^"[:space:]]*\)"\{0,1\}$/\1/p' /etc/os-release | head -n 1)"
  [[ "${host_id}" == ubuntu ]] || Fail "serial-access setup requires Ubuntu"
  exec 8>/run/lock/mentor-pi-serial-access.lock
  /usr/bin/flock -n 8 || Fail "another serial-access setup is running"
fi

selected_properties="$(${UDEVADM} info --query=property --name="${selected_device}" 2>/dev/null)" || Fail "udevadm could not inspect ${selected_device}"
[[ "$(PropertyValue "${selected_properties}" ID_VENDOR_ID)" == 1a86 &&
   "$(PropertyValue "${selected_properties}" ID_MODEL_ID)" == 55d4 ]] || Fail "selected device is not the Mentor Pi CH9102F (expected 1a86:55d4)"
if [[ -n "${requested_serial}" ]]; then identity_kind=serial; identity_value="${requested_serial}"
elif [[ -n "${requested_path}" ]]; then identity_kind=id-path; identity_value="${requested_path}"
else
  identity_kind=serial; identity_value="$(PropertyValue "${selected_properties}" ID_SERIAL_SHORT)"
  if [[ -z "${identity_value}" ]]; then identity_kind=id-path; identity_value="$(PropertyValue "${selected_properties}" ID_PATH)"; fi
fi
[[ "${identity_value}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || Fail "device has no safe unique serial or physical-path identity"
IdentityMatches "${selected_properties}" "${identity_kind}" "${identity_value}" || Fail "${selected_device} does not match the requested stable identity"

match_count=0
shopt -s nullglob
for tty_path in "${SYS_CLASS_TTY}"/*; do
  properties=""
  if properties="$(${UDEVADM} info --query=property --name="/dev/$(basename "${tty_path}")" 2>/dev/null)" && IdentityMatches "${properties}" "${identity_kind}" "${identity_value}"; then ((match_count += 1)); fi
done
shopt -u nullglob
((match_count == 1)) || Fail "expected exactly one connected CH9102F with identity ${identity_value}; found ${match_count}"
if [[ "${identity_kind}" == serial ]]; then selector="ATTRS{serial}==\"${identity_value}\""; else selector="ENV{ID_PATH}==\"${identity_value}\""; fi

temporary_directory="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-serial-access.XXXXXX")"
readonly rendered_rule="${temporary_directory}/${UDEV_RULE_NAME}"
sed "s|@MENTOR_PI_DEVICE_IDENTITY@|${selector}|" "${UDEV_TEMPLATE}" >"${rendered_rule}"
grep -Fq '@MENTOR_PI_DEVICE_IDENTITY@' "${rendered_rule}" && Fail "udev rule template rendering failed"
echo "Validated CH9102F ${identity_kind} identity: ${identity_value}"
echo "Selected CH9102F device: ${selected_device}"
if [[ "${dry_run}" == 1 ]]; then echo "Would ensure group ${SERIAL_GROUP} and add user ${target_user}."; echo "Would install $(Destination /etc/udev/rules.d/${UDEV_RULE_NAME}):"; cat "${rendered_rule}"; exit 0; fi

readonly udev_directory="$(Destination /etc/udev/rules.d)" rule_destination="$(Destination /etc/udev/rules.d/${UDEV_RULE_NAME})"
if [[ -e "${rule_destination}" || -L "${rule_destination}" ]]; then
  [[ -f "${rule_destination}" && ! -L "${rule_destination}" ]] || Fail "existing udev rule is not a regular file: ${rule_destination}"
  if ! cmp -s "${rendered_rule}" "${rule_destination}"; then cmp -s <(RuleBody "${rendered_rule}") <(RuleBody "${rule_destination}") || Fail "refusing to overwrite a semantically different existing udev rule: ${rule_destination}"; echo "Keeping semantically identical existing udev rule: ${rule_destination}"; fi
fi
if [[ "${preflight}" == 1 ]]; then
  echo "Serial-access preflight passed without changing the system."
  exit 0
fi
if [[ -z "${TEST_ROOT}" ]]; then
  target_uid="$(id -u "${target_user}" 2>/dev/null)" || Fail "local user does not exist: ${target_user}"
  [[ "${target_uid}" != 0 ]] || Fail "refusing to grant device access to root"
fi
mkdir -p "${udev_directory}"
if [[ ! -e "${rule_destination}" && ! -L "${rule_destination}" ]]; then
  if [[ -n "${TEST_ROOT}" ]]; then install -m 0644 "${rendered_rule}" "${rule_destination}"; else install -o root -g root -m 0644 "${rendered_rule}" "${rule_destination}"; fi
fi
if [[ -n "${TEST_ROOT}" ]]; then
  state_directory="$(Destination /var/lib/mentor-pi-serial-access)"; mkdir -p "${state_directory}"
  printf 'group=%s\nuser=%s\n' "${SERIAL_GROUP}" "${target_user}" >"${state_directory}/membership-request.txt"
  echo "Installed test serial-access rule and membership request."; exit 0
fi
getent group "${SERIAL_GROUP}" >/dev/null || groupadd --system "${SERIAL_GROUP}"
usermod --append --groups "${SERIAL_GROUP}" "${target_user}"
id -nG "${target_user}" | tr ' ' '\n' | grep -Fqx "${SERIAL_GROUP}" || Fail "user was not added to ${SERIAL_GROUP}"
"${UDEVADM}" control --reload-rules
resolved_device="$(readlink -f "${selected_device}")"; [[ "${resolved_device}" == /dev/* ]] || Fail "selected device did not resolve below /dev"
"${UDEVADM}" trigger --action=add --subsystem-match=tty --sysname-match="$(basename "${resolved_device}")"
"${UDEVADM}" settle
[[ -L "${DEVICE_ALIAS}" && "$(readlink -f "${DEVICE_ALIAS}")" == "${resolved_device}" ]] || Fail "udev rule was installed, but ${DEVICE_ALIAS} does not resolve to ${resolved_device}; reconnect the USB cable and rerun"
if [[ "$(stat -c %G "${resolved_device}")" != "${SERIAL_GROUP}" || "$(stat -c %a "${resolved_device}")" != 660 ]]; then
  [[ -c "${resolved_device}" ]] || Fail "resolved serial device disappeared before permission update"
  current_properties="$(${UDEVADM} info --query=property --name="${resolved_device}" 2>/dev/null)" || Fail "udevadm could not revalidate ${resolved_device}"
  IdentityMatches "${current_properties}" "${identity_kind}" "${identity_value}" || Fail "serial identity changed before permission update"
  chgrp -- "${SERIAL_GROUP}" "${resolved_device}"; chmod 0660 "${resolved_device}"
fi
[[ "$(stat -c %G "${resolved_device}")" == "${SERIAL_GROUP}" ]] || Fail "${resolved_device} is not owned by ${SERIAL_GROUP} after udev reload"
[[ "$(stat -c %a "${resolved_device}")" == 660 ]] || Fail "${resolved_device} does not have mode 0660 after udev reload"
echo "Installed stable alias ${DEVICE_ALIAS} for ${resolved_device}."
echo "Added ${target_user} to ${SERIAL_GROUP}; broad dialout access was not granted."
if [[ "${interactive}" == 1 ]]; then echo "Run 'newgrp ${SERIAL_GROUP}' to open a shell with access immediately."; echo "Inside that shell, verify membership with: id -nG"; fi
