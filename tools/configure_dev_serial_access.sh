#!/usr/bin/env bash

set -euo pipefail

readonly SERIAL_GROUP="mentor-pi-serial"
readonly UDEV_RULE_NAME="99-mentor-pi-mcu.rules"
readonly DEVICE_ALIAS="/dev/mentor_pi_mcu"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly UDEV_TEMPLATE="${PROJECT_ROOT}/micro_ros_agent/udev/99-mentor-pi-mcu.rules.in"
readonly DEVICE_FINDER="${PROJECT_ROOT}/micro_ros_agent/tools/find_device.sh"
readonly UDEVADM="${RRCLITE_SERIAL_ACCESS_UDEVADM:-/usr/bin/udevadm}"
readonly SYS_CLASS_TTY="${RRCLITE_SERIAL_ACCESS_SYS_CLASS_TTY:-/sys/class/tty}"
readonly TEST_ROOT="${RRCLITE_SERIAL_ACCESS_TEST_ROOT:-}"

selected_device=""
target_user=""
dry_run=0
temporary_directory=""

Fail() {
  echo "RRCLite development serial-access setup failed: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: configure_dev_serial_access.sh [--device /dev/EXPLICIT_DEVICE] \
  --user LOGIN_NAME [--dry-run]

This Ubuntu development-host helper verifies exactly one CH9102F,
installs /dev/mentor_pi_mcu, and grants LOGIN_NAME access through the dedicated
mentor-pi-serial group. Without --device it discovers the adapter by USB
vendor/product identity. It never grants broad dialout membership.
EOF
  exit 2
}

Cleanup() {
  [[ -n "${temporary_directory}" ]] || return
  case "${temporary_directory}" in
    "${TMPDIR:-/tmp}"/rrclite-serial-access.*)
      rm -rf -- "${temporary_directory}"
      ;;
    *)
      echo "Refusing unsafe temporary cleanup: ${temporary_directory}" >&2
      ;;
  esac
}
trap Cleanup EXIT

PropertyValue() {
  local properties="$1"
  local name="$2"
  sed -n "s/^${name}=//p" <<<"${properties}" | head -n 1
}

IdentityMatches() {
  local properties="$1"
  local expected_kind="$2"
  local expected_value="$3"
  local observed=""

  [[ "$(PropertyValue "${properties}" ID_VENDOR_ID)" == "1a86" &&
    "$(PropertyValue "${properties}" ID_MODEL_ID)" == "55d4" ]] || \
    return 1
  if [[ "${expected_kind}" == "serial" ]]; then
    observed="$(PropertyValue "${properties}" ID_SERIAL_SHORT)"
  else
    observed="$(PropertyValue "${properties}" ID_PATH)"
  fi
  [[ "${observed}" == "${expected_value}" ]]
}

RuleBody() {
  sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "$1"
}

Destination() {
  printf '%s%s' "${TEST_ROOT}" "$1"
}

while (($# > 0)); do
  case "$1" in
    --device)
      (($# >= 2)) || Usage
      selected_device="${2:-}"
      shift 2
      ;;
    --user)
      (($# >= 2)) || Usage
      target_user="${2:-}"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *) Usage ;;
  esac
done

[[ -z "${selected_device}" || \
  ("${selected_device}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ &&
   "${selected_device}" != *"/../"* && "${selected_device}" != */.. &&
   "${selected_device}" != *"/./"* && "${selected_device}" != */.) ]] || \
  Fail "device must be an explicit, well-formed /dev path"
[[ "${target_user}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || \
  Fail "user must be a valid local login name"
[[ -x "${UDEVADM}" ]] || Fail "udevadm is required at ${UDEVADM}"
[[ -f "${UDEV_TEMPLATE}" && ! -L "${UDEV_TEMPLATE}" ]] || \
  Fail "udev rule template is missing or symbolic: ${UDEV_TEMPLATE}"
[[ -x "${DEVICE_FINDER}" ]] || Fail "device discovery helper is unavailable"
if [[ -z "${selected_device}" ]]; then
  selected_device="$("${DEVICE_FINDER}" --path)" || \
    Fail "automatic CH9102F discovery did not select one device"
fi

if [[ -n "${TEST_ROOT}" ]]; then
  case "${TEST_ROOT}" in
    /tmp/rrclite-serial-access-test.*) ;;
    *) Fail "test root must be an isolated rrclite path below /tmp" ;;
  esac
else
  [[ "$(id -u)" == "0" ]] || \
    Fail "run this helper as root, for example through sudo make serial-access"
  [[ -c "${selected_device}" ]] || \
    Fail "device is not an existing character device: ${selected_device}"
  [[ -r /etc/os-release ]] || Fail "/etc/os-release is missing"
  host_id="$(sed -n 's/^ID="\{0,1\}\([^"[:space:]]*\)"\{0,1\}$/\1/p' \
    /etc/os-release | head -n 1)"
  [[ "${host_id}" == "ubuntu" ]] || \
    Fail "this interactive-user helper requires Ubuntu"
  target_uid="$(id -u "${target_user}" 2>/dev/null)" || \
    Fail "local user does not exist: ${target_user}"
  [[ "${target_uid}" != "0" ]] || Fail "refusing to grant device access to root"

  exec 8>/run/lock/rrclite-dev-serial-access.lock
  /usr/bin/flock -n 8 || Fail "another serial-access setup is running"
fi

selected_properties="$(${UDEVADM} info --query=property \
  --name="${selected_device}" 2>/dev/null)" || \
  Fail "udevadm could not inspect ${selected_device}"
[[ "$(PropertyValue "${selected_properties}" ID_VENDOR_ID)" == "1a86" &&
  "$(PropertyValue "${selected_properties}" ID_MODEL_ID)" == "55d4" ]] || \
  Fail "selected device is not the Mentor Pi CH9102F (expected 1a86:55d4)"

identity_kind="serial"
identity_value="$(PropertyValue "${selected_properties}" ID_SERIAL_SHORT)"
if [[ -z "${identity_value}" ]]; then
  identity_kind="id-path"
  identity_value="$(PropertyValue "${selected_properties}" ID_PATH)"
fi
[[ "${identity_value}" =~ ^[A-Za-z0-9._:+/@-]+$ ]] || \
  Fail "device has no safe unique serial or physical-path identity"
readonly identity_kind identity_value

match_count=0
shopt -s nullglob
for tty_path in "${SYS_CLASS_TTY}"/*; do
  properties=""
  if properties="$(${UDEVADM} info --query=property \
      --name="/dev/$(basename "${tty_path}")" 2>/dev/null)" &&
      IdentityMatches "${properties}" "${identity_kind}" "${identity_value}"; then
    ((match_count += 1))
  fi
done
shopt -u nullglob
((match_count == 1)) || \
  Fail "expected exactly one connected CH9102F with identity ${identity_value}; found ${match_count}"

if [[ "${identity_kind}" == "serial" ]]; then
  selector="ATTRS{serial}==\"${identity_value}\""
else
  selector="ENV{ID_PATH}==\"${identity_value}\""
fi
readonly selector

temporary_directory="$(mktemp -d \
  "${TMPDIR:-/tmp}/rrclite-serial-access.XXXXXX")"
readonly rendered_rule="${temporary_directory}/${UDEV_RULE_NAME}"
sed "s|@MENTOR_PI_DEVICE_IDENTITY@|${selector}|" \
  "${UDEV_TEMPLATE}" >"${rendered_rule}"
grep -Fq '@MENTOR_PI_DEVICE_IDENTITY@' "${rendered_rule}" && \
  Fail "udev rule template rendering failed"

echo "Validated CH9102F ${identity_kind} identity: ${identity_value}"
echo "Selected CH9102F device: ${selected_device}"
if [[ "${dry_run}" == "1" ]]; then
  echo "Would ensure group ${SERIAL_GROUP} and add user ${target_user}."
  echo "Would install $(Destination /etc/udev/rules.d/${UDEV_RULE_NAME}):"
  cat "${rendered_rule}"
  exit 0
fi

readonly udev_directory="$(Destination /etc/udev/rules.d)"
readonly rule_destination="${udev_directory}/${UDEV_RULE_NAME}"
mkdir -p "${udev_directory}"
if [[ -e "${rule_destination}" || -L "${rule_destination}" ]]; then
  [[ -f "${rule_destination}" && ! -L "${rule_destination}" ]] || \
    Fail "existing udev rule is not a regular file: ${rule_destination}"
  if ! cmp -s "${rendered_rule}" "${rule_destination}"; then
    cmp -s <(RuleBody "${rendered_rule}") \
      <(RuleBody "${rule_destination}") || \
      Fail "refusing to overwrite a semantically different existing udev rule: ${rule_destination}"
    echo "Keeping semantically identical existing udev rule: ${rule_destination}"
  fi
else
  if [[ -n "${TEST_ROOT}" ]]; then
    install -m 0644 "${rendered_rule}" "${rule_destination}"
  else
    install -o root -g root -m 0644 "${rendered_rule}" "${rule_destination}"
  fi
fi

if [[ -n "${TEST_ROOT}" ]]; then
  readonly state_directory="$(Destination /var/lib/rrclite-dev-serial-access)"
  mkdir -p "${state_directory}"
  printf 'group=%s\nuser=%s\n' "${SERIAL_GROUP}" "${target_user}" \
    >"${state_directory}/membership-request.txt"
  echo "Installed test serial-access rule and membership request."
  exit 0
fi

if ! getent group "${SERIAL_GROUP}" >/dev/null; then
  groupadd --system "${SERIAL_GROUP}"
fi
usermod --append --groups "${SERIAL_GROUP}" "${target_user}"
id -nG "${target_user}" | tr ' ' '\n' | grep -Fqx "${SERIAL_GROUP}" || \
  Fail "user was not added to ${SERIAL_GROUP}"

"${UDEVADM}" control --reload-rules
resolved_device="$(readlink -f "${selected_device}")"
[[ "${resolved_device}" == /dev/* ]] || \
  Fail "selected device did not resolve below /dev"
# A change event refreshes symlinks but does not reliably reapply device-node
# ownership and mode. Replay an add event for only the already validated tty so
# the newly installed GROUP and MODE policy takes effect without reconnecting.
"${UDEVADM}" trigger --action=add --subsystem-match=tty \
  --sysname-match="$(basename "${resolved_device}")"
"${UDEVADM}" settle

[[ -L "${DEVICE_ALIAS}" && "$(readlink -f "${DEVICE_ALIAS}")" == \
  "${resolved_device}" ]] || \
  Fail "udev rule was installed, but ${DEVICE_ALIAS} does not resolve to ${resolved_device}; reconnect the USB cable and rerun"

# Some RDK udev builds replay the alias on a synthetic add event without
# updating the already-existing devtmpfs node's group and mode. Revalidate the
# exact physical identity immediately before applying the same policy to the
# current node; the installed rule handles all future real add events.
if [[ "$(stat -c %G "${resolved_device}")" != "${SERIAL_GROUP}" ||
      "$(stat -c %a "${resolved_device}")" != "660" ]]; then
  [[ -c "${resolved_device}" ]] || \
    Fail "resolved serial device disappeared before permission update"
  current_properties="$(${UDEVADM} info --query=property \
    --name="${resolved_device}" 2>/dev/null)" || \
    Fail "udevadm could not revalidate ${resolved_device}"
  IdentityMatches "${current_properties}" "${identity_kind}" \
    "${identity_value}" || \
    Fail "serial identity changed before permission update"
  chgrp -- "${SERIAL_GROUP}" "${resolved_device}"
  chmod 0660 "${resolved_device}"
fi
[[ "$(stat -c %G "${resolved_device}")" == "${SERIAL_GROUP}" ]] || \
  Fail "${resolved_device} is not owned by ${SERIAL_GROUP} after udev reload"
[[ "$(stat -c %a "${resolved_device}")" == "660" ]] || \
  Fail "${resolved_device} does not have mode 0660 after udev reload"

echo "Installed stable alias ${DEVICE_ALIAS} for ${resolved_device}."
echo "Added ${target_user} to ${SERIAL_GROUP}; broad dialout access was not granted."
echo "Run 'newgrp ${SERIAL_GROUP}' to open a shell with access immediately."
echo "Inside that shell, verify membership with: id -nG"
