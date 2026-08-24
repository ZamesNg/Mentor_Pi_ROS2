#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FLASH_SOURCE="${SCRIPT_DIR}/flash.sh"
readonly SHA_SOURCE="${SCRIPT_DIR}/sha256.sh"
readonly SHA_MANIFEST_SOURCE="${SCRIPT_DIR}/sha256_manifest.sh"
readonly PACKAGE_VERIFIER_SOURCE="${SCRIPT_DIR}/verify_package.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "Firmware automatic-flash test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output=""
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "failure did not contain '${expected}': ${output}"
}

WriteArmElfFixture() {
  local output="$1"
  printf '\177ELF\001\001\001\000\000\000\000\000\000\000\000\000\002\000\050\000\001\000\000\000\001\001\000\010' \
    >"${output}"
}

CreateFixture() {
  local name="$1"
  local root="${TEST_ROOT}/${name}"
  local tools="${root}/firmware/tools"
  local build="${root}/firmware/mentor_pi_mcu/build/stm32"
  mkdir -p "${tools}" "${build}" "${root}/bin" "${root}/tmp"
  cp "${FLASH_SOURCE}" "${tools}/flash.sh"
  if [[ -f /.dockerenv ]]; then
    # The unmodified entry point is tested separately for fail-closed container
    # rejection. Remove only that already-proven guard from this disposable
    # fixture copy so the remaining physical-host orchestration can be driven
    # through fake character device/programmer/boot-control boundaries.
    awk '
      index($0, "[[ ! -f /.dockerenv ]]") == 1 {
        print "true"
        getline
        next
      }
      {print}
    ' "${tools}/flash.sh" >"${tools}/flash.sh.container-test"
    mv "${tools}/flash.sh.container-test" "${tools}/flash.sh"
  fi
  cp "${SHA_SOURCE}" "${tools}/sha256.sh"
  cp "${SHA_MANIFEST_SOURCE}" "${tools}/sha256_manifest.sh"
  cp "${PACKAGE_VERIFIER_SOURCE}" "${tools}/verify_package.sh"
  chmod 0755 "${tools}/flash.sh" "${tools}/sha256.sh" \
    "${tools}/sha256_manifest.sh" "${tools}/verify_package.sh"
  WriteArmElfFixture "${build}/mentor_pi_mcu.elf"
  digest="$("${tools}/sha256.sh" "${build}/mentor_pi_mcu.elf")"
  printf 'elf_sha256=%s\n' "${digest}" \
    >"${build}/rrclite-build-metadata.txt"

  cat >"${root}/bin/fuser" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
  cat >"${root}/bin/lsof" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
  cat >"${root}/bin/uname" <<'EOF'
#!/usr/bin/env bash
[[ "$#" == 1 && "$1" == -s ]]
printf '%s\n' Linux
EOF
  cat >"${root}/STM32_Programmer_CLI" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
: "${FAKE_PROGRAMMER_LOG:?}"
mode=preflight
image=""
expect_image=0
for argument in "$@"; do
  if ((expect_image == 1)); then
    image="${argument}"
    expect_image=0
  elif [[ "${argument}" == -w ]]; then
    mode=flash
    expect_image=1
  fi
done
printf '%s\n' "${mode}" >>"${FAKE_PROGRAMMER_LOG}"
if [[ "${mode}" == preflight ]]; then
  exit "${FAKE_PREFLIGHT_STATUS:-0}"
fi
[[ -r "${image}" && ! -w "${image}" ]] || exit 29
exit "${FAKE_FLASH_STATUS:-0}"
EOF
  cat >"${root}/ch9102_boot_control" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
: "${FAKE_BOOT_LOG:?}"
[[ "$#" == 4 && "$1" == --device && "$3" == --mode ]]
printf '%s\n' "$4" >>"${FAKE_BOOT_LOG}"
[[ "${FAKE_BOOT_FAILURE_MODE:-}" != "$4" ]]
EOF
  chmod 0755 "${root}/bin/fuser" "${root}/bin/lsof" "${root}/bin/uname" \
    "${root}/STM32_Programmer_CLI" "${root}/ch9102_boot_control"
  printf '%s' "${root}"
}

WritePackageChecksums() {
  local root="$1" package="$2"
  "${root}/firmware/tools/sha256_manifest.sh" "${package}" \
    BUILD-METADATA.txt \
    BUILD-MODE.txt \
    mentor_pi_mcu-firmware-adrc-release.bin \
    mentor_pi_mcu-firmware-adrc-release.elf \
    mentor_pi_mcu-firmware-adrc-release.hex \
    mentor_pi_mcu-firmware-adrc-release.map \
    >"${package}/SHA256SUMS"
}

CreatePackage() {
  local root="$1" name="$2"
  local package="${root}/${name}/firmware-adrc-release"
  mkdir -p "${package}"
  WriteArmElfFixture \
    "${package}/mentor_pi_mcu-firmware-adrc-release.elf"
  printf '%s\n' 'transferred package HEX' \
    >"${package}/mentor_pi_mcu-firmware-adrc-release.hex"
  printf '%s\n' 'transferred package BIN' \
    >"${package}/mentor_pi_mcu-firmware-adrc-release.bin"
  printf '%s\n' \
    '0x08000100 __flash_image_end__ =' \
    '0x20000100 __ram_used_end__ =' \
    '0x10000100 __ccm_end__ =' \
    >"${package}/mentor_pi_mcu-firmware-adrc-release.map"
  local elf_sha hex_sha bin_sha map_sha
  elf_sha="$("${root}/firmware/tools/sha256.sh" \
    "${package}/mentor_pi_mcu-firmware-adrc-release.elf")"
  hex_sha="$("${root}/firmware/tools/sha256.sh" \
    "${package}/mentor_pi_mcu-firmware-adrc-release.hex")"
  bin_sha="$("${root}/firmware/tools/sha256.sh" \
    "${package}/mentor_pi_mcu-firmware-adrc-release.bin")"
  map_sha="$("${root}/firmware/tools/sha256.sh" \
    "${package}/mentor_pi_mcu-firmware-adrc-release.map")"
  printf '%s\n' \
    'schema=mentor-pi-firmware-build-v3' \
    'target=STM32F407VET6' \
    'ros_distro=humble' \
    'builder_mode=native-pinned' \
    'build_environment=devcontainer' \
    'host_os=ubuntu-22.04' \
    'host_architecture=amd64' \
    'toolchain=arm-gnu-toolchain-13.2.rel1' \
    'motor_mode=ADRC' \
    'control_mode=CLOSED_LOOP' \
    'artifact_mode=NORMAL' \
    'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
    'release_qualified=0' \
    'ros_namespace=/mecanum_1' \
    'source_sha256=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' \
    'interfaces_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb' \
    'microros_sdk_archive_sha256=cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc' \
    'microros_sdk_tree_sha256=dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd' \
    "elf_sha256=${elf_sha}" \
    "hex_sha256=${hex_sha}" \
    "bin_sha256=${bin_sha}" \
    "map_sha256=${map_sha}" \
    >"${package}/BUILD-METADATA.txt"
  printf '%s\n' \
    'target=STM32F407VET6' \
    'motor_mode=ADRC' \
    'control_mode=CLOSED_LOOP' \
    'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
    >"${package}/BUILD-MODE.txt"
  WritePackageChecksums "${root}" "${package}"
  printf '%s' "${package}"
}

RunFlash() {
  local root="$1"
  shift
  env \
    PATH="${root}/bin:/usr/bin:/bin" \
    FAKE_PROGRAMMER_LOG="${root}/programmer.log" \
    FAKE_BOOT_LOG="${root}/boot.log" \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    RRCLITE_CH9102_BOOT_CONTROL="${root}/ch9102_boot_control" \
    STM32_CUBE_PROGRAMMER_CLI="${root}/STM32_Programmer_CLI" \
    TMPDIR="${root}/tmp" \
    "$@" "${root}/firmware/tools/flash.sh" /dev/null
}

RunPackageFlash() {
  local root="$1" package="$2"
  shift 2
  local manifest_sha256
  manifest_sha256="$("${root}/firmware/tools/sha256.sh" \
    "${package}/SHA256SUMS")"
  env \
    PATH="${root}/bin:/usr/bin:/bin" \
    FAKE_PROGRAMMER_LOG="${root}/programmer.log" \
    FAKE_BOOT_LOG="${root}/boot.log" \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    RRCLITE_CH9102_BOOT_CONTROL="${root}/ch9102_boot_control" \
    STM32_CUBE_PROGRAMMER_CLI="${root}/STM32_Programmer_CLI" \
    TMPDIR="${root}/tmp" \
    "$@" "${root}/firmware/tools/flash.sh" \
    --package "${package}" \
    --expected-namespace /mecanum_1 \
    --expected-manifest-sha256 "${manifest_sha256}" \
    /dev/null
}

package_root="$(CreateFixture package-validation)"
valid_package="$(CreatePackage "${package_root}" 'valid package')"
"${package_root}/firmware/tools/verify_package.sh" \
  "${valid_package}" >/dev/null
valid_manifest_sha256="$("${package_root}/firmware/tools/sha256.sh" \
  "${valid_package}/SHA256SUMS")"
"${package_root}/firmware/tools/verify_package.sh" \
  --expected-namespace /mecanum_1 \
  --expected-manifest-sha256 "${valid_manifest_sha256}" \
  "${valid_package}" >/dev/null
ExpectFailure 'trusted out-of-band digest' \
  "${package_root}/firmware/tools/verify_package.sh" \
  --expected-namespace /mecanum_1 \
  --expected-manifest-sha256 \
  0000000000000000000000000000000000000000000000000000000000000000 \
  "${valid_package}"
ExpectFailure 'does not match the expected robot namespace' \
  "${package_root}/firmware/tools/verify_package.sh" \
  --expected-namespace /mecanum_0 \
  --expected-manifest-sha256 "${valid_manifest_sha256}" \
  "${valid_package}"

fake_elf_root="$(CreateFixture package-fake-elf)"
fake_elf_package="$(CreatePackage "${fake_elf_root}" fake-elf)"
printf '%s\n' 'ELF' \
  >"${fake_elf_package}/mentor_pi_mcu-firmware-adrc-release.elf"
fake_elf_sha="$("${fake_elf_root}/firmware/tools/sha256.sh" \
  "${fake_elf_package}/mentor_pi_mcu-firmware-adrc-release.elf")"
awk -v digest="${fake_elf_sha}" \
  '{if ($0 ~ /^elf_sha256=/) {print "elf_sha256=" digest} else {print}}' \
  "${fake_elf_package}/BUILD-METADATA.txt" \
  >"${fake_elf_package}/BUILD-METADATA.txt.tmp"
mv "${fake_elf_package}/BUILD-METADATA.txt.tmp" \
  "${fake_elf_package}/BUILD-METADATA.txt"
WritePackageChecksums "${fake_elf_root}" "${fake_elf_package}"
ExpectFailure 'not a 32-bit little-endian ARM executable' \
  "${fake_elf_root}/firmware/tools/verify_package.sh" \
  "${fake_elf_package}"

tampered_root="$(CreateFixture package-payload-tamper)"
tampered_package="$(CreatePackage "${tampered_root}" tampered)"
printf '%s\n' 'tampered package ELF' \
  >"${tampered_package}/mentor_pi_mcu-firmware-adrc-release.elf"
ExpectFailure 'SHA256SUMS differs from the exact package payload' \
  "${tampered_root}/firmware/tools/verify_package.sh" "${tampered_package}"

extra_root="$(CreateFixture package-extra-entry)"
extra_package="$(CreatePackage "${extra_root}" extra)"
printf '%s\n' 'unexpected' >"${extra_package}/unexpected.txt"
ExpectFailure 'exact seven-file contract' \
  "${extra_root}/firmware/tools/verify_package.sh" "${extra_package}"

mode_root="$(CreateFixture package-mode-tamper)"
mode_package="$(CreatePackage "${mode_root}" mode)"
awk '{sub("NORMAL_CLOSED_LOOP_DEFAULT", "UNSAFE_ALTERNATE"); print}' \
  "${mode_package}/BUILD-MODE.txt" >"${mode_package}/BUILD-MODE.txt.tmp"
mv "${mode_package}/BUILD-MODE.txt.tmp" "${mode_package}/BUILD-MODE.txt"
WritePackageChecksums "${mode_root}" "${mode_package}"
ExpectFailure 'BUILD-MODE.txt is not the NORMAL_CLOSED_LOOP_DEFAULT contract' \
  "${mode_root}/firmware/tools/verify_package.sh" "${mode_package}"

metadata_root="$(CreateFixture package-metadata-tamper)"
metadata_package="$(CreatePackage "${metadata_root}" metadata)"
awk '{sub("classification=NORMAL_CLOSED_LOOP_DEFAULT", \
          "classification=UNSAFE_ALTERNATE"); print}' \
  "${metadata_package}/BUILD-METADATA.txt" \
  >"${metadata_package}/BUILD-METADATA.txt.tmp"
mv "${metadata_package}/BUILD-METADATA.txt.tmp" \
  "${metadata_package}/BUILD-METADATA.txt"
WritePackageChecksums "${metadata_root}" "${metadata_package}"
ExpectFailure 'unsupported identity or motor mode' \
  "${metadata_root}/firmware/tools/verify_package.sh" "${metadata_package}"

if [[ -f /.dockerenv ]]; then
  ExpectFailure 'physical host' env \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    "${FLASH_SOURCE}" /dev/null
fi

ack_root="$(CreateFixture acknowledgement)"
ExpectFailure 'set FLASH_ACK=' env \
  PATH="${ack_root}/bin:/usr/bin:/bin" \
  STM32_CUBE_PROGRAMMER_CLI="${ack_root}/STM32_Programmer_CLI" \
  "${ack_root}/firmware/tools/flash.sh" /dev/null
[[ ! -e "${ack_root}/boot.log" && ! -e "${ack_root}/programmer.log" ]] || \
  Fail "a command ran before the safety acknowledgement"

success_root="$(CreateFixture success)"
RunFlash "${success_root}" >/dev/null
printf '%s\n' bootloader application >"${success_root}/expected-boot.log"
printf '%s\n' preflight flash >"${success_root}/expected-programmer.log"
cmp "${success_root}/expected-boot.log" "${success_root}/boot.log" || \
  Fail "successful automatic boot sequence differs"
cmp "${success_root}/expected-programmer.log" \
  "${success_root}/programmer.log" || \
  Fail "successful programmer sequence differs"

snapshot_root="$(CreateFixture snapshot-hash-mismatch)"
printf '%s\n' \
  'elf_sha256=0000000000000000000000000000000000000000000000000000000000000000' \
  >"${snapshot_root}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'firmware changed during flash snapshot' RunFlash "${snapshot_root}"
[[ ! -e "${snapshot_root}/boot.log" && \
   ! -e "${snapshot_root}/programmer.log" ]] || \
  Fail "snapshot hash failure reached boot control or CubeProgrammer"

preflight_root="$(CreateFixture preflight-failure)"
ExpectFailure 'MCU remains in bootloader mode' RunFlash "${preflight_root}" \
  FAKE_PREFLIGHT_STATUS=17
[[ "$(cat "${preflight_root}/boot.log")" == bootloader ]] || \
  Fail "preflight failure reset the application"
[[ "$(cat "${preflight_root}/programmer.log")" == preflight ]] || \
  Fail "programming ran after preflight failure"

flash_root="$(CreateFixture flash-failure)"
ExpectFailure 'programming or read-back verification failed' \
  RunFlash "${flash_root}" FAKE_FLASH_STATUS=18
[[ "$(cat "${flash_root}/boot.log")" == bootloader ]] || \
  Fail "program failure reset the application"

reset_root="$(CreateFixture reset-failure)"
ExpectFailure 'automatic normal-boot reset failed' RunFlash "${reset_root}" \
  FAKE_BOOT_FAILURE_MODE=application
printf '%s\n' bootloader application >"${reset_root}/expected-boot.log"
cmp "${reset_root}/expected-boot.log" "${reset_root}/boot.log" || \
  Fail "normal-reset failure sequence differs"

manual_root="$(CreateFixture manual)"
RunFlash "${manual_root}" RRCLITE_AUTOMATIC_BOOT_CONTROL=0 >/dev/null
[[ ! -e "${manual_root}/boot.log" ]] || \
  Fail "manual fallback invoked modem control"
[[ "$(cat "${manual_root}/programmer.log")" == flash ]] || \
  Fail "manual fallback did not run exactly one programming command"

package_flash_root="$(CreateFixture package-flash)"
flash_package="$(CreatePackage "${package_flash_root}" transfer)"
rm -f "${package_flash_root}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf" \
  "${package_flash_root}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
RunPackageFlash "${package_flash_root}" "${flash_package}" >/dev/null
printf '%s\n' bootloader application \
  >"${package_flash_root}/expected-package-boot.log"
printf '%s\n' preflight flash \
  >"${package_flash_root}/expected-package-programmer.log"
cmp "${package_flash_root}/expected-package-boot.log" \
  "${package_flash_root}/boot.log" || \
  Fail "package flash boot sequence differs"
cmp "${package_flash_root}/expected-package-programmer.log" \
  "${package_flash_root}/programmer.log" || \
  Fail "package flash programmer sequence differs"

echo "Firmware package validation and automatic CH9102F flash orchestration tests passed."
