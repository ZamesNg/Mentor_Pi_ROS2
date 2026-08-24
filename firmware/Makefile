SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

PORT ?= /dev/mentor_pi_mcu
FLASH_ACK ?=
AUTOMATIC_BOOT_CONTROL ?= 1
PACKAGE_OUTPUT ?=
PACKAGE ?=
EXPECTED_NAMESPACE ?=
PACKAGE_MANIFEST_SHA256 ?=
BUILD_JOBS ?= $(shell nproc 2>/dev/null || echo 1)

.PHONY: help doctor setup extract-sdk test build firmware verify package boot-control flash flash-package microros-sdk clean

help:
	@printf '%s\n' \
		'Mentor Pi firmware' \
		'' \
		'  make doctor        Verify Ubuntu 22.04 or the VS Code Dev Container.' \
		'  make setup         Fetch pinned dependencies and Arm GNU 13.2.Rel1.' \
		'  make extract-sdk   Verify and extract the checked Humble SDK.' \
		'  make test          Run native C++ domain tests.' \
		'  make build         Build the STM32 ADRC firmware with CMake/Ninja.' \
		'  make verify        Verify provenance, hashes, and memory budgets.' \
		'  make package       Produce firmware-adrc-release/.' \
		'  make boot-control  Build the Linux CH9102F RTS/DTR reset helper.' \
		'  make flash PORT=/dev/mentor_pi_mcu FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED' \
		'                    Automatic bootloader entry/reset is enabled by default.' \
		'  make flash-package PACKAGE=/path/to/firmware-adrc-release EXPECTED_NAMESPACE=/robot' \
		'                    PACKAGE_MANIFEST_SHA256=64_HEX_DIGEST PORT=/dev/mentor_pi_mcu' \
		'                    FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED' \
		'                    Verify and flash a transferred release package.' \
		'  make microros-sdk  Regenerate and replace the checked SDK (maintainers).' \
		'  make clean         Remove firmware build output.'

doctor:
	@./tools/check_environment.sh

setup: doctor
	@./tools/setup.sh

extract-sdk:
	@./tools/extract_microros_sdk.sh

test: doctor extract-sdk
	@./tools/test_fingerprints.sh
	@./tools/test_sdk_validation.sh
	@./tools/test.sh "$(BUILD_JOBS)"
	@./tools/test_flash_sequence.sh

build firmware: doctor extract-sdk
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh

verify:
	@./tools/verify.sh
	@./tools/check_memory.sh --verified

package: verify
	@./tools/package.sh "$(PACKAGE_OUTPUT)"

boot-control: doctor extract-sdk
	@./tools/build_boot_control.sh

flash: verify
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_AUTOMATIC_BOOT_CONTROL="$(AUTOMATIC_BOOT_CONTROL)" \
		./tools/flash.sh "$(PORT)"

flash-package:
	@[[ -n "$(PACKAGE)" ]] || { \
		echo 'Set PACKAGE=/path/to/firmware-adrc-release' >&2; exit 2; }
	@[[ -n "$(EXPECTED_NAMESPACE)" ]] || { \
		echo 'Set EXPECTED_NAMESPACE=/the_intended_robot' >&2; exit 2; }
	@[[ -n "$(PACKAGE_MANIFEST_SHA256)" ]] || { \
		echo 'Set PACKAGE_MANIFEST_SHA256 to the separately recorded SHA256SUMS digest' >&2; exit 2; }
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_AUTOMATIC_BOOT_CONTROL="$(AUTOMATIC_BOOT_CONTROL)" \
		./tools/flash.sh --package "$(PACKAGE)" \
			--expected-namespace "$(EXPECTED_NAMESPACE)" \
			--expected-manifest-sha256 "$(PACKAGE_MANIFEST_SHA256)" \
			"$(PORT)"

microros-sdk: doctor setup
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/generate_microros_sdk.sh

clean:
	@./tools/clean.sh
