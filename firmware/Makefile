SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

PORT ?= /dev/mentor_pi_mcu
FLASH_ACK ?=
AUTOMATIC_BOOT_CONTROL ?= 1
PACKAGE_OUTPUT ?=
BUILD_JOBS ?= $(shell nproc 2>/dev/null || echo 1)

.PHONY: help doctor setup extract-sdk test build firmware verify package boot-control flash microros-sdk clean

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
		'  make microros-sdk  Regenerate and replace the checked SDK (maintainers).' \
		'  make clean         Remove firmware build output.'

doctor:
	@./tools/check_environment.sh

setup: doctor
	@./tools/setup.sh

extract-sdk:
	@./tools/extract_microros_sdk.sh

test: doctor extract-sdk
	@./tools/test_sdk_validation.sh
	@./tools/test.sh "$(BUILD_JOBS)"
	@./tools/test_flash_sequence.sh

build firmware: doctor extract-sdk
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh

verify:
	@./tools/verify.sh
	@./tools/check_memory.sh

package: verify
	@./tools/package.sh "$(PACKAGE_OUTPUT)"

boot-control: doctor extract-sdk
	@./tools/build_boot_control.sh

flash: verify
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_AUTOMATIC_BOOT_CONTROL="$(AUTOMATIC_BOOT_CONTROL)" \
		./tools/flash.sh "$(PORT)"

microros-sdk: doctor setup
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/generate_microros_sdk.sh

clean:
	@./tools/clean.sh
