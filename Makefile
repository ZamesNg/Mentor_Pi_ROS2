SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

HOST_ARCH ?= $(shell uname -m | sed \
	-e 's/^x86_64$$/amd64/' \
	-e 's/^aarch64$$/arm64/')
DEFAULT_HOST_RELEASE_ID := dev-$(shell date -u +%Y%m%dT%H%M%SZ)
HOST_RELEASE_ID ?= $(DEFAULT_HOST_RELEASE_ID)
HOST_OUTPUT ?= build/host-handoff/$(HOST_RELEASE_ID)-$(HOST_ARCH)
AGENT_EVIDENCE_ID ?=

PORT ?=
FLASH_ACK ?=
COMMISSIONING_BUILD_ACK ?=
COMMISSIONING_FLASH_ACK ?=

.PHONY: help doctor setup firmware firmware-commissioning host agent test flash \
	flash-commissioning

help:
	@printf '%s\n' \
		'RRCLite v2 build and flash commands' \
		'' \
		'  make doctor' \
		'      Check Git, Make, Docker, architecture, disk space, and CubeProgrammer.' \
		'  make setup' \
		'      Fetch pinned firmware sources and images for HOST_ARCH.' \
		'  make firmware' \
		'      Generate micro-ROS and build the default motor-locked firmware.' \
		'  make host' \
		'      Build and test the Humble host packages in the pinned container.' \
		'  make agent' \
		'      Build the pinned Humble Agent and capture compatibility evidence.' \
		'  make test' \
		'      Run the Humble host gate plus portable, tooling, format, and doc checks.' \
		'  make flash PORT=/dev/ttyUSB0' \
		'      FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED' \
		'      Verify and flash the current motor-locked firmware.' \
		'' \
		'Commissioning (wheels raised and current limiting enabled):' \
		'  make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED' \
		'  make flash-commissioning PORT=/dev/ttyUSB0' \
		'      FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED' \
		'      COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED'

doctor:
	@./tools/doctor.sh

setup: doctor
	@if [[ "$(HOST_ARCH)" != 'amd64' && "$(HOST_ARCH)" != 'arm64' ]]; then \
		echo 'Unsupported setup architecture. Set HOST_ARCH=amd64 or HOST_ARCH=arm64.' >&2; \
		exit 1; \
	fi
	@./tools/bootstrap_firmware_dependencies.sh
	@./tools/pull_pinned_build_images.sh --architecture "$(HOST_ARCH)"
	@printf '%s\n' \
		'Setup complete. Builder layers are cached; make firmware creates local images.'

firmware:
	@./tools/bootstrap_firmware_dependencies.sh
	@./tools/build_microros_library.sh
	@./tools/build_firmware.sh

firmware-commissioning:
	@if [[ "$(COMMISSIONING_BUILD_ACK)" != 'MOTORS_RAISED' ]]; then \
		printf '%s\n' \
			'Refusing commissioning build.' \
			'Raise all wheels, enable the current limit, then set:' \
			'  COMMISSIONING_BUILD_ACK=MOTORS_RAISED' >&2; \
		exit 1; \
	fi
	@./tools/bootstrap_firmware_dependencies.sh
	@./tools/build_microros_library.sh
	@RRCLITE_MOTOR_COMMISSIONING=1 \
		RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
		./tools/build_firmware.sh

host:
	@if [[ "$(HOST_ARCH)" != 'amd64' && "$(HOST_ARCH)" != 'arm64' ]]; then \
		echo 'Unsupported host architecture. Set HOST_ARCH=amd64 or HOST_ARCH=arm64.' >&2; \
		exit 1; \
	fi
	@./tools/build_host_handoff_container.sh \
		--architecture "$(HOST_ARCH)" \
		--release-id "$(HOST_RELEASE_ID)" \
		--output-directory "$(HOST_OUTPUT)"

agent:
	@if [[ "$(HOST_ARCH)" != 'amd64' && "$(HOST_ARCH)" != 'arm64' ]]; then \
		echo 'Unsupported Agent architecture. Set HOST_ARCH=amd64 or HOST_ARCH=arm64.' >&2; \
		exit 1; \
	fi
	@args=(--architecture "$(HOST_ARCH)"); \
		if [[ -n "$(AGENT_EVIDENCE_ID)" ]]; then \
			args+=(--evidence-id "$(AGENT_EVIDENCE_ID)"); \
		fi; \
		./tools/verify_microros_agent_build_container.sh "$${args[@]}"

test: host
	@./tools/run_quality_tests_container.sh

flash:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		./tools/flash_firmware.sh LOCKED "$(PORT)"

flash-commissioning:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_COMMISSIONING_FLASH_ACK="$(COMMISSIONING_FLASH_ACK)" \
		./tools/flash_firmware.sh COMMISSIONING "$(PORT)"
