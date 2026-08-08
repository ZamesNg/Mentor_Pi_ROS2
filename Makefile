SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

SYSTEM_ARCH := $(shell uname -m | sed \
	-e 's/^x86_64$$/amd64/' \
	-e 's/^aarch64$$/arm64/')
AGENT_EVIDENCE_ID ?=

PORT ?= /dev/mentor_pi_mcu
VEHICLE_CONFIG ?=
SERIAL_USER ?=
FLASH_ACK ?=
COMMISSIONING_BUILD_ACK ?=
COMMISSIONING_FLASH_ACK ?=
ROS_DOMAIN_ID ?= 0
RUNTIME_ACK ?=
SERIAL_SETUP_ACK ?=
PASSIVE_CHECK_ACK ?=
OLED_PRESENT ?=
PERIPHERAL_SMOKE_ACK ?=
RECOVERY_CHECK_ACK ?=
CHARACTERIZATION_ACK ?=
COMMISSIONING_RUN_ACK ?=
MOTOR_ID ?=
TARGET_RPS ?=
DURATION_MS ?=
PHYSICAL_DIRECTION_CONFIRMED ?=
PREFLIGHT_ACK ?=
RELEASE_GATES_ACK ?=
CAMPAIGN_FIXTURE_ACK ?=
FIXTURE_REVISION ?=
CAMPAIGN_BUS_ID ?=
CAMPAIGN_BUS_HOLD ?=
CAMPAIGN_BUS_TOLERANCE ?=
CAMPAIGN_BUS_OFFSET ?=
CAMPAIGN_BUS_TORQUE ?=
RECOVERY_MODE ?=

.PHONY: help doctor setup firmware firmware-commissioning host host-hardwares agent \
	agent-check run start start-commissioning start-commissioning-pid shell test serial-access serial-setup flash \
	start-hardware start-mecanum start-ackermann \
	flash-locked flash-commissioning flash-commissioning-pid firmware-commissioning-pid \
	build-commissioning build-commissioning-pid commission-motor \
	restore-locked passive-check peripheral-smoke recovery-check \
	characterize-board hil-start hil-peripheral-check hil-recovery-check \
	release-software-gates qualification-preflight campaign-load \
	campaign-soak campaign-recovery

help:
	@printf '%s\n' \
		'RRCLite v2 build and flash commands' \
		'' \
		'  make doctor' \
		'      Check Git, Make, Docker, architecture, disk space, and CubeProgrammer.' \
		'  make setup' \
		'      Fetch pinned sources and images for the detected architecture.' \
		'  make firmware' \
		'      Generate micro-ROS and build the default motor-locked firmware.' \
		'  make firmware-commissioning-pid' \
		'      Build the acknowledged commissioning firmware in closed-loop PID mode.' \
		'  make host' \
		'      Build/test Humble natively on Ubuntu 22.04, otherwise in Docker.' \
		'  make host-hardwares' \
		'      Alias for the same adaptive build/test path as make host.' \
		'  make agent' \
		'      Build the pinned Humble Agent natively on 22.04, otherwise in Docker.' \
		'  make run PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0' \
		'      RUNTIME_ACK=LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED' \
		'      Run against the MCU natively on 22.04, otherwise in Docker.' \
		'  make start' \
		'      Interactively verify passive safety and start the adaptive runtime.' \
		'  make start-commissioning' \
		'      Start the adaptive runtime only for a verified commissioning image.' \
		'  make start-commissioning-pid' \
		'      Start only for a verified closed-loop PID commissioning image.' \
		'  make start-hardware VEHICLE_CONFIG=/absolute/robot.yaml' \
		'      Start Agent, supervisor, and the YAML-selected ros2_control adapter.' \
		'  make start-mecanum | make start-ackermann' \
		'      Convenience wrappers using the checked-in YAML profiles.' \
		'  make shell ROS_DOMAIN_ID=0' \
		'      Open a sourced ROS shell for the running runtime.' \
		'  make agent-check' \
		'      Capture separate pinned-Agent build compatibility evidence.' \
		'  make test' \
		'      Run the Humble host gate plus portable, tooling, format, and doc checks.' \
		'  sudo make serial-access PORT=/dev/serial/by-id/DEVICE SERIAL_USER=LOGIN' \
		'      Verify one CH9102F, install /dev/mentor_pi_mcu, and grant the' \
		'      named development user access through mentor-pi-serial.' \
		'  make serial-setup' \
		'      Interactively detect the CH9102F and configure the stable alias.' \
		'  make flash PORT=/dev/mentor_pi_mcu' \
		'      FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED' \
		'      Low-level manual-boot flash interface for automation.' \
		'  make flash-locked' \
		'      Automatically enter the ROM bootloader, flash, verify, and reset.' \
		'  make passive-check | peripheral-smoke | recovery-check' \
		'      Run the guided passive board operations from the tutorials.' \
		'' \
		'Commissioning (wheels raised and current limiting enabled):' \
		'  make build-commissioning' \
		'  make flash-commissioning' \
		'  make build-commissioning-pid' \
		'  make flash-commissioning-pid' \
		'  make commission-motor' \
		'  make restore-locked' \
		'      Each command prompts for its exact fail-closed acknowledgement.' \
		'' \
		'Qualification:' \
		'  make qualification-preflight' \
		'  make campaign-load | campaign-soak | campaign-recovery' \
		'      Prompt for reviewed fixture values and preserve generated evidence.'

doctor:
	@./tools/doctor.sh

setup: doctor
	@if [[ "$(SYSTEM_ARCH)" != 'amd64' && "$(SYSTEM_ARCH)" != 'arm64' ]]; then \
		echo 'Unsupported system architecture; expected x86_64 or aarch64.' >&2; \
		exit 1; \
	fi
	@./tools/bootstrap_firmware_dependencies.sh
	@if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then \
		./tools/pull_pinned_build_images.sh --architecture "$(SYSTEM_ARCH)"; \
	elif grep -Eq '^VERSION_ID="?22[.]04"?$$' /etc/os-release; then \
		echo 'Native Ubuntu 22.04/Humble mode: Docker image pull skipped.'; \
	else \
		echo 'Docker is required outside native Ubuntu 22.04.' >&2; \
		exit 1; \
	fi
	@printf '%s\n' \
		'Setup complete. Pinned sources and available build dependencies are ready.'

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

firmware-commissioning-pid:
	@if [[ "$(COMMISSIONING_BUILD_ACK)" != 'MOTORS_RAISED' ]]; then \
		printf '%s\n' \
			'Refusing commissioning PID build.' \
			'Raise all wheels, enable the current limit, then set:' \
			'  COMMISSIONING_BUILD_ACK=MOTORS_RAISED' >&2; \
		exit 1; \
	fi
	@./tools/bootstrap_firmware_dependencies.sh
	@./tools/build_microros_library.sh
	@RRCLITE_MOTOR_COMMISSIONING=1 \
		RRCLITE_MOTOR_COMMISSIONING_CLOSED_LOOP=1 \
		RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
		./tools/build_firmware.sh

host:
	@./tools/build_host.sh

host-hardwares: host

agent:
	@./tools/build_agent.sh

agent-check:
	@if [[ "$(SYSTEM_ARCH)" != 'amd64' && "$(SYSTEM_ARCH)" != 'arm64' ]]; then \
		echo 'Unsupported system architecture; expected x86_64 or aarch64.' >&2; \
		exit 1; \
	fi
	@args=(--architecture "$(SYSTEM_ARCH)"); \
		if [[ -n "$(AGENT_EVIDENCE_ID)" ]]; then \
			args+=(--evidence-id "$(AGENT_EVIDENCE_ID)"); \
		fi; \
		./tools/verify_microros_agent_build_container.sh "$${args[@]}"

run:
	@RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/run_runtime.sh \
			--device "$(PORT)" \
			--ros-domain-id "$(ROS_DOMAIN_ID)"

start:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start

start-commissioning:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start-commissioning

start-commissioning-pid:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start-commissioning-pid

start-hardware:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		VEHICLE_CONFIG="$(VEHICLE_CONFIG)" RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start-hardware

start-mecanum:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		VEHICLE_CONFIG="$(CURDIR)/mentor_pi_ros2/src/mentor_pi_hardwares/config/mecanum/hardware.yaml" \
		RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start-mecanum

start-ackermann:
	@PORT="$(PORT)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		VEHICLE_CONFIG="$(CURDIR)/mentor_pi_ros2/src/mentor_pi_hardwares/config/ackermann/hardware.yaml" \
		RRCLITE_RUNTIME_ACK="$(RUNTIME_ACK)" \
		./tools/tutorial_action.sh start-ackermann

shell:
	@./tools/open_runtime_shell.sh --ros-domain-id "$(ROS_DOMAIN_ID)"

test: host
	@./tools/run_quality_tests_container.sh

serial-access:
	@./tools/configure_dev_serial_access.sh \
		--device "$(PORT)" --user "$(SERIAL_USER)"

serial-setup:
	@SERIAL_SETUP_ACK="$(SERIAL_SETUP_ACK)" \
		./tools/tutorial_action.sh serial-setup

flash:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		./tools/flash_firmware.sh LOCKED "$(PORT)"

flash-locked:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		./tools/guided_flash.sh LOCKED "$(PORT)"

flash-commissioning:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_COMMISSIONING_FLASH_ACK="$(COMMISSIONING_FLASH_ACK)" \
		./tools/guided_flash.sh COMMISSIONING "$(PORT)"

flash-commissioning-pid:
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		RRCLITE_COMMISSIONING_FLASH_ACK="$(COMMISSIONING_FLASH_ACK)" \
		./tools/guided_flash.sh COMMISSIONING_PID "$(PORT)"

build-commissioning:
	@COMMISSIONING_BUILD_ACK="$(COMMISSIONING_BUILD_ACK)" \
		./tools/tutorial_action.sh build-commissioning

build-commissioning-pid:
	@COMMISSIONING_BUILD_ACK="$(COMMISSIONING_BUILD_ACK)" \
		./tools/tutorial_action.sh build-commissioning-pid

commission-motor:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		COMMISSIONING_RUN_ACK="$(COMMISSIONING_RUN_ACK)" \
		MOTOR_ID="$(MOTOR_ID)" TARGET_RPS="$(TARGET_RPS)" \
		DURATION_MS="$(DURATION_MS)" \
		PHYSICAL_DIRECTION_CONFIRMED="$(PHYSICAL_DIRECTION_CONFIRMED)" \
		./tools/tutorial_action.sh commission-motor

restore-locked: firmware
	@RRCLITE_UART_BOOTLOADER_ACK="$(FLASH_ACK)" \
		./tools/guided_flash.sh LOCKED "$(PORT)"

passive-check:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" PASSIVE_CHECK_ACK="$(PASSIVE_CHECK_ACK)" \
		OLED_PRESENT="$(OLED_PRESENT)" \
		./tools/tutorial_action.sh passive-check

peripheral-smoke:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		PERIPHERAL_SMOKE_ACK="$(PERIPHERAL_SMOKE_ACK)" \
		OLED_PRESENT="$(OLED_PRESENT)" \
		./tools/tutorial_action.sh peripheral-smoke

recovery-check:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" RECOVERY_CHECK_ACK="$(RECOVERY_CHECK_ACK)" \
		./tools/tutorial_action.sh recovery-check

characterize-board:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		CHARACTERIZATION_ACK="$(CHARACTERIZATION_ACK)" \
		./tools/tutorial_action.sh characterize-board

hil-start:
	@./tools/tutorial_action.sh hil-start

hil-peripheral-check:
	@./tools/tutorial_action.sh hil-peripheral-check

hil-recovery-check:
	@./tools/tutorial_action.sh hil-recovery-check

release-software-gates:
	@RELEASE_GATES_ACK="$(RELEASE_GATES_ACK)" \
		./tools/tutorial_action.sh release-software-gates

qualification-preflight:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" PREFLIGHT_ACK="$(PREFLIGHT_ACK)" \
		./tools/tutorial_action.sh qualification-preflight

campaign-load campaign-soak campaign-recovery:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		CAMPAIGN_FIXTURE_ACK="$(CAMPAIGN_FIXTURE_ACK)" \
		FIXTURE_REVISION="$(FIXTURE_REVISION)" \
		CAMPAIGN_BUS_ID="$(CAMPAIGN_BUS_ID)" \
		CAMPAIGN_BUS_HOLD="$(CAMPAIGN_BUS_HOLD)" \
		CAMPAIGN_BUS_TOLERANCE="$(CAMPAIGN_BUS_TOLERANCE)" \
		CAMPAIGN_BUS_OFFSET="$(CAMPAIGN_BUS_OFFSET)" \
		CAMPAIGN_BUS_TORQUE="$(CAMPAIGN_BUS_TORQUE)" \
		RECOVERY_MODE="$(RECOVERY_MODE)" \
		./tools/tutorial_action.sh "$@"
