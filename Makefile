SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_DOMAIN_ID ?= 0
PORT ?= /dev/mentor_pi_mcu
SERIAL_SETUP_ACK ?=
PASSIVE_CHECK_ACK ?=
PERIPHERAL_SMOKE_ACK ?=
CHARACTERIZATION_ACK ?=
PREFLIGHT_ACK ?=
OLED_PRESENT ?=
CAMPAIGN_FIXTURE_ACK ?=
FIXTURE_REVISION ?=
CAMPAIGN_BUS_ID ?=
CAMPAIGN_BUS_HOLD ?=
CAMPAIGN_BUS_TOLERANCE ?=
CAMPAIGN_BUS_OFFSET ?=
CAMPAIGN_BUS_TORQUE ?=
RECOVERY_MODE ?=
RUNTIME_CONTEXT ?= development
PACKAGED_FIRMWARE_SHA256 ?=

.PHONY: help doctor check-compatibility install-evidence-tools serial-setup passive-check \
	peripheral-smoke characterize-board qualification-preflight \
	campaign-load campaign-soak campaign-recovery

help:
	@printf '%s\n' \
		'Mentor Pi integration and HIL commands' \
		'' \
		'Component build commands:' \
		'  make -C firmware help' \
		'  make -C micro_ros_agent help' \
		'  make -C ros2_ws help' \
		'' \
		'Integration commands:' \
		'  make doctor' \
		'  make check-compatibility' \
		'  sudo make install-evidence-tools' \
		'  make serial-setup' \
		'  make passive-check | peripheral-smoke | characterize-board' \
		'  make qualification-preflight' \
		'  make campaign-load | campaign-soak | campaign-recovery' \
		'' \
		'Root integration actions require native Ubuntu 22.04.'

doctor:
	@./tools/doctor.sh

check-compatibility:
	@./tools/check_compatibility.sh

install-evidence-tools:
	@./tools/install_evidence_tools.sh

serial-setup:
	@PORT="$(PORT)" SERIAL_SETUP_ACK="$(SERIAL_SETUP_ACK)" \
		./tools/tutorial_action.sh serial-setup

passive-check:
	@./tools/check_compatibility.sh
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" PASSIVE_CHECK_ACK="$(PASSIVE_CHECK_ACK)" \
		OLED_PRESENT="$(OLED_PRESENT)" \
		MENTOR_PI_RUNTIME_ACTION_CONTEXT="$(RUNTIME_CONTEXT)" \
		MENTOR_PI_PACKAGED_FIRMWARE_SHA256="$(PACKAGED_FIRMWARE_SHA256)" \
		./tools/tutorial_action.sh passive-check

peripheral-smoke:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		PERIPHERAL_SMOKE_ACK="$(PERIPHERAL_SMOKE_ACK)" \
		OLED_PRESENT="$(OLED_PRESENT)" \
		MENTOR_PI_RUNTIME_ACTION_CONTEXT="$(RUNTIME_CONTEXT)" \
		MENTOR_PI_PACKAGED_FIRMWARE_SHA256="$(PACKAGED_FIRMWARE_SHA256)" \
		./tools/tutorial_action.sh peripheral-smoke

characterize-board:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		CHARACTERIZATION_ACK="$(CHARACTERIZATION_ACK)" \
		MENTOR_PI_RUNTIME_ACTION_CONTEXT="$(RUNTIME_CONTEXT)" \
		MENTOR_PI_PACKAGED_FIRMWARE_SHA256="$(PACKAGED_FIRMWARE_SHA256)" \
		./tools/tutorial_action.sh characterize-board

qualification-preflight:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" PREFLIGHT_ACK="$(PREFLIGHT_ACK)" \
		MENTOR_PI_RUNTIME_ACTION_CONTEXT="$(RUNTIME_CONTEXT)" \
		MENTOR_PI_PACKAGED_FIRMWARE_SHA256="$(PACKAGED_FIRMWARE_SHA256)" \
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
		MENTOR_PI_RUNTIME_ACTION_CONTEXT="$(RUNTIME_CONTEXT)" \
		MENTOR_PI_PACKAGED_FIRMWARE_SHA256="$(PACKAGED_FIRMWARE_SHA256)" \
		./tools/tutorial_action.sh "$@"
