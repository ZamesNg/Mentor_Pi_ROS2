SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_DOMAIN_ID ?= 0
DEVICE ?= /dev/mentor_pi_mcu
ID_SERIAL_SHORT ?=
ID_PATH ?=
RELEASE_ID ?=
BUILD_JOBS ?= $(shell nproc 2>/dev/null || echo 1)

.PHONY: help doctor setup build test install-service clean

help:
	@printf '%s\n' \
		'Mentor Pi micro-ROS Agent (Ubuntu 22.04 / Humble)' \
		'' \
		'  make doctor         Verify the supported environment.' \
		'  make setup          Fetch sources and install ROS dependencies.' \
		'  make build          Build the patched, pinned Agent natively.' \
		'  make test           Validate locks, patch, metadata, and executable.' \
		'  sudo make install-service DEVICE=/dev/ttyUSB0 ID_SERIAL_SHORT=VALUE' \
		'                       Install and enable mentor-pi-agent.service.' \
		'  make clean          Remove this component build directory.'

doctor:
	@./tools/check_environment.sh

setup: doctor
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh --setup

build: doctor
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh

test: build
	@./test/contract_test.sh

install-service:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" DEVICE="$(DEVICE)" \
		ID_SERIAL_SHORT="$(ID_SERIAL_SHORT)" ID_PATH="$(ID_PATH)" \
		RELEASE_ID="$(RELEASE_ID)" ./tools/install_service.sh

clean:
	@./tools/clean.sh
