SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

DEVICE ?=
ID_SERIAL_SHORT ?=
ID_PATH ?=
RELEASE_ID ?=
ROS_LOCALHOST_ONLY ?= 0
BUILD_JOBS ?= $(shell nproc 2>/dev/null || echo 1)

.PHONY: help doctor find-device setup build test install-service clean

help:
	@printf '%s\n' \
		'Mentor Pi micro-ROS Agent (Ubuntu 22.04 / Humble)' \
		'' \
		'  make doctor         Verify the supported environment.' \
		'  make find-device    Locate exactly one connected Mentor Pi CH9102F.' \
		'  make setup          Fetch sources and install ROS dependencies.' \
		'  make build          Build the patched, pinned Agent natively.' \
		'  make test           Validate locks, patch, metadata, and executable.' \
		'  sudo make install-service ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0' \
		'                       Install using the deployment DDS boundary.' \
		'  sudo make install-service ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1' \
		'                       Install loopback DDS for an onboard Zenoh bridge.' \
		'  make clean          Remove this component build directory.'

doctor:
	@./tools/check_environment.sh

find-device:
	@./tools/find_device.sh --path

setup: doctor
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh --setup

build: doctor
	@RRCLITE_BUILD_JOBS="$(BUILD_JOBS)" ./tools/build.sh

test: build
	@./test/source_checkout_test.sh
	@./test/device_discovery_test.sh
	@./test/configure_serial_access_test.sh
	@./test/contract_test.sh

install-service:
	@ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
		ROS_LOCALHOST_ONLY="$(ROS_LOCALHOST_ONLY)" DEVICE="$(DEVICE)" \
		ID_SERIAL_SHORT="$(ID_SERIAL_SHORT)" ID_PATH="$(ID_PATH)" \
		RELEASE_ID="$(RELEASE_ID)" ./tools/install_service.sh

clean:
	@./tools/clean.sh
