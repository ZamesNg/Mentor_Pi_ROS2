SHELL := /usr/bin/env bash
.DEFAULT_GOAL := help

ROS_DISTRO ?= humble
BUILD_JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)
.PHONY: help doctor deps build test clean

help:
	@printf '%s\n' \
		'Mentor Pi ROS 2 workspace (Ubuntu 22.04 / Humble)' \
		'' \
		'  make doctor  Verify the native/Dev Container environment.' \
		'  make deps    Import pinned ALTO sources and run rosdep.' \
		'  make build   Build only packages below ros2_ws/src.' \
		'  make test    Run package tests and report their results.' \
		'  make clean   Remove this workspace build/install/log output.'

doctor:
	@./tools/check_environment.sh

deps: doctor
	@./tools/setup_dependencies.sh

build: doctor
	@./tools/colcon.sh build --symlink-install \
		--parallel-workers "$(BUILD_JOBS)"

test: build
	@./tools/colcon.sh test --parallel-workers "$(BUILD_JOBS)"
	@./tools/colcon.sh test-result --verbose

clean:
	@./tools/clean.sh
