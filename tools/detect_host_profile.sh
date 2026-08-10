#!/usr/bin/env bash

set -euo pipefail

readonly MODEL_PATH="${RRCLITE_DEVICE_TREE_MODEL:-/proc/device-tree/model}"

Fail() {
  echo "Host-profile detection error: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || Fail "usage: ./tools/detect_host_profile.sh"

case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
  *) Fail "unsupported architecture: $(uname -m)" ;;
esac

profile=normal
model=""
if [[ -r "${MODEL_PATH}" ]]; then
  model="$(tr -d '\000' <"${MODEL_PATH}")"
  if [[ "${architecture}" == arm64 && \
        "${model,,}" =~ (rdk.*x5|x5.*rdk|d-robotics.*x5|hobot.*x5) ]]; then
    profile=rdk-x5
  fi
fi

printf '%s\n' \
  "profile=${profile}" \
  "architecture=${architecture}" \
  "model=${model:-unknown}"
