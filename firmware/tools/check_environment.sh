#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Firmware environment error: $*" >&2
  exit 1
}

[[ "$(uname -s)" == Linux ]] || \
  Fail "use the repository VS Code Dev Container on macOS"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "native builds require Ubuntu 22.04; use the VS Code Dev Container elsewhere"
case "$(uname -m)" in
  x86_64 | amd64 | aarch64 | arm64) ;;
  *) Fail "only amd64 and arm64 Linux builds are supported" ;;
esac
for command in cmake git ninja tar xz; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is unavailable"
done
echo "Firmware environment: Ubuntu 22.04 ($(uname -m))."
