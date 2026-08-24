#!/usr/bin/env bash

set -euo pipefail

[[ "$#" -ge 1 ]] || { echo "Usage: sha256.sh FILE..." >&2; exit 2; }
if command -v sha256sum >/dev/null 2>&1; then
  sha256sum "$@" | awk '{print $1}'
elif command -v shasum >/dev/null 2>&1; then
  shasum -a 256 "$@" | awk '{print $1}'
else
  echo "Neither sha256sum nor shasum is available" >&2
  exit 1
fi
