#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

for path in "${WORKSPACE_ROOT}/build" "${WORKSPACE_ROOT}/install" \
    "${WORKSPACE_ROOT}/log"; do
  case "${path}" in
    "${WORKSPACE_ROOT}"/build | "${WORKSPACE_ROOT}"/install | \
      "${WORKSPACE_ROOT}"/log)
      rm -rf -- "${path}"
      ;;
    *)
      echo "Refusing unsafe workspace cleanup: ${path}" >&2
      exit 1
      ;;
  esac
done
