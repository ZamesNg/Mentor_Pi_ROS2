#!/usr/bin/env bash

set -euo pipefail

os_release=""
architecture=""
declare -a repository_paths=()
declare -a repository_origins=()
declare -a repository_commits=()

Usage() {
  cat >&2 <<'EOF'
Usage: verify_microros_agent_install_state.sh \
  --os-release PATH --architecture amd64|arm64 \
  [--repository PATH --origin URL --commit 40_HEX_SHA ...]
EOF
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --os-release)
      os_release="${2:-}"
      shift 2
      ;;
    --architecture)
      architecture="${2:-}"
      shift 2
      ;;
    --repository)
      repository_paths+=("${2:-}")
      shift 2
      ;;
    --origin)
      repository_origins+=("${2:-}")
      shift 2
      ;;
    --commit)
      repository_commits+=("${2:-}")
      shift 2
      ;;
    *) Usage ;;
  esac
done

[[ -n "${os_release}" && -n "${architecture}" ]] || Usage
if ((${#repository_paths[@]} != ${#repository_origins[@]} ||
      ${#repository_paths[@]} != ${#repository_commits[@]})); then
  Usage
fi

Fail() {
  echo "micro-ROS Agent install-state error: $*" >&2
  exit 1
}

ReadOsReleaseValue() {
  local key="$1"
  local count=""
  local value=""
  count="$(grep -Ec "^${key}=" "${os_release}" || true)"
  [[ "${count}" == "1" ]] ||
    Fail "${os_release} must contain exactly one ${key}= entry"
  value="$(sed -n "s/^${key}=//p" "${os_release}")"
  if [[ "${value}" == \"*\" && "${value}" == *\" ]]; then
    value="${value#\"}"
    value="${value%\"}"
  fi
  printf '%s' "${value}"
}

[[ -r "${os_release}" ]] || Fail "cannot read OS identity ${os_release}"
[[ "$(ReadOsReleaseValue ID)" == "ubuntu" ]] ||
  Fail "the deployment host must be Ubuntu"
[[ "$(ReadOsReleaseValue VERSION_ID)" == "24.04" ]] ||
  Fail "the deployment host must be Ubuntu 24.04"
case "${architecture}" in
  amd64|arm64) ;;
  *) Fail "only Ubuntu 24.04 amd64 and arm64 are supported" ;;
esac

for ((index = 0; index < ${#repository_paths[@]}; ++index)); do
  repository="${repository_paths[index]}"
  expected_origin="${repository_origins[index]}"
  expected_commit="${repository_commits[index]}"
  [[ "${expected_commit}" =~ ^[0-9a-f]{40}$ ]] ||
    Fail "expected commit for ${repository} is not a lowercase 40-hex SHA"
  [[ -d "${repository}/.git" ]] ||
    Fail "${repository} is not the expected standalone Git checkout"

  actual_origin="$(git -C "${repository}" remote get-url origin 2>/dev/null)" ||
    Fail "${repository} has no readable origin remote"
  [[ "${actual_origin}" == "${expected_origin}" ]] ||
    Fail "origin mismatch at ${repository}: ${actual_origin}"

  actual_commit="$(git -C "${repository}" rev-parse --verify HEAD 2>/dev/null)" ||
    Fail "cannot resolve HEAD at ${repository}"
  [[ "${actual_commit}" == "${expected_commit}" ]] ||
    Fail "revision mismatch at ${repository}: ${actual_commit}"
  if git -C "${repository}" symbolic-ref -q HEAD >/dev/null 2>&1; then
    Fail "${repository} must remain detached at its pinned revision"
  fi
  if [[ -n "$(git -C "${repository}" status --porcelain=v1 \
      --untracked-files=all)" ]]; then
    Fail "${repository} has modified or untracked source files"
  fi
done

echo "Verified Ubuntu 24.04 ${architecture} and pinned Agent source state."
