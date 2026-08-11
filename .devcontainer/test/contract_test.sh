#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEVCONTAINER_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${DEVCONTAINER_ROOT}/Dockerfile"
readonly ZSHRC="${DEVCONTAINER_ROOT}/zshrc"
readonly SEEDER="${DEVCONTAINER_ROOT}/seed-zsh-history.sh"

for revision in \
    97b27bb2ec0701330b18c2d3e340b22e742b3fa8 \
    85919cd1ffa7d2d5412f6d3fe437ebdbeeec4fc5 \
    c4d95591843d49838b7ad30081e7aba3135a6703; do
  grep -Fq "${revision}" "${DOCKERFILE}"
done
grep -Fq 'USER ${USERNAME}' "${DOCKERFILE}"
grep -Fq '"remoteUser": "mentor"' \
  "${DEVCONTAINER_ROOT}/devcontainer.json"
grep -Fq '"updateRemoteUserUID": true' \
  "${DEVCONTAINER_ROOT}/devcontainer.json"
grep -Fq 'sudo -H -u "${USERNAME}" rosdep update' "${DOCKERFILE}"
grep -Fq '.ros/rosdep/sources.cache/index' "${DOCKERFILE}"
grep -Fq 'apt-get install -y --no-install-recommends ripgrep' "${DOCKERFILE}"
grep -Fq 'zsh-autosuggestions' "${ZSHRC}"
grep -Fq 'zsh-syntax-highlighting' "${ZSHRC}"
grep -Fq 'terminal.integrated.defaultProfile.linux' \
  "${DEVCONTAINER_ROOT}/devcontainer.json"

temporary_directory="$(mktemp -d)"
trap 'rm -rf "${temporary_directory}"' EXIT
history_file="${temporary_directory}/.zsh_history"
zshrc_file="${temporary_directory}/.zshrc"
printf '%s\n' 'echo preserve-existing-history' >"${history_file}"
printf '%s\n' 'setopt nonomatch' >"${zshrc_file}"

bash "${SEEDER}" "${history_file}" "${zshrc_file}"
cp "${history_file}" "${temporary_directory}/first-history"
cp "${zshrc_file}" "${temporary_directory}/first-zshrc"
bash "${SEEDER}" "${history_file}" "${zshrc_file}"
cmp "${history_file}" "${temporary_directory}/first-history"
cmp "${zshrc_file}" "${temporary_directory}/first-zshrc"

[[ "$(head -n 1 "${history_file}")" == 'echo preserve-existing-history' ]]
[[ "$(tail -n 1 "${history_file}")" == 'make -C ros2_ws build' ]]
[[ "$(grep -Fc '# BEGIN MENTOR PI DEVCONTAINER COMMANDS' \
  "${history_file}")" == 1 ]]
[[ "$(grep -Fc '# BEGIN MENTOR PI SEEDED HISTORY LOADER' \
  "${zshrc_file}")" == 1 ]]
grep -Fxq 'make -C firmware build' "${history_file}"
grep -Fxq 'make -C firmware microros-sdk' "${history_file}"
grep -Fxq 'make -C micro_ros_agent build' "${history_file}"
grep -Fxq 'colcon test-result --verbose' "${history_file}"
if grep -Eq 'flash|install-service|campaign-|passive-check|characterize-board' \
    "${history_file}"; then
  echo "Dev Container history contains native hardware commands" >&2
  exit 1
fi

echo "Dev Container shell contract passed."
