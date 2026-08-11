#!/usr/bin/env bash

set -euo pipefail

history_file="${1:-${HISTFILE:-${HOME}/.zsh_history}}"
zshrc_file="${2:-${ZDOTDIR:-${HOME}}/.zshrc}"
history_dir="$(dirname "${history_file}")"
zshrc_dir="$(dirname "${zshrc_file}")"
start_marker="# BEGIN MENTOR PI DEVCONTAINER COMMANDS"
end_marker="# END MENTOR PI DEVCONTAINER COMMANDS"
loader_start_marker="# BEGIN MENTOR PI SEEDED HISTORY LOADER"
loader_end_marker="# END MENTOR PI SEEDED HISTORY LOADER"
default_command="make -C ros2_ws build"

mkdir -p "${history_dir}" "${zshrc_dir}"
touch "${history_file}" "${zshrc_file}"

temporary_file="$(mktemp "${history_file}.tmp.XXXXXX")"
trap 'rm -f "${temporary_file}"' EXIT

awk \
  -v start_marker="${start_marker}" \
  -v end_marker="${end_marker}" \
  -v default_command="${default_command}" '
    $0 == start_marker { in_managed_block = 1; next }
    $0 == end_marker { in_managed_block = 0; next }
    in_managed_block { next }
    $0 == default_command { next }
    { print }
  ' "${history_file}" >"${temporary_file}"

{
  printf '%s\n' "${start_marker}"
  printf '%s\n' \
    "make help" \
    "make doctor" \
    "make check-compatibility" \
    "make -C firmware doctor" \
    "make -C firmware setup" \
    "make -C firmware test" \
    "make -C firmware build" \
    "make -C firmware verify" \
    "make -C firmware package" \
    "make -C firmware microros-sdk" \
    "make -C micro_ros_agent doctor" \
    "make -C micro_ros_agent setup" \
    "make -C micro_ros_agent build" \
    "make -C micro_ros_agent test" \
    "make -C ros2_ws doctor" \
    "make -C ros2_ws deps" \
    "make -C ros2_ws test" \
    "cd ros2_ws" \
    "colcon list" \
    "colcon build" \
    "colcon test" \
    "colcon test-result --verbose"
  printf '%s\n' "${end_marker}"
  printf '%s\n' "${default_command}"
} >>"${temporary_file}"

mv "${temporary_file}" "${history_file}"
trap - EXIT

if ! grep --fixed-strings --line-regexp --quiet \
    "${loader_start_marker}" "${zshrc_file}"; then
  {
    printf '\n%s\n' "${loader_start_marker}"
    printf '%s\n' \
      'if [[ -r "${HISTFILE:-${HOME}/.zsh_history}" ]]; then' \
      '  builtin fc -R "${HISTFILE:-${HOME}/.zsh_history}"' \
      'fi'
    printf '%s\n' "${loader_end_marker}"
  } >>"${zshrc_file}"
fi
