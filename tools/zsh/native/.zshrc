# Mentor Pi wrapper for `make shell` on native Ubuntu 22.04. It loads the
# project overlays, then the user's existing zsh configuration without editing
# that file.
[[ -r /opt/ros/humble/setup.zsh ]] || {
  print -u2 -- "Mentor Pi shell: ROS 2 Humble zsh setup is missing."
  return 1
}
[[ -r "${MENTOR_PI_AGENT_PREFIX}/local_setup.zsh" ]] || {
  print -u2 -- "Mentor Pi shell: Agent zsh setup is missing."
  return 1
}
[[ -r "${MENTOR_PI_HOST_PREFIX}/setup.zsh" ]] || {
  print -u2 -- "Mentor Pi shell: host zsh setup is missing."
  return 1
}

source /opt/ros/humble/setup.zsh
source "${MENTOR_PI_AGENT_PREFIX}/local_setup.zsh"
source "${MENTOR_PI_HOST_PREFIX}/setup.zsh"

_mentor_pi_wrapper_zdotdir="${ZDOTDIR}"
_mentor_pi_user_zdotdir="${MENTOR_PI_USER_ZDOTDIR:-${HOME}}"
export ZDOTDIR="${_mentor_pi_user_zdotdir}"
if [[ -r "${_mentor_pi_user_zdotdir}/.zshrc" && \
    "${_mentor_pi_user_zdotdir}/.zshrc" != \
      "${_mentor_pi_wrapper_zdotdir}/.zshrc" ]]; then
  source "${_mentor_pi_user_zdotdir}/.zshrc"
fi
unset _mentor_pi_wrapper_zdotdir _mentor_pi_user_zdotdir

PROMPT="(mentor-pi-humble) ${PROMPT}"
