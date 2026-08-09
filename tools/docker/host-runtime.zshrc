# Read-only-friendly interactive Mentor Pi shell for the Humble runtime image.
export ZSH=/opt/mentor_pi/oh-my-zsh
export ZSH_THEME=robbyrussell
export DISABLE_AUTO_UPDATE=true
export ZSH_DISABLE_COMPFIX=true
export ZSH_CACHE_DIR="${HOME}/.cache/oh-my-zsh"
zstyle ':omz:update' mode disabled

HISTFILE="${HOME}/.zsh_history"
HISTSIZE=5000
SAVEHIST=5000
setopt append_history hist_ignore_dups share_history

plugins=(git sudo)

for _mentor_pi_setup in \
    /opt/ros/humble/setup.zsh \
    /opt/mentor_pi/micro_ros_agent/local_setup.zsh \
    /opt/mentor_pi/host/setup.zsh; do
  [[ -r "${_mentor_pi_setup}" ]] || {
    print -u2 -- "Mentor Pi shell: missing ${_mentor_pi_setup}"
    return 1
  }
done
[[ -r "${ZSH}/oh-my-zsh.sh" ]] || {
  print -u2 -- "Mentor Pi shell: Oh My Zsh is missing."
  return 1
}

mkdir -p -- "${ZSH_CACHE_DIR}"
source "${ZSH}/oh-my-zsh.sh"
source /opt/ros/humble/setup.zsh
source /opt/mentor_pi/micro_ros_agent/local_setup.zsh
source /opt/mentor_pi/host/setup.zsh

[[ -r /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh ]] || {
  print -u2 -- "Mentor Pi shell: zsh autosuggestions are missing."
  return 1
}
source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh

# Syntax highlighting must remain last so it observes every registered widget.
[[ -r /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh ]] || {
  print -u2 -- "Mentor Pi shell: zsh syntax highlighting is missing."
  return 1
}
source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh

PROMPT="(mentor-pi-humble) ${PROMPT}"
unset _mentor_pi_setup
