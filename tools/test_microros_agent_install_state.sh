#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly VALIDATOR="${SCRIPT_DIR}/verify_microros_agent_install_state.sh"
readonly IDLE_GUARD="${SCRIPT_DIR}/require_microros_agent_install_idle.sh"
readonly INSTALLER="${SCRIPT_DIR}/install_microros_agent.sh"
readonly SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-agent-state.XXXXXX")"

Cleanup() {
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "micro-ROS Agent install-state test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

readonly FAKE_SYSTEMCTL="${TEST_ROOT}/systemctl"
cat >"${FAKE_SYSTEMCTL}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ "$#" == "4" ]]
[[ "$1" == "show" ]]
[[ "$2" == "--property=LoadState" ]]
[[ "$3" == "--property=ActiveState" ]]
[[ "$4" == "mentor-pi-controller.target" ]]
case "${MENTOR_PI_TEST_TARGET_STATE:-inactive}" in
  inactive)
    printf '%s\n' 'LoadState=loaded' 'ActiveState=inactive'
    ;;
  not-found)
    printf '%s\n' 'LoadState=not-found' 'ActiveState=inactive'
    ;;
  active | activating | reloading | deactivating)
    printf '%s\n' 'LoadState=loaded' \
      "ActiveState=${MENTOR_PI_TEST_TARGET_STATE}"
    ;;
  error)
    exit 5
    ;;
  malformed)
    printf '%s\n' 'LoadState=loaded'
    ;;
  *)
    printf '%s\n' 'LoadState=masked' 'ActiveState=inactive'
    ;;
esac
EOF
chmod +x "${FAKE_SYSTEMCTL}"

MENTOR_PI_TEST_TARGET_STATE=inactive \
  "${IDLE_GUARD}" --systemctl "${FAKE_SYSTEMCTL}"
MENTOR_PI_TEST_TARGET_STATE=not-found \
  "${IDLE_GUARD}" --systemctl "${FAKE_SYSTEMCTL}"
for unsafe_state in active activating reloading deactivating error malformed \
    unexpected; do
  ExpectFailure env MENTOR_PI_TEST_TARGET_STATE="${unsafe_state}" \
    "${IDLE_GUARD}" --systemctl "${FAKE_SYSTEMCTL}"
done
ExpectFailure "${IDLE_GUARD}" --systemctl "${TEST_ROOT}/missing-systemctl"

WriteOsRelease() {
  local destination="$1"
  local identity="$2"
  local version="$3"
  printf 'ID=%s\nVERSION_ID="%s"\n' "${identity}" "${version}" \
    >"${destination}"
}

MakeRepository() {
  local destination="$1"
  local origin="$2"
  git init -q "${destination}"
  git -C "${destination}" config user.name "Mentor Pi test"
  git -C "${destination}" config user.email "test@mentor-pi.invalid"
  printf '%s\n' "pinned source" >"${destination}/source.txt"
  git -C "${destination}" add source.txt
  git -C "${destination}" commit -q -m "pinned source"
  git -C "${destination}" remote add origin "${origin}"
  git -C "${destination}" checkout -q --detach HEAD
}

readonly OS_RELEASE="${TEST_ROOT}/os-release"
readonly REPOSITORY="${TEST_ROOT}/repository"
readonly EXPECTED_ORIGIN="https://github.com/example/pinned.git"
WriteOsRelease "${OS_RELEASE}" ubuntu 24.04
MakeRepository "${REPOSITORY}" "${EXPECTED_ORIGIN}"
readonly EXPECTED_COMMIT="$(git -C "${REPOSITORY}" rev-parse HEAD)"

"${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64 \
  --repository "${REPOSITORY}" --origin "${EXPECTED_ORIGIN}" \
  --commit "${EXPECTED_COMMIT}" >/dev/null
"${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture arm64 >/dev/null

WriteOsRelease "${OS_RELEASE}" debian 24.04
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64
WriteOsRelease "${OS_RELEASE}" ubuntu 22.04
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64
WriteOsRelease "${OS_RELEASE}" ubuntu 24.04
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture riscv64

printf '%s\n' 'modified source' >"${REPOSITORY}/source.txt"
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" --commit "${EXPECTED_COMMIT}"
git -C "${REPOSITORY}" checkout -q -- source.txt

printf '%s\n' 'untracked source' >"${REPOSITORY}/untracked.txt"
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" --commit "${EXPECTED_COMMIT}"
rm "${REPOSITORY}/untracked.txt"

git -C "${REPOSITORY}" remote set-url origin \
  https://github.com/example/wrong.git
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" --commit "${EXPECTED_COMMIT}"
git -C "${REPOSITORY}" remote set-url origin "${EXPECTED_ORIGIN}"

git -C "${REPOSITORY}" switch -q -c attached
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" --commit "${EXPECTED_COMMIT}"
git -C "${REPOSITORY}" checkout -q --detach "${EXPECTED_COMMIT}"

ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" \
  --commit 0000000000000000000000000000000000000000
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" \
  --architecture amd64 --repository "${REPOSITORY}" \
  --origin "${EXPECTED_ORIGIN}" --commit not-a-commit

grep -Fq 'verify_microros_agent_install_state.sh' "${INSTALLER}" ||
  Fail "the production installer does not invoke the state validator"
grep -Fq 'microros_agent_source.lock' "${INSTALLER}" ||
  Fail "the production installer does not consume the Agent source lock"
[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] ||
  Fail "the Agent source lock is missing or symbolic"
grep -Fqx 'format=mentor-pi-micro-ros-agent-source-lock-v1' \
  "${SOURCE_LOCK}" || Fail "the Agent source lock format is wrong"
[[ "$(grep -Ec '^(agent_commit|messages_commit)=[0-9a-f]{40}$' \
    "${SOURCE_LOCK}")" == "2" ]] ||
  Fail "the Agent source lock does not contain two pinned commits"
grep -Fq -- '--os-release /etc/os-release' "${INSTALLER}" ||
  Fail "the production installer does not verify the deployment OS"
[[ "$(grep -Fc -- '--repository' "${INSTALLER}")" == "2" ]] ||
  Fail "the production installer does not verify both pinned repositories"

readonly GUARD_LINE="$(grep -n -F '"${INSTALL_IDLE_GUARD}"' "${INSTALLER}" |
  tail -n 1 | cut -d: -f1)"
readonly FIRST_MUTATION_LINE="$(grep -n -F 'apt-get update' "${INSTALLER}" |
  cut -d: -f1)"
[[ -n "${GUARD_LINE}" && -n "${FIRST_MUTATION_LINE}" &&
  "${GUARD_LINE}" -lt "${FIRST_MUTATION_LINE}" ]] ||
  Fail "the production installer does not run the idle guard before mutation"

bash -n "${VALIDATOR}"
bash -n "${IDLE_GUARD}"
bash -n "${INSTALLER}"
echo "micro-ROS Agent install-state tests passed"
