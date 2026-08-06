#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly VALIDATOR="${SCRIPT_DIR}/verify_microros_agent_install_state.sh"
readonly IDLE_GUARD="${SCRIPT_DIR}/require_microros_agent_install_idle.sh"
readonly INSTALLER="${SCRIPT_DIR}/install_microros_agent.sh"
readonly BUILD_HELPER="${SCRIPT_DIR}/build_microros_agent_from_lock.sh"
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

ExpectFailureContaining() {
  local expected="$1"
  shift
  local output
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] ||
    Fail "failure did not contain '${expected}': ${output}"
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
readonly HUMBLE_SETUP="${TEST_ROOT}/humble-setup.bash"
readonly WRONG_ROS_SETUP="${TEST_ROOT}/wrong-ros-setup.bash"
WriteOsRelease "${OS_RELEASE}" ubuntu 22.04
MakeRepository "${REPOSITORY}" "${EXPECTED_ORIGIN}"
readonly EXPECTED_COMMIT="$(git -C "${REPOSITORY}" rev-parse HEAD)"
cat >"${HUMBLE_SETUP}" <<'EOF'
: "${AMENT_TRACE_SETUP_FILES:=}"
export ROS_DISTRO=humble
EOF
cat >"${WRONG_ROS_SETUP}" <<'EOF'
: "${AMENT_TRACE_SETUP_FILES:=}"
export ROS_DISTRO=jazzy
EOF

VerifyRosSetupFixture() {
  local setup="$1"
  (
    set +u
    source "${setup}"
    set -u
    [[ "${ROS_DISTRO:-}" == "humble" ]]
  )
}
VerifyRosSetupFixture "${HUMBLE_SETUP}"
ExpectFailure VerifyRosSetupFixture "${WRONG_ROS_SETUP}"

"${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64 \
  --repository "${REPOSITORY}" --origin "${EXPECTED_ORIGIN}" \
  --commit "${EXPECTED_COMMIT}" >/dev/null
"${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture arm64 >/dev/null

WriteOsRelease "${OS_RELEASE}" debian 22.04
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64
WriteOsRelease "${OS_RELEASE}" ubuntu 24.04
ExpectFailure "${VALIDATOR}" --os-release "${OS_RELEASE}" --architecture amd64
WriteOsRelease "${OS_RELEASE}" ubuntu 22.04
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

grep -Fq 'build_microros_agent_from_lock.sh' "${INSTALLER}" ||
  Fail "the production installer does not invoke the shared build helper"
grep -Fq 'verify_microros_agent_install_state.sh' "${BUILD_HELPER}" ||
  Fail "the shared build helper does not invoke the state validator"
grep -Fq 'microros_agent_source.lock' "${INSTALLER}" ||
  Fail "the production installer does not consume the Agent source lock"
[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] ||
  Fail "the Agent source lock is missing or symbolic"
grep -Fqx 'format=mentor-pi-micro-ros-agent-source-lock-v2' \
  "${SOURCE_LOCK}" || Fail "the Agent source lock format is wrong"
grep -Fqx 'ros_distro=humble' "${SOURCE_LOCK}" ||
  Fail "the Agent source lock does not bind ROS 2 Humble"
grep -Fqx \
  'agent_repository=https://github.com/micro-ROS/micro-ROS-Agent.git' \
  "${SOURCE_LOCK}" || Fail "the Agent repository allowlist entry is wrong"
grep -Fqx \
  'messages_repository=https://github.com/micro-ROS/micro_ros_msgs.git' \
  "${SOURCE_LOCK}" || Fail "the message repository allowlist entry is wrong"
grep -Fqx \
  'xrce_agent_repository=https://github.com/eProsima/Micro-XRCE-DDS-Agent.git' \
  "${SOURCE_LOCK}" || Fail "the XRCE Agent repository allowlist entry is wrong"
[[ "$(grep -Ec '^(agent_commit|messages_commit|xrce_agent_commit)=[0-9a-f]{40}$' \
    "${SOURCE_LOCK}")" == "3" ]] ||
  Fail "the Agent source lock does not contain three pinned commits"

# Exercise the consumers against a copied lock so removing either pre-network
# origin check cannot be hidden by merely retaining the canonical lock row.
readonly LOCK_CONSUMER_FIXTURE="${TEST_ROOT}/lock-consumer"
readonly FIXTURE_LOCK="${LOCK_CONSUMER_FIXTURE}/microros_agent_source.lock"
mkdir -p "${LOCK_CONSUMER_FIXTURE}"
cp "${BUILD_HELPER}" "${INSTALLER}" "${VALIDATOR}" \
  "${LOCK_CONSUMER_FIXTURE}/"
chmod +x "${LOCK_CONSUMER_FIXTURE}/build_microros_agent_from_lock.sh" \
  "${LOCK_CONSUMER_FIXTURE}/install_microros_agent.sh" \
  "${LOCK_CONSUMER_FIXTURE}/verify_microros_agent_install_state.sh"
sed \
  's#^xrce_agent_repository=.*#xrce_agent_repository=https://example.invalid/untrusted.git#' \
  "${SOURCE_LOCK}" >"${FIXTURE_LOCK}"
ExpectFailureContaining 'unexpected repository' env ROS_DISTRO=humble \
  "${LOCK_CONSUMER_FIXTURE}/build_microros_agent_from_lock.sh" fetch \
  --work-root "${LOCK_CONSUMER_FIXTURE}/work"
ExpectFailureContaining 'unexpected XRCE Agent repository' \
  "${LOCK_CONSUMER_FIXTURE}/install_microros_agent.sh"

cp "${SOURCE_LOCK}" "${FIXTURE_LOCK}"
printf '%s\n' \
  'xrce_agent_repository=https://github.com/eProsima/Micro-XRCE-DDS-Agent.git' \
  >>"${FIXTURE_LOCK}"
ExpectFailureContaining 'exactly one xrce_agent_repository=' \
  env ROS_DISTRO=humble \
  "${LOCK_CONSUMER_FIXTURE}/build_microros_agent_from_lock.sh" fetch \
  --work-root "${LOCK_CONSUMER_FIXTURE}/work"

sed 's/^xrce_agent_commit=.*/xrce_agent_commit=not-a-commit/' \
  "${SOURCE_LOCK}" >"${FIXTURE_LOCK}"
ExpectFailureContaining 'malformed commit' env ROS_DISTRO=humble \
  "${LOCK_CONSUMER_FIXTURE}/build_microros_agent_from_lock.sh" fetch \
  --work-root "${LOCK_CONSUMER_FIXTURE}/work"
grep -Fq -- '--os-release /etc/os-release' "${INSTALLER}" ||
  Fail "the production installer does not verify the deployment OS"
[[ "$(grep -Fc -- '--repository' "${BUILD_HELPER}")" == "3" ]] ||
  Fail "the shared build helper does not verify all pinned repositories"
grep -Fq '"${AGENT_BUILD_HELPER}" fetch --work-root "${BUILD_ROOT}"' \
  "${INSTALLER}" || Fail "the installer bypasses the shared source fetch"
grep -Fq '"${AGENT_BUILD_HELPER}" build --work-root "${BUILD_ROOT}"' \
  "${INSTALLER}" || Fail "the installer bypasses the shared Agent build"

readonly GUARD_LINE="$(grep -n -F '"${INSTALL_IDLE_GUARD}"' "${INSTALLER}" |
  tail -n 1 | cut -d: -f1)"
readonly ROS_SOURCE_LINE="$(grep -n -F 'source "${ROS_SETUP}"' "${INSTALLER}" |
  cut -d: -f1)"
readonly ROS_IDENTITY_LINE="$(grep -n -F \
  '[[ "${ROS_DISTRO:-}" == "humble" ]]' "${INSTALLER}" | cut -d: -f1)"
readonly FIRST_MUTATION_LINE="$(grep -n -F 'apt-get update' "${INSTALLER}" |
  cut -d: -f1)"
[[ -n "${GUARD_LINE}" && -n "${ROS_SOURCE_LINE}" &&
  -n "${ROS_IDENTITY_LINE}" && -n "${FIRST_MUTATION_LINE}" &&
  "${GUARD_LINE}" -lt "${ROS_SOURCE_LINE}" &&
  "${ROS_SOURCE_LINE}" -lt "${ROS_IDENTITY_LINE}" &&
  "${ROS_IDENTITY_LINE}" -lt "${FIRST_MUTATION_LINE}" ]] ||
  Fail "the installer does not verify Humble after its idle guard and before mutation"
[[ "$(grep -Fc 'source "${ROS_SETUP}"' "${INSTALLER}")" == "1" ]] ||
  Fail "the installer must source the ROS setup exactly once"

bash -n "${VALIDATOR}"
bash -n "${IDLE_GUARD}"
bash -n "${INSTALLER}"
bash -n "${BUILD_HELPER}"
echo "micro-ROS Agent install-state tests passed"
