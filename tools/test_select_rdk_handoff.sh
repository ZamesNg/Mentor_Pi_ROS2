#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SELECTOR="${SCRIPT_DIR}/select_rdk_handoff.sh"
readonly TRANSFER="${SCRIPT_DIR}/transfer_rdk_handoff.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "RDK handoff selector test failure: $*" >&2
  exit 1
}

MakeFixture() {
  local name="$1"
  local root="${TEST_ROOT}/${name}"
  mkdir -p "${root}/board-handoff"
  cat >"${root}/RDK-HANDOFF.txt" <<EOF
package_format=rrclite-rdk-handoff-v1
release_id=${name}
build_execution=qemu-emulated
build_host_architecture=amd64
target_architecture=arm64
native_target_validated=0
EOF
  printf '%s\n' fixture >"${root}/board-handoff/payload"
  (cd "${root}" && find RDK-HANDOFF.txt board-handoff -type f -print | \
    LC_ALL=C sort | xargs sha256sum >SHA256SUMS)
}

MakeFixture rdk-arm64-20260810T010203Z
MakeFixture rdk-arm64-20260810T040506Z
selected="$("${SELECTOR}" --latest-under "${TEST_ROOT}")"
[[ "${selected}" == "${TEST_ROOT}/rdk-arm64-20260810T040506Z" ]] || \
  Fail "selector did not choose the newest timestamp"
[[ "$("${SELECTOR}" --verify "${TEST_ROOT}/rdk-arm64-20260810T010203Z")" == \
  "${TEST_ROOT}/rdk-arm64-20260810T010203Z" ]] || \
  Fail "explicit older handoff verification failed"

mkdir -p "${TEST_ROOT}/fake-bin"
cat >"${TEST_ROOT}/fake-bin/ssh" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >"${TRANSFER_SSH_LOG:?}"
EOF
cat >"${TEST_ROOT}/fake-bin/scp" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >"${TRANSFER_SCP_LOG:?}"
EOF
chmod +x "${TEST_ROOT}/fake-bin/ssh" "${TEST_ROOT}/fake-bin/scp"
transfer_output="$(
  TRANSFER_SSH_LOG="${TEST_ROOT}/ssh.log" \
  TRANSFER_SCP_LOG="${TEST_ROOT}/scp.log" \
  PATH="${TEST_ROOT}/fake-bin:${PATH}" \
    "${TRANSFER}" --login operator@rdk-host \
      --handoff "${TEST_ROOT}/rdk-arm64-20260810T010203Z" \
      --remote-directory /srv/mentor-pi/received
)"
[[ "${transfer_output}" == *'Created checksummed RDK handoff archive:'* && \
   "${transfer_output}" == *'Transferred rdk-arm64-20260810T010203Z'* ]] || \
  Fail "compact transfer did not report archive creation and transfer"
(cd "${TEST_ROOT}" && \
  sha256sum --check rdk-arm64-20260810T010203Z.tar.sha256 >/dev/null) || \
  Fail "compact transfer archive checksum does not verify"
grep -Fqx -- '-- operator@rdk-host mkdir -p -- '\''/srv/mentor-pi/received'\''' \
  "${TEST_ROOT}/ssh.log" || Fail "compact transfer passed the wrong SSH command"
grep -Fq -- \
  'operator@rdk-host:/srv/mentor-pi/received/' "${TEST_ROOT}/scp.log" || \
  Fail "compact transfer passed the wrong SCP destination"
reuse_output="$(
  TRANSFER_SSH_LOG="${TEST_ROOT}/ssh.log" \
  TRANSFER_SCP_LOG="${TEST_ROOT}/scp.log" \
  PATH="${TEST_ROOT}/fake-bin:${PATH}" \
    "${TRANSFER}" --login operator@rdk-host \
      --handoff "${TEST_ROOT}/rdk-arm64-20260810T010203Z" \
      --remote-directory /srv/mentor-pi/received
)"
[[ "${reuse_output}" == *'Reusing verified RDK handoff archive:'* ]] || \
  Fail "compact transfer did not reuse its verified archive"
printf '%s\n' refreshed > \
  "${TEST_ROOT}/rdk-arm64-20260810T010203Z/board-handoff/payload"
(cd "${TEST_ROOT}/rdk-arm64-20260810T010203Z" && \
  find RDK-HANDOFF.txt board-handoff -type f -print | LC_ALL=C sort | \
    xargs sha256sum >SHA256SUMS)
refresh_output="$(
  TRANSFER_SSH_LOG="${TEST_ROOT}/ssh.log" \
  TRANSFER_SCP_LOG="${TEST_ROOT}/scp.log" \
  PATH="${TEST_ROOT}/fake-bin:${PATH}" \
    "${TRANSFER}" --login operator@rdk-host \
      --handoff "${TEST_ROOT}/rdk-arm64-20260810T010203Z" \
      --remote-directory /srv/mentor-pi/received
)"
[[ "${refresh_output}" == *'Replacing stale RDK handoff archive:'* ]] || \
  Fail "compact transfer did not replace a stale archive"
tar -xOf "${TEST_ROOT}/rdk-arm64-20260810T010203Z.tar" \
  rdk-arm64-20260810T010203Z/SHA256SUMS | \
  cmp -s - "${TEST_ROOT}/rdk-arm64-20260810T010203Z/SHA256SUMS" || \
  Fail "replacement archive does not match the refreshed bundle"
if "${TRANSFER}" --login '-oProxyCommand=bad@rdk-host' \
    --handoff "${TEST_ROOT}/rdk-arm64-20260810T010203Z" >/dev/null 2>&1; then
  Fail "compact transfer accepted an unsafe SSH login"
fi

printf '%s\n' tampered >>"${TEST_ROOT}/rdk-arm64-20260810T040506Z/board-handoff/payload"
if "${SELECTOR}" --verify \
    "${TEST_ROOT}/rdk-arm64-20260810T040506Z" >/dev/null 2>&1; then
  Fail "selector accepted a tampered handoff"
fi

echo "RDK handoff selection tests passed."
