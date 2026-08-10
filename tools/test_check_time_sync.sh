#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "time-sync test failure: $*" >&2
  exit 1
}

mkdir -p "${TEST_ROOT}/bin"
cat >"${TEST_ROOT}/bin/timedatectl" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "${TEST_NTP_STATE:-no}"
EOF
cat >"${TEST_ROOT}/bin/chronyc" <<'EOF'
#!/usr/bin/env bash
printf 'Leap status     : %s\n' "${TEST_CHRONY_STATE:-Not synchronised}"
EOF
chmod +x "${TEST_ROOT}/bin/timedatectl"
chmod +x "${TEST_ROOT}/bin/chronyc"

PATH="${TEST_ROOT}/bin:/usr/bin:/bin" TEST_NTP_STATE=yes \
  "${SCRIPT_DIR}/check_time_sync.sh" >/dev/null || \
  Fail "synchronized systemd state was rejected"
PATH="${TEST_ROOT}/bin:/usr/bin:/bin" TEST_NTP_STATE=no \
  TEST_CHRONY_STATE=Normal "${SCRIPT_DIR}/check_time_sync.sh" >/dev/null || \
  Fail "synchronized chrony fallback was rejected"
if PATH="${TEST_ROOT}/bin:/usr/bin:/bin" TEST_NTP_STATE=no \
    TEST_CHRONY_STATE='Not synchronised' \
    "${SCRIPT_DIR}/check_time_sync.sh" >/dev/null 2>&1; then
  Fail "unsynchronized systemd state was accepted"
fi

echo "Time-synchronization preflight tests passed."
