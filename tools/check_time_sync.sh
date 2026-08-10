#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Clock synchronization check failed: $*" >&2
  exit 1
}

systemd_state="unavailable"
if command -v timedatectl >/dev/null 2>&1; then
  synchronized="$(timedatectl show --property=NTPSynchronized --value \
    2>/dev/null || true)"
  systemd_state="${synchronized:-unknown}"
  if [[ "${synchronized}" == yes ]]; then
    echo "Host clock synchronization is active (systemd)."
    exit 0
  fi
fi

if command -v chronyc >/dev/null 2>&1; then
  tracking="$(chronyc tracking 2>/dev/null || true)"
  grep -Eq '^Leap status[[:space:]]*:[[:space:]]*Normal$' <<<"${tracking}" || \
    Fail "chrony does not report normal synchronized time"
  echo "Host clock synchronization is active (chrony)."
  exit 0
fi

Fail "host time is not synchronized (systemd=${systemd_state}, chrony unavailable or non-normal)"
