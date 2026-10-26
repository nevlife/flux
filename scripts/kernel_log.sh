#!/usr/bin/env bash
# Judges a GPU test run by the kernel log. A use after unregister or a write through a read-only
# import can fault in the IOMMU or the GPU and leave no CUDA error, so new fault lines fail the run:
# NVRM Xid, Intel DMAR and AMD-Vi faults, SMMU context faults and events, nvgpu MMU faults.
#
#   kernel_log.sh mark          prints a mark for the log as it is now
#   kernel_log.sh check MARK    prints the fault lines logged after MARK, exit 1 if any
#   kernel_log.sh run CMD...    runs CMD between the two, failing if either fails
#
# The journal needs group adm or systemd-journal, dmesg needs kernel.dmesg_restrict=0. Without
# either, mark and check exit 3 and run says the run was not judged.
set -uo pipefail

faults='NVRM: Xid|DMAR: \[DMA (Read|Write)|DMAR: DRHD: handling fault|AMD-Vi: Event logged \[IO_PAGE_FAULT|smmu.*([Ff]ault|event 0x)|nvgpu.*[Mm][Mm][Uu] ?[Ff]ault'

mark() {
  if [ -n "$(journalctl -k -q -n 1 -o cat 2>/dev/null)" ]; then
    echo "j:$(journalctl -k -q -n 0 --show-cursor 2>/dev/null | sed -n 's/^-- cursor: //p')"
  elif dmesg >/dev/null 2>&1; then
    echo "d:$(dmesg | wc -l)"
  else
    return 3
  fi
}

since() {
  case "$1" in
    j:*) journalctl -k -q --no-pager -o short-monotonic --after-cursor="${1#j:}" ;;
    d:*) dmesg | tail -n "+$((${1#d:} + 1))" ;;
    *) echo "kernel_log.sh: bad mark '$1'" >&2; return 2 ;;
  esac
}

check() {
  local new
  # journald copies kernel messages in on its own schedule.
  sleep 1
  new=$(since "$1") || return 3
  new=$(grep -E "$faults" <<<"$new")
  [ -z "$new" ] && return 0
  printf 'kernel faults logged during the run:\n%s\n' "$new" >&2
  return 1
}

case "${1:-}" in
  mark) mark ;;
  check) check "${2:?mark}" ;;
  run)
    shift
    if ! at=$(mark); then
      echo "kernel_log.sh: the kernel log is not readable without root; this run is not judged" >&2
      exec "$@"
    fi
    "$@"
    rc=$?
    check "$at" || rc=1
    exit "$rc"
    ;;
  *) echo "usage: kernel_log.sh mark | check MARK | run CMD..." >&2; exit 2 ;;
esac
