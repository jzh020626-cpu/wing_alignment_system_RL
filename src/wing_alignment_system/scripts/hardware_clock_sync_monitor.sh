#!/bin/bash
set -u

RUN_ID=""
HOST_ID=""
HOST_ROLE=""
OUT_DIR=""
MODE="once"
DURATION_SEC="0"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --run-id) RUN_ID="$2"; shift 2 ;;
    --host-id) HOST_ID="$2"; shift 2 ;;
    --host-role) HOST_ROLE="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --once) MODE="once"; shift 1 ;;
    --duration-sec) MODE="duration"; DURATION_SEC="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$RUN_ID" ] || [ -z "$HOST_ID" ] || [ -z "$HOST_ROLE" ] || [ -z "$OUT_DIR" ]; then
  echo "missing required args" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
CSV="$OUT_DIR/clock_sync_status.csv"
printf "run_id,host_id,host_role,monitor_source,sync_available,sync_verified,offset_ms,reference_clock,time_base,one_way_delay_allowed,t_wall\n" > "$CSV"

write_row() {
  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
    "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}" >> "$CSV"
}

now_wall() {
  date +%s.%N
}

probe_clock_sync() {
  local monitor_source="unavailable"
  local sync_available="false"
  local sync_verified="false"
  local offset_ms=""
  local reference_clock=""
  local time_base="system_wall"
  local one_way_delay_allowed="false"
  local output=""

  if command -v chronyc >/dev/null 2>&1; then
    output="$(chronyc tracking 2>/dev/null || true)"
    if [ -n "$output" ]; then
      monitor_source="chronyc"
      sync_available="true"
      reference_clock="$(printf "%s\n" "$output" | awk -F: '/Reference ID/ {gsub(/^[ \t]+/, "", $2); print $2; exit}')"
      offset_ms="$(printf "%s\n" "$output" | awk '/Last offset/ {print $4 * 1000.0; exit}')"
      if printf "%s\n" "$output" | grep -qi "Leap status.*Normal"; then
        sync_verified="true"
      fi
    fi
  fi

  if [ "$sync_available" != "true" ] && command -v pmc >/dev/null 2>&1; then
    output="$(pmc -u -b 0 "GET TIME_STATUS_NP" 2>/dev/null || true)"
    if [ -n "$output" ]; then
      monitor_source="pmc"
      sync_available="true"
      reference_clock="ptp"
      offset_ms="$(printf "%s\n" "$output" | awk '/master_offset/ {print $2 / 1000000.0; exit}')"
      if printf "%s\n" "$output" | grep -q "master_offset"; then
        sync_verified="true"
        time_base="device_time"
      fi
    fi
  fi

  if [ "$sync_available" != "true" ] && command -v timedatectl >/dev/null 2>&1; then
    output="$(timedatectl show 2>/dev/null || true)"
    if [ -n "$output" ]; then
      monitor_source="timedatectl"
      sync_available="true"
      reference_clock="ntp"
      if printf "%s\n" "$output" | grep -q '^SystemClockSynchronized=yes'; then
        sync_verified="true"
      fi
    fi
  fi

  if [ "$sync_available" != "true" ] && command -v ntpq >/dev/null 2>&1; then
    output="$(ntpq -pn 2>/dev/null || true)"
    if [ -n "$output" ]; then
      monitor_source="ntpq"
      sync_available="true"
      reference_clock="$(printf "%s\n" "$output" | awk '/^\*/ {print $1; exit}' | sed 's/^\*//')"
      offset_ms="$(printf "%s\n" "$output" | awk '/^\*/ {print $9; exit}')"
      if [ -n "$reference_clock" ]; then
        sync_verified="true"
      fi
    fi
  fi

  if [ "$sync_verified" = "true" ]; then
    one_way_delay_allowed="true"
  fi

  write_row \
    "$RUN_ID" \
    "$HOST_ID" \
    "$HOST_ROLE" \
    "$monitor_source" \
    "$sync_available" \
    "$sync_verified" \
    "$offset_ms" \
    "$reference_clock" \
    "$time_base" \
    "$one_way_delay_allowed" \
    "$(now_wall)"
}

if [ "$MODE" = "once" ]; then
  probe_clock_sync
  exit 0
fi

end_epoch=$(( $(date +%s) + DURATION_SEC ))
while [ "$(date +%s)" -lt "$end_epoch" ]; do
  probe_clock_sync
  sleep 1
done
probe_clock_sync
