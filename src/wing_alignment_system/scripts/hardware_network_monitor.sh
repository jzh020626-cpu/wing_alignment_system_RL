#!/bin/bash
set -u

RUN_ID=""
HOST_ID=""
IFACE=""
OUT_DIR=""
DURATION_SEC="0"
THROUGHPUT_PROBE=""
PING_TARGETS=()
PEER_ROLES=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --run-id) RUN_ID="$2"; shift 2 ;;
    --host-id) HOST_ID="$2"; shift 2 ;;
    --iface) IFACE="$2"; shift 2 ;;
    --ping-target) PING_TARGETS+=("$2"); shift 2 ;;
    --peer-role) PEER_ROLES+=("$2"); shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --duration-sec) DURATION_SEC="$2"; shift 2 ;;
    --throughput-probe) THROUGHPUT_PROBE="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$RUN_ID" ] || [ -z "$HOST_ID" ] || [ -z "$IFACE" ] || [ -z "$OUT_DIR" ] || [ -z "$DURATION_SEC" ]; then
  echo "missing required args" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
PING_CSV="$OUT_DIR/network_ping_samples.csv"
IFACE_CSV="$OUT_DIR/interface_counters.csv"
printf "run_id,host_id,peer_host,peer_role,iface,packet_loss_percent,rtt_ms,rtt_min_ms,rtt_avg_ms,rtt_max_ms,jitter_ms,rtt_mdev_ms,status,note,t_wall
" > "$PING_CSV"
printf "run_id,host_id,iface,rx_bytes,tx_bytes,rx_dropped,tx_dropped,rx_errors,tx_errors,wifi_rssi_dbm,wifi_link_quality,t_wall
" > "$IFACE_CSV"
if [ -n "$THROUGHPUT_PROBE" ]; then
  printf "run_id,host_id,phase,status,note,t_wall
" > "$OUT_DIR/throughput_probe.csv"
  printf "%s,%s,%s,%s,%s,%s
" "$RUN_ID" "$HOST_ID" "$THROUGHPUT_PROBE" "disabled" "throughput probe is context-only and off by default during Patch 2A" "$(date +%s.%N)" >> "$OUT_DIR/throughput_probe.csv"
fi

if [ "${#PING_TARGETS[@]}" -eq 0 ]; then
  PING_TARGETS=("127.0.0.1")
fi

peer_role_for_index() {
  local idx="$1"
  if [ "$idx" -lt "${#PEER_ROLES[@]}" ]; then
    printf "%s" "${PEER_ROLES[$idx]}"
  else
    printf "unknown"
  fi
}

is_numeric() {
  printf "%s" "$1" | grep -Eq '^-?[0-9]+([.][0-9]+)?$'
}

blank_if_not_numeric() {
  local value="$1"
  if is_numeric "$value"; then
    printf "%s" "$value"
  else
    printf ""
  fi
}

extract_packet_loss() {
  printf "%s
" "$1" | sed -nE 's/.* ([0-9]+([.][0-9]+)?)% packet loss.*/\1/p' | head -n 1
}

extract_time_sample() {
  printf "%s
" "$1" | sed -nE 's/.*time=([-0-9.]+) ?ms.*/\1/p' | head -n 1
}

extract_rtt_stats() {
  local output="$1"
  local rtt_line=""
  local stats=""
  rtt_line="$(printf "%s
" "$output" | grep -E '^(rtt|round-trip) ' | head -n 1 || true)"
  if [ -z "$rtt_line" ]; then
    printf "|||"
    return
  fi
  stats="${rtt_line#*= }"
  stats="${stats% ms*}"
  IFS='/' read -r rtt_min rtt_avg rtt_max rtt_mdev <<EOF
$stats
EOF
  printf "%s|%s|%s|%s" "$rtt_min" "$rtt_avg" "$rtt_max" "$rtt_mdev"
}

last_nonempty_line() {
  printf "%s
" "$1" | awk 'NF { line=$0 } END { print line }'
}

normalize_note() {
  printf "%s" "$1" | tr '
' ' ' | tr ',' ';' | sed 's/[[:space:]]\+/ /g; s/^ //; s/ $//'
}

write_ping_row() {
  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s
"     "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}" "${12}" "${13}" "${14}" "${15}" >> "$PING_CSV"
}

write_iface_row() {
  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s
"     "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}" "${12}" >> "$IFACE_CSV"
}

sample_ping() {
  local peer_host="$1"
  local peer_role="$2"
  local output=""
  local ping_rc=0
  local packet_loss_percent=""
  local rtt_ms=""
  local rtt_min_ms=""
  local rtt_avg_ms=""
  local rtt_max_ms=""
  local rtt_mdev_ms=""
  local jitter_ms=""
  local status="ping_unavailable"
  local note="ping command not found"
  local stats=""

  if command -v ping >/dev/null 2>&1; then
    output="$(ping -n -c 1 -W 1 "$peer_host" 2>&1)"
    ping_rc=$?
    packet_loss_percent="$(extract_packet_loss "$output")"
    rtt_ms="$(extract_time_sample "$output")"
    stats="$(extract_rtt_stats "$output")"
    IFS='|' read -r rtt_min_ms rtt_avg_ms rtt_max_ms rtt_mdev_ms <<EOF
$stats
EOF

    packet_loss_percent="$(blank_if_not_numeric "$packet_loss_percent")"
    rtt_ms="$(blank_if_not_numeric "$rtt_ms")"
    rtt_min_ms="$(blank_if_not_numeric "$rtt_min_ms")"
    rtt_avg_ms="$(blank_if_not_numeric "$rtt_avg_ms")"
    rtt_max_ms="$(blank_if_not_numeric "$rtt_max_ms")"
    rtt_mdev_ms="$(blank_if_not_numeric "$rtt_mdev_ms")"
    jitter_ms="$rtt_mdev_ms"
    if [ -z "$rtt_ms" ] && [ -n "$rtt_avg_ms" ]; then
      rtt_ms="$rtt_avg_ms"
    fi

    if [ "$ping_rc" -eq 0 ]; then
      if [ -n "$packet_loss_percent" ] || [ -n "$rtt_ms" ] || [ -n "$rtt_avg_ms" ]; then
        status="ok"
        note=""
      else
        status="parse_error"
        note="unrecognized ping output"
      fi
    elif [ -n "$packet_loss_percent" ] && awk "BEGIN { exit !($packet_loss_percent >= 100.0) }"; then
      status="packet_loss"
      note="100% packet loss"
    else
      status="ping_failed"
      note="$(last_nonempty_line "$output")"
      if [ -z "$note" ]; then
        note="ping exited with code $ping_rc"
      fi
    fi
  fi

  note="$(normalize_note "$note")"
  write_ping_row "$RUN_ID" "$HOST_ID" "$peer_host" "$peer_role" "$IFACE" "$packet_loss_percent" "$rtt_ms" "$rtt_min_ms" "$rtt_avg_ms" "$rtt_max_ms" "$jitter_ms" "$rtt_mdev_ms" "$status" "$note" "$(date +%s.%N)"
}

sample_iface() {
  local dev_line=""
  local rx_bytes=""
  local tx_bytes=""
  local rx_dropped=""
  local tx_dropped=""
  local rx_errors=""
  local tx_errors=""
  local wifi_rssi_dbm=""
  local wifi_link_quality=""

  if [ -r /proc/net/dev ]; then
    dev_line="$(awk -F'[: ]+' -v iface="$IFACE" '$2 == iface {print $3" "$4" "$5" "$6" "$11" "$12" "$13" "$14; exit}' /proc/net/dev)"
    if [ -n "$dev_line" ]; then
      rx_bytes="$(printf "%s
" "$dev_line" | awk '{print $1}')"
      rx_errors="$(printf "%s
" "$dev_line" | awk '{print $2}')"
      rx_dropped="$(printf "%s
" "$dev_line" | awk '{print $3}')"
      tx_bytes="$(printf "%s
" "$dev_line" | awk '{print $5}')"
      tx_errors="$(printf "%s
" "$dev_line" | awk '{print $6}')"
      tx_dropped="$(printf "%s
" "$dev_line" | awk '{print $7}')"
    fi
  fi

  if command -v iw >/dev/null 2>&1; then
    local iw_output
    iw_output="$(iw dev "$IFACE" link 2>/dev/null || true)"
    wifi_rssi_dbm="$(printf "%s
" "$iw_output" | awk '/signal:/ {print $2; exit}')"
    wifi_link_quality="$(printf "%s
" "$iw_output" | awk -F': ' '/tx bitrate:/ {print $2; exit}')"
  fi

  write_iface_row "$RUN_ID" "$HOST_ID" "$IFACE" "$rx_bytes" "$tx_bytes" "$rx_dropped" "$tx_dropped" "$rx_errors" "$tx_errors" "$wifi_rssi_dbm" "$wifi_link_quality" "$(date +%s.%N)"
}

loop_count="$DURATION_SEC"
if [ "$loop_count" -lt 1 ] 2>/dev/null; then
  loop_count=1
fi

i=0
while [ "$i" -lt "$loop_count" ]; do
  idx=0
  for peer_host in "${PING_TARGETS[@]}"; do
    sample_ping "$peer_host" "$(peer_role_for_index "$idx")"
    idx=$((idx + 1))
  done
  sample_iface
  i=$((i + 1))
  if [ "$i" -lt "$loop_count" ]; then
    sleep 1
  fi
done
