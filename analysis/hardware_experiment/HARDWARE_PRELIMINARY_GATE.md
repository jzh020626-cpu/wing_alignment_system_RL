# Hardware Preliminary Gate

## Gate Intent
- H0 is recorder and host-monitor startup validation only.
- H0 does not prove external sensor availability.
- H0 does not prove nominal task execution.
- H4 and H5 are readiness gates only. They are not permission to run severe degradation tests.
- Patch 2B adds opt-in recorder launch wiring. Host clock and network monitors remain external scripts and are not started by ROS launch.

## H0
Required:
- manifest exists
- `evidence_class=hardware_preliminary`
- required manifest schema keys exist
- `manifest_sha256` matches canonical manifest JSON
- `clock_sync_status.csv` exists, has a header, and has at least one data row
- `network_ping_samples.csv` exists, has a header, and has at least one data row
- `interface_counters.csv` exists, has a header, and has at least one data row
- `recorder_health.csv` exists, has a header, and has at least one data row
- `recorder_topic_status.csv` exists and has a header
- `run_id` is consistent where rows exist
- timestamps are monotonic where rows exist
- recorder endpoint policy fields are present if `recorder_health.csv` has rows

Warnings only in H0:
- missing QR rows
- missing delta rows
- missing force rows
- missing slide rows
- missing mocap rows
- missing mission/watchdog/scheduler rows

H0 failures:
- required schema key missing
- required startup artifact missing
- `run_id` mismatch
- timestamp non-monotonic
- recorder endpoint policy fields missing

Recommended standalone Patch 2A H0 dry-run:
```bash
export ROS_DOMAIN_ID=36
export RUN_ID="hwprelim_h0_$(date -u +%Y%m%dT%H%M%SZ)"
export HOST_ID="$(hostname)"
export IFACE="wlan0"
export HW_ROOT="$HOME/.ros/hardware_preliminary/$RUN_ID"

python3 src/wing_alignment_system/scripts/create_bench_run_manifest.py --out-dir "$HW_ROOT/manifest" --profile nominal --baseline-mode current_safe_default --evidence-class hardware_preliminary --run-id "$RUN_ID"
bash src/wing_alignment_system/scripts/hardware_clock_sync_monitor.sh --run-id "$RUN_ID" --host-id "$HOST_ID" --host-role control_host --out-dir "$HW_ROOT/manifest/hardware_capture/$RUN_ID" --once
bash src/wing_alignment_system/scripts/hardware_network_monitor.sh --run-id "$RUN_ID" --host-id "$HOST_ID" --iface "$IFACE" --ping-target 192.168.1.1 --peer-role gateway --out-dir "$HW_ROOT/manifest/hardware_capture/$RUN_ID" --duration-sec 5
ros2 run wing_alignment_system passive_measurement_recorder --run-id "$RUN_ID" --out-dir "$HW_ROOT/manifest/hardware_capture/$RUN_ID" --config-file src/wing_alignment_system/config/mission_params.yaml --robots tracer1,tracer2,tracer3 --slides huatai1,huatai2,huatai3 --duration-sec 10
python3 src/wing_alignment_system/scripts/validate_bench_run_artifacts.py --manifest "$HW_ROOT/manifest/run_manifest.json" --gate H0 --out-dir "$HW_ROOT/validation"
```

Opt-in integrated run with `system_bringup.launch.py`:
```bash
ros2 launch wing_alignment_system system_bringup.launch.py \
  config_file:=src/wing_alignment_system/config/mission_params.yaml \
  start_passive_recorder:=true \
  measurement_log_dir:="$HW_ROOT/manifest/hardware_capture/$RUN_ID" \
  measurement_run_id:="$RUN_ID" \
  measurement_robots:=tracer1,tracer2,tracer3 \
  measurement_slides:=huatai1,huatai2,huatai3
```

Notes:
- Host monitors still must be started manually with `hardware_clock_sync_monitor.sh` and `hardware_network_monitor.sh`.
- Leaving `measurement_run_id` empty is allowed by launch, but real-machine runs should pass it explicitly for artifact consistency.
- Offline derivation remains a separate post-run step:
```bash
python3 src/wing_alignment_system/scripts/derive_hardware_preliminary_artifacts.py \
  --manifest "$HW_ROOT/manifest/run_manifest.json" \
  --out-dir "$HW_ROOT/manifest/hardware_derived/$RUN_ID"
```

## H4
H4 fails unless:
- `prereq_gate_results.H3.status == passed`
- every `operator_safety_precheck` field is present and true
- at least one robot/control endpoint peer exists, unless a waiver is present with both `reason` and `approved_by`

H4 hard stops:
- emergency-stop or safety-supervisor channel would be degraded
- gateway-only monitoring is used without a waiver
- packet loss exceeds the operator threshold before task motion
- one-way delay is required while clock sync is unavailable or unverified

## H5
H5 fails unless:
- `prereq_gate_results.H3.status == passed`
- every `operator_safety_precheck` field is present and true

H5 hard stops:
- host-load testing perturbs safety-supervisor timing
- watchdog heartbeat is missed before task motion
- recorder drop counts exceed the allowed threshold
- emergency-stop latency cannot be observed from available local artifacts

## Timing And Interpretation Rules
- True one-way delay is unavailable without verified clock sync and valid upstream source stamps.
- If a stream uses `stamp_origin=local_receive_fallback`, that stream may not claim one-way delay.
- Bandwidth or throughput is network context only.
- `recorder_callback_timing.csv` is passive-recorder callback proxy only.

## Operator Safety Precheck Fields
- `operator_precheck_ack`
- `safety_observer_present`
- `emergency_stop_tested_pre_run`
- `workspace_clear`
- `degradation_scope_confirmed`
- `safety_channels_excluded_from_degradation`
