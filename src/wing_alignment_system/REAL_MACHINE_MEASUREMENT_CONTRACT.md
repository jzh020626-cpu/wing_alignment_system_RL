# Real-Machine Measurement Contract

## Scope
This contract covers `evidence_class=hardware_preliminary` only. It defines real-machine data readiness, naming, and validation boundaries for ROS2 hardware runs without changing controller behavior.

Assumptions:
- `ROS_DOMAIN_ID` is provided by the runtime environment and is not set by this repo.
- QR, mocap, force, and slide-status producers are external unless they are observed at runtime.
- H0 is startup validation only. H0 does not prove sensor availability or task execution.

## Schema Freeze
- Patch 1.5 freezes the manifest and validator schema for `hardware_preliminary`.
- `manifest_sha256` is computed from canonical JSON with sorted keys.
- `manifest_sha256` itself is excluded from the hash input.
- Validator and tests must be able to recompute the hash and match it exactly.
- `prereq_gate_results` is structured in new manifests and backward-compatible in the validator for older string-only manifests.

## Evidence Boundary
- `hardware_preliminary` is bounded evidence, not full hardware validation.
- Do not report true one-way delay unless clock synchronization is verified and the upstream source stamp is valid.
- If clock sync is not verified, only report receive-side age, age-at-use, inter-arrival, RTT, jitter, loss, and callback-delay proxies.
- Bandwidth or throughput is network context only. It is not freshness evidence.
- Recorder callback timing is passive-recorder proxy only. It is not controller callback timing.

## Artifact Classes
`capture_artifacts`
- `recorder_health.csv`
- `recorder_topic_status.csv`
- `raw_qr_samples.csv`
- `delta_samples.csv`
- `force_samples.csv`
- `slide_status_samples.csv`
- `safety_events.csv`
- `chassis_command_samples.csv`
- `slide_command_samples.csv`
- `recorder_callback_timing.csv`
- `clock_sync_status.csv`
- `network_ping_samples.csv`
- `interface_counters.csv`

`derived_artifacts`
- `command_residence_events.csv`
- `control_loop_timing.csv`
- `executor_backlog_proxy.csv`
- `authority_proxy_timeseries.csv`
- `terminal_residual_proxy.csv`
- `phase_attributed_scheduler_events.csv`
- `derivation_report.json`

Patch 1 only defines manifest and validator schema for these artifacts. It does not implement the recorder, host monitors, or derivation scripts.
Patch 2A implements:
- a passive read-only recorder node,
- host-side clock sync monitor script,
- host-side network monitor script.
Patch 2A does not integrate them into launch and does not implement derived artifacts.
Patch 2C implements:
- `scripts/derive_hardware_preliminary_artifacts.py`
- bounded offline proxy derivation only
- partial-output behavior when mission, watchdog, or capture inputs are missing

## Field Labels
Use one of:
- `measured`: direct sensor-backed quantity with a real sensor.
- `feedback`: device or middleware feedback stream without a direct physical claim.
- `proxy`: derived timing or authority quantity, or any residual/support/stress estimate without a dedicated sensor.
- `estimated`: model-based or simulation-calibrated quantity.
- `configured`: declared configuration, policy, topic, QoS, peer, or boundary metadata.

## Clock Sync Policy
`clock_sync_status.csv` must carry:
- `run_id`
- `host_id`
- `host_role`
- `monitor_source`
- `sync_available`
- `sync_verified`
- `offset_ms`
- `reference_clock`
- `time_base`
- `one_way_delay_allowed`
- `t_wall`

Strict rule:
- True one-way delay is allowed only if `sync_verified=true`, `source_stamp_valid=true`, and `stamp_origin=upstream_header`.
- If `stamp_origin=local_receive_fallback`, then `source_stamp_valid` must be false and one-way delay must be disallowed.

## MOCAP And External Producers
- Mocap is `configured` until runtime-observed.
- Default configured topics are:
  - `mocap_wing_topic=/Rigid8/pose`
  - `mocap_robot_topics=/Rigid17/pose,/Rigid14/pose,/Rigid15/pose`
  - `mocap_message_type=geometry_msgs/msg/PoseStamped`
  - `mocap_frame_id_policy=configured_until_runtime_observed`
- QR raw pose, mocap pose, force, and slide-status streams remain external producers unless runtime capture proves otherwise.

## H0 Contract
H0 validates only:
- manifest presence and schema
- canonical `manifest_sha256` match
- recorder startup health artifact presence
- host monitor artifact presence
- CSV headers
- `run_id` propagation where rows exist
- timestamp monotonicity where rows exist
- recorder endpoint policy fields

H0 hard failures include:
- required schema key missing
- required startup artifact missing
- `run_id` mismatch
- non-monotonic timestamps
- missing recorder endpoint policy fields

H0 does not require:
- QR rows
- delta rows
- force rows
- slide rows
- mocap rows
- mission rows
- watchdog rows
- scheduler rows

Missing non-startup capture rows in H0 are warnings, not failures.

## Derived Artifact Boundary
- `command_residence_events.csv` fields are proxy only. They are not true one-way delay or actuator-residence truth.
- `phase_attributed_scheduler_events.csv` is estimated offline attribution. It does not modify scheduler source logs.
- `control_loop_timing.csv` is inter-arrival timing proxy only, not controller internal loop truth.
- `executor_backlog_proxy.csv` is passive-recorder and watchdog proxy only, not internal executor queue-depth truth.
- `authority_proxy_timeseries.csv` contains internal-state proxy fields and command-magnitude proxy ratios only. It is not physical authority allocation or true alpha.
- `terminal_residual_proxy.csv` is a proxy summary only. It is not physical docking error or measured support residual truth.

## Patch 2C Usage
```bash
python3 src/wing_alignment_system/scripts/derive_hardware_preliminary_artifacts.py \
  --manifest "$HW_ROOT/manifest/run_manifest.json" \
  --out-dir "$HW_ROOT/manifest/hardware_derived/$RUN_ID"
```
