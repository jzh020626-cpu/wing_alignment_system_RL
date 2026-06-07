# Instrumentation Plan

Patch 2A scope: passive recorder implementation, host monitor scripts, and static/runtime-free regression coverage. No launch change, no derived-artifact generation, and no controller-path change is implemented here.

Schema-freeze notes:
- `manifest_sha256` is frozen as canonical JSON hash without the `manifest_sha256` field itself.
- New manifests emit structured `prereq_gate_results` objects for `H0` through `H3`.
- Validator remains backward-compatible with older string-only prereq entries.

## Field Table
| field name | artifact file | source topic or source script | label | availability | control-path risk | implementation note |
| --- | --- | --- | --- | --- | --- | --- |
| `run_id` | manifest and all CSVs | manifest generator and future recorder/monitor scripts | configured | already available | none | validator now checks consistency where rows exist |
| `manifest_sha256` | `run_manifest.json` | `create_bench_run_manifest.py` | configured | already available | none | computed from canonical manifest JSON without the hash field |
| `host_role` | `clock_sync_status.csv` | future clock monitor script | configured | needs external monitor | none | required for clock-sync auditability |
| `sync_verified` | `clock_sync_status.csv` | future clock monitor script | configured | needs external monitor | none | gates true one-way delay claims |
| `reference_clock` | `clock_sync_status.csv` | future clock monitor script | configured | needs external monitor | none | required in Patch 1 schema |
| `peer_role` | `network_ping_samples.csv` | future network monitor script | configured | needs external monitor | none | `gateway` is context only; H4 needs robot/control peer or waiver |
| `user_defined_publishers_count` | `recorder_health.csv` | future passive recorder | configured | needs passive recorder | none | must remain `0` |
| `control_publishers_count` | `recorder_health.csv` | future passive recorder | configured | needs passive recorder | none | must remain `0` |
| `service_clients_count` | `recorder_health.csv` | future passive recorder | configured | needs passive recorder | none | must remain `0` |
| `user_defined_services_count` | `recorder_health.csv` | future passive recorder | configured | needs passive recorder | none | must remain `0` |
| `ros_infrastructure_endpoints` | `recorder_health.csv` | future passive recorder | configured | needs passive recorder | none | informational only unless control-affecting |
| `msg_source_stamp` | raw capture CSVs | future passive recorder | feedback | needs passive recorder | none | required when the upstream message has a usable stamp |
| `source_stamp_valid` | raw capture CSVs | future passive recorder | configured | needs passive recorder | none | must be false for local-receive fallback |
| `stamp_origin` | raw capture CSVs | future passive recorder | configured | needs passive recorder | none | one of `upstream_header`, `local_receive_fallback`, `none` |
| `source_time_base` | raw capture CSVs | future passive recorder | configured | needs passive recorder | none | one of `ros_time`, `system_wall`, `device_time`, `unknown` |
| `frame_id` | raw pose/delta capture CSVs | future passive recorder | feedback | needs passive recorder | none | capture only if present upstream |
| `one_way_delay_allowed` | clock and stamped capture CSVs | future recorder/monitor path | configured | needs passive recorder | none | validator blocks one-way-delay claims unless policy is satisfied |
| `recorder_callback_timing` | `recorder_callback_timing.csv` | future passive recorder | proxy | needs passive recorder | none | passive-recorder callback proxy only, not controller timing |
| `authority_proxy_*` | `authority_proxy_timeseries.csv` | future derivation script | proxy | unavailable | none | explicit controller alpha does not exist today |
| `command_residence_*` | `command_residence_events.csv` | future derivation script | proxy | unavailable | none | must include join quality and join key type |
| `terminal_residual_proxy` | `terminal_residual_proxy.csv` | future derivation script | proxy | unavailable | none | remains proxy, not measured residual |

## Patch 1 Validator Coverage
- H0 validates only startup artifacts and manifest policy.
- H4 and H5 additionally validate machine-readable prereqs and operator safety precheck fields.
- Missing external sensor streams during H0 are warnings only.
- H0 failures include missing required schema keys, missing startup artifacts, `run_id` mismatches, non-monotonic timestamps, missing recorder endpoint policy fields, and manifest hash mismatch.

## Patch 2C
- `scripts/derive_hardware_preliminary_artifacts.py` now generates:
  - `command_residence_events.csv`
  - `control_loop_timing.csv`
  - `executor_backlog_proxy.csv`
  - `authority_proxy_timeseries.csv`
  - `terminal_residual_proxy.csv`
  - `phase_attributed_scheduler_events.csv`
  - `derivation_report.json`
- Every derived artifact remains bounded by proxy or estimated labels. None of these outputs are measured physical truth.
- Missing mission, watchdog, or capture inputs produce partial or empty-header outputs plus `derivation_report.json` availability notes.

## Remaining After Patch 2C
- Real H0/H1/H2/H3 hardware preliminary runs
- Derived artifact review on real-machine capture
