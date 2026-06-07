from __future__ import annotations

from freshness_real_robot_logger.freshness_metrics import compute_aoi_and_effective_freshness


def test_effective_freshness_decreases_as_aoi_increases():
    low = compute_aoi_and_effective_freshness(
        sender_timestamp_ns=1_000_000_000,
        receiver_timestamp_ns=1_100_000_000,
        phase="dispatch",
        default_tau_ms=1000.0,
        phase_tau_ms={"dispatch": 1000.0},
    )
    high = compute_aoi_and_effective_freshness(
        sender_timestamp_ns=1_000_000_000,
        receiver_timestamp_ns=1_900_000_000,
        phase="dispatch",
        default_tau_ms=1000.0,
        phase_tau_ms={"dispatch": 1000.0},
    )

    assert low["AoI_ms"] == 100.0
    assert high["AoI_ms"] == 900.0
    assert low["Effective_Freshness"] > high["Effective_Freshness"]


def test_phase_specific_tau_changes_effective_freshness():
    dispatch = compute_aoi_and_effective_freshness(
        sender_timestamp_ns=1_000_000_000,
        receiver_timestamp_ns=1_500_000_000,
        phase="dispatch",
        default_tau_ms=1000.0,
        phase_tau_ms={"dispatch": 1000.0, "final_alignment": 300.0},
    )
    final_alignment = compute_aoi_and_effective_freshness(
        sender_timestamp_ns=1_000_000_000,
        receiver_timestamp_ns=1_500_000_000,
        phase="final_alignment",
        default_tau_ms=1000.0,
        phase_tau_ms={"dispatch": 1000.0, "final_alignment": 300.0},
    )

    assert dispatch["tau_ms"] == 1000.0
    assert final_alignment["tau_ms"] == 300.0
    assert final_alignment["Effective_Freshness"] < dispatch["Effective_Freshness"]


def test_unknown_phase_uses_default_tau():
    result = compute_aoi_and_effective_freshness(
        sender_timestamp_ns=1_000_000_000,
        receiver_timestamp_ns=1_500_000_000,
        phase="unknown_phase",
        default_tau_ms=777.0,
        phase_tau_ms={"dispatch": 1000.0},
    )

    assert result["tau_ms"] == 777.0
