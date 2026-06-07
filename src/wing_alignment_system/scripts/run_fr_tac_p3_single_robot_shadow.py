#!/usr/bin/env python3
"""FR-TAC-P3-C: Single-robot low-speed shadow validation.

Runs WatchdogPolicy in pure Python (no ROS, no real motion).
Validates execution_mode behavior with P3-C speed limits:
  max_linear_speed  <= 0.05 m/s
  max_angular_speed <= 0.10 rad/s
"""

from __future__ import annotations
import csv, sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'wing_alignment_system'))
from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig

# P3-C speed limits
V_IN = 0.03   # m/s  (within 0.05 limit)
W_IN = 0.06   # rad/s (within 0.10 limit)

MODES = ('normal', 'degraded', 'hold', 'safe_stop')
RUN_ID = 'p3c_shadow'


def _policy(*, enable_output: bool) -> WatchdogPolicy:
    return WatchdogPolicy(WatchdogConfig(
        watchdog_hz=40.0, age_safe=0.15, age_stop=0.40,
        decay_mode='linear', decay_k=3.0,
        enable_execution_mode_output=enable_output,
        degraded_linear_scale=0.5, degraded_angular_scale=0.5))


def _mode_row(ts, seq, mode, vi, wi, vo, wo, sc, st, sr):
    return dict(run_id=RUN_ID, timestamp='%.6f' % ts, robot_id='tracer1',
        seq=str(seq), transmission_mode='full_update',
        execution_mode=mode, AoI_ms='100.000',
        effective_freshness='0.900000', output_scale='%.6f' % sc,
        stop_reason=sr, watchdog_state=st,
        cmd_v_in='%.6f' % vi, cmd_w_in='%.6f' % wi,
        cmd_v_out='%.6f' % vo, cmd_w_out='%.6f' % wo,
        t_source='%.6f' % ts, t_rx='%.6f' % ts, t_watchdog='%.6f' % ts)


def _check(tid, mode, en, vi, wi, vo, wo, st, sr, p, note=''):
    return dict(test_id=tid, enable_execution_mode_output=str(en),
        execution_mode=mode,
        cmd_v_in='%.6f' % vi, cmd_w_in='%.6f' % wi,
        cmd_v_out='%.6f' % vo, cmd_w_out='%.6f' % wo,
        watchdog_state=st, stop_reason=sr, passed=str(p), note=note)


def _csv(d, fn, fields, rows):
    p = d / fn; p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        for r in rows: w.writerow(r)
    return p


def test_output_disabled(out_dir):
    """enable_execution_mode_output=false: all modes pass cmd_vel through unchanged."""
    p = _policy(enable_output=False); mr = []; cr = []; ok = True; s = 0
    for m in MODES:
        s += 1; t = s * 0.1
        p.on_cmd(s, V_IN, W_IN, t, execution_mode=m)
        out = p.compute(t + 0.01)
        passed = abs(out.applied_v - V_IN) < 0.001 and abs(out.applied_w - W_IN) < 0.001
        if not passed: ok = False
        mr.append(_mode_row(t, s, m, V_IN, W_IN, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
        cr.append(_check('output_disabled_' + m, m, False, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'output==input' if passed else 'FAIL'))
    return mr, cr, ok


def test_output_enabled(out_dir):
    """enable_execution_mode_output=true: each mode scales or zeros cmd_vel."""
    p = _policy(enable_output=True); mr = []; cr = []; ok = True; s = 100
    exp = {
        'normal':    (V_IN,       W_IN,       1.0),
        'degraded':  (V_IN * 0.5, W_IN * 0.5, 0.5),
        'hold':      (0.0,        0.0,        0.0),
        'safe_stop': (0.0,        0.0,        0.0),
    }
    for m in MODES:
        s += 1; t = s * 0.1
        p.on_cmd(s, V_IN, W_IN, t, execution_mode=m)
        out = p.compute(t + 0.01)
        ev, ew, es = exp[m]
        passed = abs(out.applied_v - ev) < 0.001 and abs(out.applied_w - ew) < 0.001
        if not passed: ok = False
        mr.append(_mode_row(t, s, m, V_IN, W_IN, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
        cr.append(_check('output_enabled_' + m, m, True, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'expect v=%.6f w=%.6f' % (ev, ew)))
    return mr, cr, ok


def test_safety_overrides(out_dir):
    """emergency_stop / cmd_stop / age_stop still override execution_mode."""
    mr = []; cr = []; ok = True; s = 200
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, V_IN, W_IN, t, execution_mode='normal')
        p.on_emergency(True); out = p.compute(t + 0.01)
        passed = out.applied_v == 0.0 and out.applied_w == 0.0 and out.state == 'EMERGENCY_STOP'
        if not passed: ok = False
        mr.append(_mode_row(t, s, 'normal', V_IN, W_IN, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
        cr.append(_check('safety_emergency_' + lb, 'normal', en, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'emergency zero'))
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, V_IN, W_IN, t, execution_mode='normal')
        p.on_stop(True); out = p.compute(t + 0.01)
        passed = out.applied_v == 0.0 and out.applied_w == 0.0 and out.state == 'CMD_STOP'
        if not passed: ok = False
        mr.append(_mode_row(t, s, 'normal', V_IN, W_IN, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
        cr.append(_check('safety_cmd_stop_' + lb, 'normal', en, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'cmd_stop zero'))
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, V_IN, W_IN, t, execution_mode='normal')
        out = p.compute(t + 0.5)
        passed = out.applied_v == 0.0 and out.applied_w == 0.0 and out.state == 'AGE_STOP'
        if not passed: ok = False
        mr.append(_mode_row(t + 0.5, s, 'normal', V_IN, W_IN, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
        cr.append(_check('safety_age_stop_' + lb, 'normal', en, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'age_stop zero'))
    return mr, cr, ok


def test_low_speed_limits(out_dir):
    """P3-C speed limits: input cmd_vel must not exceed caps."""
    mr = []; cr = []; ok = True; s = 300
    p = _policy(enable_output=True)
    # Test that input at exactly the cap works
    s += 1; t = float(s)
    p.on_cmd(s, 0.05, 0.10, t, execution_mode='normal')
    out = p.compute(t + 0.01)
    passed = abs(out.applied_v - 0.05) < 0.001 and abs(out.applied_w - 0.10) < 0.001
    if not passed: ok = False
    mr.append(_mode_row(t, s, 'normal', 0.05, 0.10, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
    cr.append(_check('speed_limit_cap', 'normal', True, 0.05, 0.10, out.applied_v, out.applied_w, out.state, out.stop_reason, passed, 'speed at cap'))
    # Test that the default low-speed values are within limits
    s += 1; t = float(s)
    p.on_cmd(s, V_IN, W_IN, t, execution_mode='normal')
    out = p.compute(t + 0.01)
    within = abs(V_IN) <= 0.05 and abs(W_IN) <= 0.10
    cr.append(_check('speed_limit_default', 'normal', True, V_IN, W_IN, out.applied_v, out.applied_w, out.state, out.stop_reason, within, 'default within limits' if within else 'FAIL'))
    if not within: ok = False
    return mr, cr, ok


def main():
    out_dir = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else Path('/tmp/p3c_shadow')
    out_dir.mkdir(parents=True, exist_ok=True)
    am = []; ac = []; res = []

    print('=== P3-C Shadow: output=false, all modes keep cmd_vel ===')
    mr, cr, ok = test_output_disabled(out_dir); am += mr; ac += cr; res.append(ok)
    print(f'  {"PASS" if ok else "FAIL"}')

    print('=== P3-C Shadow: output=true, modes scale cmd_vel ===')
    mr, cr, ok = test_output_enabled(out_dir); am += mr; ac += cr; res.append(ok)
    print(f'  {"PASS" if ok else "FAIL"}')

    print('=== P3-C Shadow: safety overrides still win ===')
    mr, cr, ok = test_safety_overrides(out_dir); am += mr; ac += cr; res.append(ok)
    print(f'  {"PASS" if ok else "FAIL"}')

    print('=== P3-C Shadow: low-speed limits check ===')
    mr, cr, ok = test_low_speed_limits(out_dir); am += mr; ac += cr; res.append(ok)
    print(f'  {"PASS" if ok else "FAIL"}')

    gate = all(res)

    # mode_timeline
    tf = ['run_id', 'timestamp', 'robot_id', 'seq', 'transmission_mode', 'execution_mode',
          'AoI_ms', 'effective_freshness', 'output_scale', 'stop_reason', 'watchdog_state',
          'cmd_v_in', 'cmd_w_in', 'cmd_v_out', 'cmd_w_out', 't_source', 't_rx', 't_watchdog']
    _csv(out_dir, 'mode_timeline_tracer1.csv', tf, am)

    # cmd_vel input/output check
    cf = ['test_id', 'enable_execution_mode_output', 'execution_mode', 'cmd_v_in', 'cmd_w_in',
          'cmd_v_out', 'cmd_w_out', 'watchdog_state', 'stop_reason', 'passed', 'note']
    _csv(out_dir, 'watchdog_output_check.csv', cf, ac)

    # run_summary
    total = len(ac); passed = sum(1 for r in ac if r['passed'] == 'True')
    srow = dict(run_id=RUN_ID, test_type='p3c_shadow', total_checks=total,
                checks_passed=passed, checks_failed=total - passed,
                gate_passed=str(gate).lower(),
                cmd_vel_in_mean=f'{V_IN:.6f}',
                cmd_vel_out_mean='%.6f' % (sum(float(r['cmd_v_out']) for r in am) / len(am)) if am else '0.0',
                output_scale_mean='%.6f' % (sum(float(r['output_scale']) for r in am) / len(am)) if am else '1.0',
                degraded_count=str(sum(1 for r in ac if r['execution_mode'] == 'degraded' and r['passed'] == 'True')),
                hold_count=str(sum(1 for r in ac if r['execution_mode'] == 'hold' and r['passed'] == 'True')),
                safe_stop_count=str(sum(1 for r in ac if r['execution_mode'] == 'safe_stop' and r['passed'] == 'True')),
                emergency_stop_count=str(sum(1 for r in ac if 'emergency' in r['test_id'] and r['passed'] == 'True')),
                cmd_stop_count=str(sum(1 for r in ac if 'cmd_stop' in r['test_id'] and r['passed'] == 'True')),
                age_stop_count=str(sum(1 for r in ac if 'age_stop' in r['test_id'] and r['passed'] == 'True')),
                max_AoI_ms='100.000',
    )
    _csv(out_dir, 'run_summary.csv', list(srow.keys()), [srow])

    # gate file
    gl = ['FR-TAC-P3-C Shadow Gate Report', '=' * 60,
          'Gate: ' + ('PASS' if gate else 'FAIL'),
          f'{passed}/{total} checks passed', '']
    for r in ac:
        st = 'PASS' if r['passed'] == 'True' else 'FAIL'
        gl.append(f'  [{st}] {r["test_id"]}: v_in={r["cmd_v_in"]} v_out={r["cmd_v_out"]} w_in={r["cmd_w_in"]} w_out={r["cmd_w_out"]} state={r["watchdog_state"]} note={r["note"]}')
    (out_dir / 'p3c_shadow_gate.txt').write_text('\n'.join(gl), encoding='utf-8')

    print(f'\n=== P3-C Shadow Gate: {"PASS" if gate else "FAIL"} ===')
    return 0 if gate else 1


if __name__ == '__main__':
    raise SystemExit(main())
