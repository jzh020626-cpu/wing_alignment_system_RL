#!/usr/bin/env python3
"""FR-TAC-P3-A: Non-zero cmd_vel shadow evidence."""

from __future__ import annotations
import csv, sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'wing_alignment_system'))
from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig

MODES = ('normal', 'degraded', 'hold', 'safe_stop')
INPUT_V = 0.20
INPUT_W = 0.40
RUN_ID = 'p3a_nonzero_shadow'

def _policy(*, enable_output):
    return WatchdogPolicy(WatchdogConfig(
        watchdog_hz=40.0, age_safe=0.15, age_stop=0.40,
        decay_mode='linear', decay_k=3.0,
        enable_execution_mode_output=enable_output,
        degraded_linear_scale=0.5, degraded_angular_scale=0.25))

def _row(ts, seq, mode, vi, wi, vo, wo, sc, st, sr):
    return dict(run_id=RUN_ID, timestamp='%.6f'%ts, robot_id='tracer1',
        seq=str(seq), transmission_mode='full_update',
        execution_mode=mode, AoI_ms='100.000',
        effective_freshness='0.900000', output_scale='%.6f'%sc,
        stop_reason=sr, watchdog_state=st,
        cmd_v_in='%.6f'%vi, cmd_w_in='%.6f'%wi,
        cmd_v_out='%.6f'%vo, cmd_w_out='%.6f'%wo,
        t_source='%.6f'%ts, t_rx='%.6f'%ts, t_watchdog='%.6f'%ts)

def _check(tid, mode, en, vi, wi, vo, wo, st, sr, p, note=''):
    return dict(test_id=tid, enable_execution_mode_output=str(en),
        execution_mode=mode,
        cmd_v_in='%.6f'%vi, cmd_w_in='%.6f'%wi,
        cmd_v_out='%.6f'%vo, cmd_w_out='%.6f'%wo,
        watchdog_state=st, stop_reason=sr, passed=str(p), note=note)

def _csv(d, fn, fields, rows):
    p = d / fn; p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        for r in rows: w.writerow(r)
    return p

def _status(ok):
    return 'PASS' if ok else 'FAIL'

def test_output_disabled(o):
    p = _policy(enable_output=False); mr = []; cr = []; ok = True; s = 0
    for m in MODES:
        s += 1; t = s * 0.1
        p.on_cmd(s, INPUT_V, INPUT_W, t, execution_mode=m)
        out = p.compute(t + 0.01)
        passed = abs(out.applied_v-INPUT_V)<0.001 and abs(out.applied_w-INPUT_W)<0.001
        if not passed: ok = False
        mr.append(_row(t,s,m,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.output_scale,out.state,out.stop_reason))
        cr.append(_check('output_disabled_'+m,m,False,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.state,out.stop_reason,passed,'output==input' if passed else 'FAIL'))
    return mr, cr, ok

def test_output_enabled(o):
    p = _policy(enable_output=True); mr = []; cr = []; ok = True; s = 100
    exp = {'normal':(INPUT_V,INPUT_W,1.0),'degraded':(INPUT_V*0.5,INPUT_W*0.25,0.25),'hold':(0.0,0.0,0.0),'safe_stop':(0.0,0.0,0.0)}
    for m in MODES:
        s += 1; t = s * 0.1
        p.on_cmd(s, INPUT_V, INPUT_W, t, execution_mode=m)
        out = p.compute(t + 0.01)
        ev, ew, es = exp[m]
        passed = abs(out.applied_v-ev)<0.001 and abs(out.applied_w-ew)<0.001
        if not passed: ok = False
        mr.append(_row(t,s,m,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.output_scale,out.state,out.stop_reason))
        cr.append(_check('output_enabled_'+m,m,True,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.state,out.stop_reason,passed,'expect v=%.6f w=%.6f'%(ev,ew)))
    return mr, cr, ok

def test_safety_overrides(o):
    mr = []; cr = []; ok = True; s = 200
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, INPUT_V, INPUT_W, t, execution_mode='normal')
        p.on_emergency(True); out = p.compute(t + 0.01)
        passed = out.applied_v==0.0 and out.applied_w==0.0 and out.state=='EMERGENCY_STOP'
        if not passed: ok = False
        mr.append(_row(t,s,'normal',INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.output_scale,out.state,out.stop_reason))
        cr.append(_check('safety_emergency_'+lb,'normal',en,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.state,out.stop_reason,passed,'emergency zero'))
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, INPUT_V, INPUT_W, t, execution_mode='normal')
        p.on_stop(True); out = p.compute(t + 0.01)
        passed = out.applied_v==0.0 and out.applied_w==0.0 and out.state=='CMD_STOP'
        if not passed: ok = False
        mr.append(_row(t,s,'normal',INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.output_scale,out.state,out.stop_reason))
        cr.append(_check('safety_cmd_stop_'+lb,'normal',en,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.state,out.stop_reason,passed,'cmd_stop zero'))
    for en in (False, True):
        lb = 'enabled' if en else 'disabled'
        p = _policy(enable_output=en); s += 1; t = float(s)
        p.on_cmd(s, INPUT_V, INPUT_W, t, execution_mode='normal')
        out = p.compute(t + 0.5)
        passed = out.applied_v==0.0 and out.applied_w==0.0 and out.state=='AGE_STOP'
        if not passed: ok = False
        mr.append(_row(t+0.5,s,'normal',INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.output_scale,out.state,out.stop_reason))
        cr.append(_check('safety_age_stop_'+lb,'normal',en,INPUT_V,INPUT_W,out.applied_v,out.applied_w,out.state,out.stop_reason,passed,'age_stop zero'))
    return mr, cr, ok

def main():
    out_dir = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) > 1 else Path('/tmp/p3a_nonzero_shadow')
    out_dir.mkdir(parents=True, exist_ok=True)
    am = []; ac = []; res = []

    print('=== P3-A-1: output=false, all modes keep cmd_vel ===')
    mr, cr, ok = test_output_disabled(out_dir); am += mr; ac += cr; res.append(ok)
    print('  ' + _status(ok))

    print('=== P3-A-2: output=true, modes scale cmd_vel ===')
    mr, cr, ok = test_output_enabled(out_dir); am += mr; ac += cr; res.append(ok)
    print('  ' + _status(ok))

    print('=== P3-A-3: safety overrides still win ===')
    mr, cr, ok = test_safety_overrides(out_dir); am += mr; ac += cr; res.append(ok)
    print('  ' + _status(ok))

    gate = all(res)
    tf = ['run_id','timestamp','robot_id','seq','transmission_mode','execution_mode','AoI_ms','effective_freshness','output_scale','stop_reason','watchdog_state','cmd_v_in','cmd_w_in','cmd_v_out','cmd_w_out','t_source','t_rx','t_watchdog']
    _csv(out_dir, 'mode_timeline_tracer1.csv', tf, am)
    cf = ['test_id','enable_execution_mode_output','execution_mode','cmd_v_in','cmd_w_in','cmd_v_out','cmd_w_out','watchdog_state','stop_reason','passed','note']
    _csv(out_dir, 'watchdog_output_check.csv', cf, ac)

    total = len(ac); passed = sum(1 for r in ac if r['passed'] == 'True')
    srow = dict(run_id=RUN_ID, test_type='p3a_nonzero_shadow', total_checks=total, checks_passed=passed, checks_failed=total-passed, gate_passed=str(gate).lower())
    _csv(out_dir, 'run_summary.csv', list(srow.keys()), [srow])

    gl = ['FR-TAC-P3-A Gate Report', '='*60, 'Gate: '+('PASS' if gate else 'FAIL'), '%d/%d checks passed'%(passed,total), '']
    for r in ac:
        status = 'PASS' if r['passed']=='True' else 'FAIL'
        gl.append('  [%s] %s: v_in=%s v_out=%s w_in=%s w_out=%s state=%s note=%s'%(status,r['test_id'],r['cmd_v_in'],r['cmd_v_out'],r['cmd_w_in'],r['cmd_w_out'],r['watchdog_state'],r['note']))
    (out_dir/'p3a_gate.txt').write_text(chr(10).join(gl), encoding='utf-8')

    print('')
    print('=== P3-A Gate: '+('PASS' if gate else 'FAIL')+' ===')
    return 0 if gate else 1

if __name__ == '__main__':
    raise SystemExit(main())
