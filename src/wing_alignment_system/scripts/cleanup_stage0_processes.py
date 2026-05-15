#!/usr/bin/env python3
"""Dry-run by default cleanup helper for Stage 0 ROS2 smoke leftovers."""

from __future__ import annotations

import argparse
import getpass
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass


STAGE0_LAUNCH_FILES = (
    "run_all.launch.py",
    "mission_bringup.launch.py",
    "system_bringup.launch.py",
)
STAGE0_NODE_EXECUTABLES = (
    "mission_coordinator",
    "goto_pose_driver",
    "cmd_scheduler",
    "cmd_watchdog",
    "force_monitor",
    "qr_delta_publisher",
)
STAGE0_PACKAGE_MARKERS = (
    "/wing_alignment_system/lib/wing_alignment_system/",
    "/wing_alignment_sensing/lib/wing_alignment_sensing/",
)


@dataclass
class MatchedProcess:
    pid: int
    user: str
    command: str
    reason: str


def _ps_rows() -> list[tuple[int, str, str]]:
    proc = subprocess.run(
        ["ps", "-eo", "pid=,user=,args="],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    rows: list[tuple[int, str, str]] = []
    for line in proc.stdout.splitlines():
        parts = line.strip().split(maxsplit=2)
        if len(parts) < 3:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        rows.append((pid, parts[1], parts[2]))
    return rows


def _match_stage0(command: str) -> str:
    if "cleanup_stage0_processes.py" in command:
        return ""
    if "ros2 launch" in command and "wing_alignment_system" in command:
        for launch_file in STAGE0_LAUNCH_FILES:
            if launch_file in command:
                return f"launch:{launch_file}"
    for marker in STAGE0_PACKAGE_MARKERS:
        if marker not in command:
            continue
        for executable in STAGE0_NODE_EXECUTABLES:
            if f"{marker}{executable}" in command:
                return f"node:{executable}"
    return ""


def find_processes(pattern_set: str, include_other_users: bool = False) -> list[MatchedProcess]:
    if pattern_set != "stage0":
        raise ValueError(f"unsupported pattern-set: {pattern_set}")
    current_user = getpass.getuser()
    current_pid = os.getpid()
    parent_pid = os.getppid()
    matches: list[MatchedProcess] = []
    for pid, user, command in _ps_rows():
        if pid in (current_pid, parent_pid):
            continue
        if (not include_other_users) and user != current_user:
            continue
        reason = _match_stage0(command)
        if reason:
            matches.append(MatchedProcess(pid=pid, user=user, command=command, reason=reason))
    return matches


def _print_processes(title: str, processes: list[MatchedProcess]) -> None:
    print(title)
    if not processes:
        print("(none)")
        return
    for proc in processes:
        print(f"{proc.pid}\t{proc.user}\t{proc.reason}\t{proc.command}")


def _terminate(processes: list[MatchedProcess], wait_sec: float) -> dict:
    term_pids = [proc.pid for proc in processes]
    for pid in term_pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.time() + max(0.0, wait_sec)
    while time.time() < deadline:
        alive = [pid for pid in term_pids if _pid_alive(pid)]
        if not alive:
            break
        time.sleep(0.1)
    killed_pids = []
    for pid in term_pids:
        if not _pid_alive(pid):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            killed_pids.append(pid)
        except ProcessLookupError:
            pass
    return {"sigterm_pids": term_pids, "sigkill_pids": killed_pids}


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Check or clean Stage 0 ROS2 smoke leftover processes.")
    parser.add_argument("--pattern-set", default="stage0", choices=["stage0"])
    parser.add_argument("--kill", action="store_true", help="Terminate matched processes. Default is dry-run only.")
    parser.add_argument("--wait-sec", type=float, default=2.0, help="Seconds to wait after SIGTERM before SIGKILL.")
    parser.add_argument("--include-other-users", action="store_true", help="Allow matching processes from other users.")
    parser.add_argument("--json-out", default="", help="Optional JSON report path.")
    args = parser.parse_args()

    before = find_processes(args.pattern_set, include_other_users=args.include_other_users)
    _print_processes("before:", before)

    action = {"mode": "dry_run", "sigterm_pids": [], "sigkill_pids": []}
    if args.kill and before:
        action = {"mode": "kill", **_terminate(before, args.wait_sec)}
    elif args.kill:
        action = {"mode": "kill", "sigterm_pids": [], "sigkill_pids": []}

    after = find_processes(args.pattern_set, include_other_users=args.include_other_users)
    _print_processes("after:", after)

    report = {
        "pattern_set": args.pattern_set,
        "kill_requested": bool(args.kill),
        "action": action,
        "before": [asdict(proc) for proc in before],
        "after": [asdict(proc) for proc in after],
        "after_count": len(after),
    }
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fp:
            json.dump(report, fp, indent=2, ensure_ascii=False)
            fp.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
