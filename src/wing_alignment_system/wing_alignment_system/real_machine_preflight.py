#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import shutil
import shlex
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional


@dataclass
class CheckResult:
    name: str
    status: str
    message: str
    details: Dict[str, object]


def _mib(value_kib: int) -> float:
    return float(value_kib) / 1024.0


def read_meminfo(path: str = "/proc/meminfo") -> Dict[str, int]:
    out: Dict[str, int] = {}
    with open(path, "r", encoding="utf-8") as fp:
        for line in fp:
            parts = line.split()
            if len(parts) < 2:
                continue
            key = parts[0].rstrip(":")
            try:
                out[key] = int(parts[1])
            except ValueError:
                continue
    return out


def check_memory(
    meminfo: Dict[str, int],
    *,
    min_mem_available_mib: float,
    max_swap_used_mib: float,
) -> CheckResult:
    mem_available_mib = _mib(int(meminfo.get("MemAvailable", 0)))
    swap_total_mib = _mib(int(meminfo.get("SwapTotal", 0)))
    swap_free_mib = _mib(int(meminfo.get("SwapFree", 0)))
    swap_used_mib = max(0.0, swap_total_mib - swap_free_mib)
    details = {
        "mem_available_mib": round(mem_available_mib, 1),
        "swap_total_mib": round(swap_total_mib, 1),
        "swap_used_mib": round(swap_used_mib, 1),
        "min_mem_available_mib": float(min_mem_available_mib),
        "max_swap_used_mib": float(max_swap_used_mib),
    }
    if mem_available_mib < float(min_mem_available_mib):
        return CheckResult(
            "memory",
            "fail",
            "available memory is below the real-machine launch threshold",
            details,
        )
    if swap_used_mib > float(max_swap_used_mib):
        return CheckResult(
            "memory",
            "fail",
            "swap is already in heavy use before launch",
            details,
        )
    return CheckResult("memory", "pass", "memory headroom is acceptable", details)


def run_text(cmd: List[str], timeout_sec: float = 8.0) -> str:
    proc = subprocess.run(
        cmd,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=max(1.0, float(timeout_sec)),
    )
    return proc.stdout


def ros2_command(ros_domain_id: str = "", workspace_setup: str = "") -> List[str]:
    found = shutil.which("ros2")
    if found:
        base = [found]
    else:
        fallback = Path("/opt/ros/humble/bin/ros2")
        if fallback.exists():
            base = [str(fallback)]
        else:
            base = ["ros2"]
    domain = str(ros_domain_id or os.environ.get("ROS_DOMAIN_ID", "") or "36").strip()
    setup_candidates = [
        "/opt/ros/humble/setup.bash",
        workspace_setup,
        str(Path.cwd() / "install" / "setup.bash"),
        str(Path.home() / "hjz" / "install" / "setup.bash"),
    ]
    setup_cmds = []
    for setup in setup_candidates:
        if not setup:
            continue
        setup_path = Path(os.path.expanduser(setup))
        if setup_path.exists():
            setup_cmds.append(f"source {shlex.quote(str(setup_path))} 2>/dev/null || true")
    env_cmd = f"export ROS_DOMAIN_ID={shlex.quote(domain)}" if domain else ":"
    ros2_bin = "ros2" if found else shlex.quote(base[0])
    script = "; ".join([*setup_cmds, env_cmd, f"{ros2_bin} node list"])
    return ["bash", "-lc", script]


def ps_rows(run: Callable[[List[str], float], str] = run_text) -> List[Dict[str, object]]:
    text = run(["ps", "-eo", "pid=,ppid=,user=,rss=,pcpu=,args="], 8.0)
    rows: List[Dict[str, object]] = []
    for line in text.splitlines():
        parts = line.strip().split(maxsplit=5)
        if len(parts) < 6:
            continue
        try:
            rows.append(
                {
                    "pid": int(parts[0]),
                    "ppid": int(parts[1]),
                    "user": parts[2],
                    "rss_kib": int(parts[3]),
                    "pcpu": float(parts[4]),
                    "args": parts[5],
                }
            )
        except ValueError:
            continue
    return rows


def check_huatai_one_processes(
    rows: Iterable[Dict[str, object]],
    *,
    max_huatai_one: int,
) -> CheckResult:
    matches = [
        row for row in rows
        if "huatai_one" in str(row.get("args", ""))
        and "ros2 run huatai_one huatai_one" not in str(row.get("args", ""))
    ]
    details = {
        "count": len(matches),
        "max_huatai_one": int(max_huatai_one),
        "processes": [
            {
                "pid": row.get("pid"),
                "ppid": row.get("ppid"),
                "rss_mib": round(_mib(int(row.get("rss_kib", 0))), 1),
                "pcpu": row.get("pcpu"),
                "args": str(row.get("args", ""))[:240],
            }
            for row in matches
        ],
    }
    if len(matches) > int(max_huatai_one):
        return CheckResult(
            "huatai_one_processes",
            "fail",
            "multiple local huatai_one motor processes are active",
            details,
        )
    return CheckResult(
        "huatai_one_processes",
        "pass",
        "local huatai_one process count is acceptable",
        details,
    )


def duplicate_names(names: Iterable[str]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for raw in names:
        name = str(raw).strip()
        if not name or name.startswith("WARNING:"):
            continue
        counts[name] = counts.get(name, 0) + 1
    return {name: count for name, count in counts.items() if count > 1}


def check_ros_duplicate_nodes(
    *,
    skip: bool,
    ros_domain_id: str = "",
    workspace_setup: str = "",
    run: Callable[[List[str], float], str] = run_text,
) -> CheckResult:
    if skip:
        return CheckResult("ros_duplicate_nodes", "skip", "ROS graph check skipped", {})
    try:
        text = run(ros2_command(ros_domain_id, workspace_setup), 8.0)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return CheckResult(
            "ros_duplicate_nodes",
            "warn",
            "could not inspect ROS graph",
            {"error": str(exc), "ros_domain_id": str(ros_domain_id or os.environ.get("ROS_DOMAIN_ID", "") or "36")},
        )
    duplicates = duplicate_names(text.splitlines())
    details = {
        "duplicates": duplicates,
        "ros_domain_id": str(ros_domain_id or os.environ.get("ROS_DOMAIN_ID", "") or "36"),
    }
    if duplicates:
        return CheckResult(
            "ros_duplicate_nodes",
            "fail",
            "ROS graph contains duplicate node names",
            details,
        )
    return CheckResult("ros_duplicate_nodes", "pass", "no duplicate ROS node names detected", details)


def directory_size_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                continue
    return total


def check_ros_log_size(path: Path, *, warn_ros_dir_gib: float) -> CheckResult:
    size_bytes = directory_size_bytes(path)
    size_gib = float(size_bytes) / (1024.0 ** 3)
    details = {
        "path": str(path),
        "size_gib": round(size_gib, 2),
        "warn_ros_dir_gib": float(warn_ros_dir_gib),
    }
    if size_gib >= float(warn_ros_dir_gib):
        return CheckResult(
            "ros_log_size",
            "warn",
            "ROS log directory is large; archive or clean old run logs before long experiments",
            details,
        )
    return CheckResult("ros_log_size", "pass", "ROS log directory size is acceptable", details)


def build_report(args: argparse.Namespace) -> List[CheckResult]:
    meminfo = read_meminfo(args.meminfo_path)
    rows = ps_rows()
    ros_dir = Path(os.path.expanduser(args.ros_dir))
    return [
        check_memory(
            meminfo,
            min_mem_available_mib=args.min_mem_available_mib,
            max_swap_used_mib=args.max_swap_used_mib,
        ),
        check_huatai_one_processes(rows, max_huatai_one=args.max_huatai_one),
        check_ros_duplicate_nodes(
            skip=args.skip_ros_graph,
            ros_domain_id=args.ros_domain_id,
            workspace_setup=args.workspace_setup,
        ),
        check_ros_log_size(ros_dir, warn_ros_dir_gib=args.warn_ros_dir_gib),
    ]


def print_report(results: List[CheckResult]) -> None:
    for item in results:
        tag = item.status.upper()
        print(f"[{tag}] {item.name}: {item.message}")
        if item.details:
            print(json.dumps(item.details, ensure_ascii=False, sort_keys=True))


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preflight guard for full real-machine wing_alignment_system launches."
    )
    parser.add_argument("--min-mem-available-mib", type=float, default=1536.0)
    parser.add_argument("--max-swap-used-mib", type=float, default=512.0)
    parser.add_argument("--max-huatai-one", type=int, default=1)
    parser.add_argument("--warn-ros-dir-gib", type=float, default=5.0)
    parser.add_argument("--ros-dir", default="~/.ros")
    parser.add_argument("--meminfo-path", default="/proc/meminfo")
    parser.add_argument("--skip-ros-graph", action="store_true")
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "36"))
    parser.add_argument("--workspace-setup", default="install/setup.bash")
    parser.add_argument("--json-out", default="")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    results = build_report(args)
    print_report(results)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fp:
            json.dump([asdict(item) for item in results], fp, indent=2, ensure_ascii=False)
            fp.write("\n")
    return 2 if any(item.status == "fail" for item in results) else 0


if __name__ == "__main__":
    sys.exit(main())
