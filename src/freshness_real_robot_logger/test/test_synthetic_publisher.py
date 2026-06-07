from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def test_synthetic_publisher_exits_after_duration():
    repo_root = Path(__file__).resolve().parents[4]
    script_path = repo_root / "ros2_ws" / "src" / "freshness_real_robot_logger" / "scripts" / "synthetic_calibration_publisher.py"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "36")

    result = subprocess.run(
        [
            sys.executable,
            str(script_path),
            "--rate-hz",
            "5",
            "--duration-sec",
            "0.5",
            "--drop-every-n",
            "0",
        ],
        cwd=str(repo_root),
        env=env,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 0, result.stderr
