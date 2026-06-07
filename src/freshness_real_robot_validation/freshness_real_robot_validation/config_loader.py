from __future__ import annotations

from pathlib import Path

import yaml


def load_yaml_config(path: str) -> dict:
    file_path = Path(path).expanduser()
    if not file_path.exists():
        return {}
    payload = yaml.safe_load(file_path.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}
