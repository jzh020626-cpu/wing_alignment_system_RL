# -*- coding: utf-8 -*-

import os


def now_sec(node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


def expanduser(path: str) -> str:
    return os.path.expanduser(path) if path else path
