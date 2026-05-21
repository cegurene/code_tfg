#!/usr/bin/env python3
"""Compatibility wrapper for the replay runner using the same pyCandle flow.

This mirrors the existing naming pattern in the repo so the new workflow has a
pair of runner scripts just like the current hip motor runners.
"""

from __future__ import annotations

import importlib.util
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(here, "hip_motor_torque_replay_runner_pycandle.py")
base_spec = importlib.util.spec_from_file_location("mimo_hip_motor_torque_replay_runner_pycandle_base", base_path)
if base_spec is None or base_spec.loader is None:
    raise RuntimeError(f"No se pudo cargar el runner base desde {base_path}")
base = importlib.util.module_from_spec(base_spec)
sys.modules[base_spec.name] = base
base_spec.loader.exec_module(base)


def main() -> int:
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())