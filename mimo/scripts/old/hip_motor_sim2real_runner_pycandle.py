#!/usr/bin/env python3
"""Sim2Real runner for hip motor tracking using pyCandle instead of ROS2 transport.

This runner reuses the control/telemetry pipeline from
`mimoEnv.hip_motor_sim2real_runner` and swaps only the real hardware interface
layer to communicate directly with pyCandle MD80 commands.
"""

from __future__ import annotations

import math
import os
import sys
import time
import importlib
import importlib.util
from typing import Optional

here = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(here, "hip_motor_sim2real_runner.py")
base_spec = importlib.util.spec_from_file_location("mimo_hip_motor_sim2real_runner_base", base_path)
if base_spec is None or base_spec.loader is None:
    raise RuntimeError(f"No se pudo cargar el runner base desde {base_path}")
base = importlib.util.module_from_spec(base_spec)
sys.modules[base_spec.name] = base
base_spec.loader.exec_module(base)


class PyCandleMotorInterface:
    """Drop-in replacement for Ros2MotorInterface using pyCandle."""

    _baud_label: str = "1M"
    _fdcan_enabled: bool = True
    _max_torque_nm: Optional[float] = 1.0
    _add_all_drives: bool = True
    _strict_id_match: bool = False

    @classmethod
    def configure_from_args(cls, args) -> None:
        cls._baud_label = str(args.pycandle_baud)
        cls._fdcan_enabled = bool(args.pycandle_fdcan)
        cls._max_torque_nm = (
            float(args.pycandle_max_torque) if args.pycandle_max_torque is not None else None
        )
        cls._add_all_drives = bool(args.pycandle_add_all_drives)
        cls._strict_id_match = bool(args.pycandle_strict_id_match)

    def __init__(
        self,
        motor_id: int,
        joint_state_topic: str,
        command_topic: str,
        verbose: bool,
    ) -> None:
        del joint_state_topic
        del command_topic

        try:
            pyCandle = importlib.import_module("pyCandle")
        except Exception as exc:
            raise RuntimeError("pyCandle no disponible. Instala/activa pyCandle antes de usar real mode.") from exc

        self._pycandle = pyCandle
        self._verbose = bool(verbose)
        self.motor_id = int(motor_id)

        baud = self._resolve_baud_constant(self._baud_label)
        self.candle = pyCandle.Candle(baud, self._fdcan_enabled)

        ids = list(self.candle.ping())
        if not ids:
            raise RuntimeError("No se detectaron drives en el bus CAN (pyCandle ping vacio).")

        ids_to_add = ids if self._add_all_drives else [self.motor_id]
        for drive_id in ids_to_add:
            if drive_id in ids:
                self.candle.addMd80(drive_id)

        if not getattr(self.candle, "md80s", None):
            raise RuntimeError("No se pudo inicializar ningun MD80 en pyCandle.")

        self.drive = self._select_drive(self.motor_id, ids)

        self.candle.controlMd80Mode(self.drive, pyCandle.RAW_TORQUE)
        self.candle.controlMd80Enable(self.drive, True)
        if self._max_torque_nm is not None:
            self.drive.setMaxTorque(float(self._max_torque_nm))

        self.candle.begin()

        self.current_pos: Optional[float] = None
        self.current_vel: Optional[float] = None
        self.current_effort: Optional[float] = None
        self.last_state_ts: float = 0.0

        self._poll_state()

        if self._verbose:
            print(
                "pyCandle interface ready: motor_id={} baud={} fdcan={} max_torque={}Nm".format(
                    self.motor_id,
                    self._baud_label,
                    self._fdcan_enabled,
                    "n/a" if self._max_torque_nm is None else f"{self._max_torque_nm:.3f}",
                ),
                flush=True,
            )

    def _resolve_baud_constant(self, label: str):
        normalized = str(label).strip().upper()
        if normalized in {"1M", "1MBPS", "1000000"}:
            if hasattr(self._pycandle, "CAN_BAUD_1M"):
                return self._pycandle.CAN_BAUD_1M
            raise RuntimeError("pyCandle no expone CAN_BAUD_1M.")

        if normalized in {"500K", "500KBPS", "500000"} and hasattr(self._pycandle, "CAN_BAUD_500K"):
            return self._pycandle.CAN_BAUD_500K

        raise RuntimeError(
            "Baudrate pyCandle no soportado: '{}'. Usa 1M (o 500K si tu pyCandle lo soporta).".format(label)
        )

    @staticmethod
    def _get_drive_id(drive) -> Optional[int]:
        try:
            if hasattr(drive, "getId"):
                return int(drive.getId())
        except Exception:
            pass

        try:
            if hasattr(drive, "id"):
                return int(drive.id)
        except Exception:
            pass

        return None

    def _select_drive(self, motor_id: int, ping_ids) -> object:
        drives = list(self.candle.md80s)

        for drive in drives:
            drive_id = self._get_drive_id(drive)
            if drive_id is not None and drive_id == motor_id:
                return drive

        if motor_id in ping_ids and len(drives) == len(ping_ids):
            idx = ping_ids.index(motor_id)
            if 0 <= idx < len(drives):
                return drives[idx]

        if len(drives) == 1 and not self._strict_id_match:
            return drives[0]

        known_ids = [self._get_drive_id(d) for d in drives]
        raise RuntimeError(
            "No se pudo mapear motor-id={} a un objeto MD80. IDs ping={} IDs md80s={}. "
            "Prueba --no-pycandle-add-all-drives o --no-pycandle-strict-id-match si solo hay un drive.".format(
                motor_id, ping_ids, known_ids
            )
        )

    def _poll_state(self) -> None:
        try:
            self.current_pos = float(self.drive.getPosition())
            self.current_vel = float(self.drive.getVelocity())
            self.current_effort = float(self.drive.getTorque())
            self.last_state_ts = time.time()
        except Exception:
            # Keep previous values; watchdog in control loop will zero command if stale.
            pass

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._poll_state()
        if timeout_s > 0.0:
            time.sleep(min(float(timeout_s), 0.002))

    def wait_for_state(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 <= timeout_s:
            self.spin_once(timeout_s=0.01)
            if self.current_pos is not None:
                return True
        return False

    def publish_torque(self, torque_nm: float) -> None:
        self.drive.setTargetTorque(float(torque_nm))

    def close(self) -> None:
        try:
            self.publish_torque(0.0)
            time.sleep(0.02)
        except Exception:
            pass
        try:
            self.candle.end()
        except Exception:
            pass
        try:
            self.candle.controlMd80Enable(self.drive, False)
        except Exception:
            pass


def _build_parser():
    parser = base._build_parser()
    parser.description = (
        "Sim2Real hip runner using pyCandle transport for REAL mode (no ROS2 topics for torque/state)."
    )
    parser.set_defaults(
        virtual_actuators="0.575,0.5,0.0",
        ref_offset=0.0,
        ref_source="csv",
        trajectory_csv=os.path.join(here, "healthy_hip_reference.csv"),
        trajectory_csv_phase_column="percent_cycle",
        trajectory_csv_angle_column="hip_angle_mean",
        trajectory_period_s=1.063,
        trajectory_angle_unit="deg",
    )
    parser.add_argument(
        "--pycandle-baud",
        default="1M",
        help="CAN baudrate label for pyCandle (1M, optional 500K if supported).",
    )
    parser.add_argument(
        "--pycandle-fdcan",
        action=base.argparse.BooleanOptionalAction,
        default=True,
        help="Enable FDCAN mode in pyCandle Candle constructor.",
    )
    parser.add_argument(
        "--pycandle-max-torque",
        type=float,
        default=1.0,
        help="Set MD80 max output torque in pyCandle (None to skip setMaxTorque).",
    )
    parser.add_argument(
        "--pycandle-add-all-drives",
        action=base.argparse.BooleanOptionalAction,
        default=True,
        help="Add all ping-detected drives to pyCandle update list.",
    )
    parser.add_argument(
        "--pycandle-strict-id-match",
        action=base.argparse.BooleanOptionalAction,
        default=False,
        help="Require exact mapping between --motor-id and drive object (no single-drive fallback).",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    gains = base.PIDGains(kp=args.kp, ki=args.ki, kd=args.kd)
    virtual_actuators = base._parse_virtual_actuators(args.virtual_actuators)

    if args.mode == "sim":
        return base._run_sim_mode(args, gains, virtual_actuators)

    PyCandleMotorInterface.configure_from_args(args)
    base.Ros2MotorInterface = PyCandleMotorInterface
    return base._run_real_mode(args, gains, virtual_actuators)


if __name__ == "__main__":
    raise SystemExit(main())
