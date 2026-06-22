"""Sim2Real runner for hip motor trajectory tracking.

This script runs either a simulation reference-tracking flow or a real motor
control loop. In `muscle` mode, the default pipeline is now:

trajectory -> torque -> actuators

and the legacy actuator-driven torque generation is still available as an option.

1) SIM MODE: uses Gym/MuJoCo environment (same as validation flow)
2) REAL MODE: subscribes to motor joint state and publishes torque command via ROS2

Safety defaults for REAL MODE:
- Torque output disabled by default (`--apply-torque` must be enabled explicitly)
- Command watchdog (forces zero torque if state updates are stale)
- Hard command clipping is opt-in via `--safe-torque-limit` and `--torque-limit-mode`

Typical usage:

SIM:
  python mimoEnv/hip_motor_sim2real_runner.py --mode sim --steps 4000 --no-render

REAL (dry run, no torque applied):
  python mimoEnv/hip_motor_sim2real_runner.py --mode real --rate-hz 100 --motor-id 308

REAL (apply torque, carefully):
  python mimoEnv/hip_motor_sim2real_runner.py --mode real --motor-id 308 \
      --apply-torque --safe-torque-limit 0.5 --rate-hz 100
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import importlib
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

# Optional simulation dependencies
try:
    gym = importlib.import_module("gymnasium")
    import mimoEnv  # noqa: F401 - gym env registration
    from mimoEnv.utils import get_joint_qpos_addr, get_joint_qvel_addr
except Exception:
    gym = None
    get_joint_qpos_addr = None
    get_joint_qvel_addr = None


@dataclass
class PIDGains:
    kp: float = 1.8
    ki: float = 0.15
    kd: float = 0.08


@dataclass
class VirtualActuator:
    amplitude: float
    frequency_hz: float
    phase_rad: float


@dataclass
class CsvReferenceTrajectory:
    phase_pct: np.ndarray
    angle_rad: np.ndarray
    slope_rad_per_pct: np.ndarray
    period_s: float
    phase_offset_pct: float
    ref_offset_rad: float


@dataclass
class MuscleUnitPattern:
    fmax: float
    vmax: float
    amplitude: float
    frequency_hz: float
    phase_rad: float
    bias: float


@dataclass
class HipMuscleParams:
    q_neutral: float = 0.01 # Posición neutral - son 0,57º de flexión
    q_min: float = 1.45 # Máxima flexión - son 83,07º
    q_max: float = -1.00 # Máxima extensión - son -57,29º
    lce_min: float = 0.75
    lce_max: float = 1.05
    tau: float = 0.02
    lmin: float = 0.5
    lmax: float = 1.6
    fvmax: float = 1.2
    fpmax: float = 1.3


@dataclass
class RealSafety:
    safe_torque_limit: Optional[float] = None
    watchdog_timeout_s: float = 0.25


class HipMuscleController:
    """Reduced 1-DoF hip muscle model inspired by MIMo's MuscleModel."""

    def __init__(
        self,
        flexors: Sequence[MuscleUnitPattern],
        extensors: Sequence[MuscleUnitPattern],
        params: HipMuscleParams,
    ) -> None:
        if not flexors or not extensors:
            raise ValueError("Need at least one flexor and one extensor muscle")

        self.params = params
        self.flexors = list(flexors)
        self.extensors = list(extensors)

        # Keep semantic endpoints (q_min=flexion, q_max=extension) even if numeric order is reversed.
        phi_flex = params.q_min - params.q_neutral
        phi_ext = params.q_max - params.q_neutral
        span = phi_ext - phi_flex
        if abs(span) < 1e-6:
            raise ValueError("Invalid hip geometry: q_min and q_max are too close")

        # Same geometric conversion used in mimoActuation.muscle.MuscleModel.
        self.moment_flex = (params.lce_max - params.lce_min) / span
        self.lce_flex_ref = params.lce_min - self.moment_flex * phi_flex
        self.moment_ext = (params.lce_max - params.lce_min) / (-span)
        self.lce_ext_ref = params.lce_min - self.moment_ext * phi_ext

        self.activity_flex = np.zeros((len(self.flexors),), dtype=np.float64)
        self.activity_ext = np.zeros((len(self.extensors),), dtype=np.float64)

    @staticmethod
    def _clip01(x: np.ndarray) -> np.ndarray:
        return np.clip(x, 0.0, 1.0)

    def _target_activity(self, t_s: float) -> Tuple[np.ndarray, np.ndarray]:
        u_flex = []
        for m in self.flexors:
            val = m.bias + m.amplitude * math.sin(2.0 * math.pi * m.frequency_hz * t_s + m.phase_rad)
            u_flex.append(val)

        u_ext = []
        for m in self.extensors:
            val = m.bias + m.amplitude * math.sin(2.0 * math.pi * m.frequency_hz * t_s + m.phase_rad)
            u_ext.append(val)

        return self._clip01(np.asarray(u_flex, dtype=np.float64)), self._clip01(np.asarray(u_ext, dtype=np.float64))

    @staticmethod
    def _bump(length: np.ndarray, a: float, mid: float, b: float) -> np.ndarray:
        left = 0.5 * (a + mid)
        right = 0.5 * (mid + b)

        a_dif = np.square(length - a) * 0.5
        b_dif = np.square(length - b) * 0.5
        mid_dif = np.square(length - mid) * 0.5

        output = b_dif / ((b - right) * (b - right))
        output[length < right] = 1 - mid_dif[length < right] / ((right - mid) * (right - mid))
        output[length < mid] = 1 - mid_dif[length < mid] / ((mid - left) * (mid - left))
        output[length < left] = a_dif[length < left] / ((left - a) * (left - a))
        output[(length <= a) | (length >= b)] = 0
        return output

    def _fl(self, lce: np.ndarray) -> np.ndarray:
        p = self.params
        return self._bump(lce, p.lmin, 1.0, p.lmax) + 0.15 * self._bump(
            lce,
            p.lmin,
            0.5 * (p.lmin + 0.95),
            0.95,
        )

    def _fv(self, lce_dot: np.ndarray, vmax: np.ndarray) -> np.ndarray:
        p = self.params
        c = p.fvmax - 1.0
        eff_vel = lce_dot / np.maximum(1e-6, vmax)
        eff_vel_con1 = eff_vel[eff_vel <= c]
        eff_vel_con2 = eff_vel[eff_vel <= 0]

        output = np.full(eff_vel.shape, p.fvmax, dtype=np.float64)
        if c > 1e-6:
            output[eff_vel <= c] = p.fvmax - (c - eff_vel_con1) * (c - eff_vel_con1) / c
        output[eff_vel <= 0] = (eff_vel_con2 + 1.0) * (eff_vel_con2 + 1.0)
        output[eff_vel < -1.0] = 0.0
        return output

    def _fp(self, lce: np.ndarray) -> np.ndarray:
        p = self.params
        b = 0.5 * (p.lmax + 1.0)
        tmp = (lce[lce <= b] - 1.0) / (b - 1.0)

        output = 0.25 * p.fpmax * (1.0 + 3.0 * (lce - b) / (b - 1.0))
        output[lce <= b] = 0.25 * p.fpmax * tmp * tmp * tmp
        output[lce <= 1.0] = 0.0
        return output

    def step(self, q: float, qd: float, t_s: float, dt: float) -> Tuple[float, dict]:
        p = self.params
        u_flex, u_ext = self._target_activity(t_s)

        tau_dyn = max(1e-6, p.tau)
        self.activity_flex += dt * (u_flex - self.activity_flex) / tau_dyn
        self.activity_ext += dt * (u_ext - self.activity_ext) / tau_dyn
        self.activity_flex = self._clip01(self.activity_flex)
        self.activity_ext = self._clip01(self.activity_ext)

        phi = q - p.q_neutral
        lce_flex = np.full_like(self.activity_flex, self.moment_flex * phi + self.lce_flex_ref)
        lce_ext = np.full_like(self.activity_ext, self.moment_ext * phi + self.lce_ext_ref)
        lce_dot_flex = np.full_like(self.activity_flex, self.moment_flex * qd)
        lce_dot_ext = np.full_like(self.activity_ext, self.moment_ext * qd)

        vmax_flex = np.asarray([m.vmax for m in self.flexors], dtype=np.float64)
        vmax_ext = np.asarray([m.vmax for m in self.extensors], dtype=np.float64)
        fmax_flex = np.asarray([m.fmax for m in self.flexors], dtype=np.float64)
        fmax_ext = np.asarray([m.fmax for m in self.extensors], dtype=np.float64)

        force_flex = self._fl(lce_flex) * self._fv(lce_dot_flex, vmax_flex) * self.activity_flex + self._fp(lce_flex)
        force_ext = self._fl(lce_ext) * self._fv(lce_dot_ext, vmax_ext) * self.activity_ext + self._fp(lce_ext)

        # MuJoCo convention in MuscleModel: joint torque is minus summed muscle moments.
        torque_nm = -(
            self.moment_flex * np.sum(force_flex * fmax_flex)
            + self.moment_ext * np.sum(force_ext * fmax_ext)
        )

        dbg = {
            "u_flex": u_flex.copy(),
            "u_ext": u_ext.copy(),
            "a_flex": self.activity_flex.copy(),
            "a_ext": self.activity_ext.copy(),
            "u_flex_mean": float(np.mean(u_flex)),
            "u_ext_mean": float(np.mean(u_ext)),
            "a_flex_mean": float(np.mean(self.activity_flex)),
            "a_ext_mean": float(np.mean(self.activity_ext)),
            "q_des": float(
                p.q_min
                + (p.q_max - p.q_min)
                * (float(np.mean(u_ext)) / (1e-6 + float(np.mean(u_ext)) + float(np.mean(u_flex))))
            ),
            "tau_raw": float(torque_nm),
        }
        return float(torque_nm), dbg


class Ros2MotorInterface:
    """ROS2 adapter for reading joint state and publishing torque commands."""

    def __init__(
        self,
        motor_id: int,
        joint_state_topic: str,
        command_topic: str,
        verbose: bool,
    ) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception as exc:
            raise RuntimeError(
                "ROS2 dependencies are missing. Install/source ROS2 before real mode."
            ) from exc

        self._rclpy = rclpy
        self._node_cls = Node
        self._joint_state_msg_cls = JointState
        self._verbose = verbose
        self.motor_id = int(motor_id)

        # Prefer candle_ros2 MotionCommand, fallback to rl_interfaces MotionCommand
        motion_cmd_cls = None
        try:
            candle_mod = importlib.import_module("candle_ros2.msg")
            motion_cmd_cls = getattr(candle_mod, "MotionCommand")
        except Exception:
            try:
                rl_mod = importlib.import_module("rl_interfaces.msg")
                motion_cmd_cls = getattr(rl_mod, "MotionCommand")
            except Exception as exc:
                raise RuntimeError(
                    "No MotionCommand message available (candle_ros2.msg or rl_interfaces.msg)."
                ) from exc

        self._motion_cmd_cls = motion_cmd_cls
        self._joint_state_topic = joint_state_topic
        self._command_topic = command_topic

        self.current_pos: Optional[float] = None
        self.current_vel: Optional[float] = None
        self.current_effort: Optional[float] = None
        self.last_state_ts: float = 0.0

        self._rclpy.init(args=None)
        self.node = self._node_cls("mimo_hip_sim2real_runner")

        self.pub = self.node.create_publisher(self._motion_cmd_cls, self._command_topic, 10)
        self.sub = self.node.create_subscription(
            self._joint_state_msg_cls,
            self._joint_state_topic,
            self._on_joint_state,
            10,
        )

        if self._verbose:
            self.node.get_logger().info(
                f"ROS2 interface ready: joint_state='{self._joint_state_topic}', command='{self._command_topic}'"
            )

    def _on_joint_state(self, msg) -> None:
        # Expected name format from MD80 stream: "Joint <id>"
        target_name = f"Joint {self.motor_id}"

        idx = None
        if getattr(msg, "name", None):
            for i, n in enumerate(msg.name):
                if n == target_name:
                    idx = i
                    break

        # Fallback if only one motor is present and name is unavailable/unexpected
        if idx is None:
            if len(msg.position) == 1:
                idx = 0
            else:
                return

        self.current_pos = float(msg.position[idx]) if idx < len(msg.position) else None
        self.current_vel = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
        self.current_effort = float(msg.effort[idx]) if idx < len(msg.effort) else 0.0
        self.last_state_ts = time.time()

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def wait_for_state(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 <= timeout_s:
            self.spin_once(timeout_s=0.05)
            if self.current_pos is not None:
                return True
        return False

    def publish_torque(self, torque_nm: float) -> None:
        msg = self._motion_cmd_cls()

        # Compatible shape with candle_ros2 / rl_interfaces MotionCommand
        msg.drive_ids = [self.motor_id]
        msg.target_position = [0.0]
        msg.target_velocity = [0.0]
        msg.target_torque = [float(torque_nm)]

        self.pub.publish(msg)

    def close(self) -> None:
        try:
            self.publish_torque(0.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        finally:
            self._rclpy.shutdown()


def _parse_virtual_actuators(spec: str) -> List[VirtualActuator]:
    parts = [x.strip() for x in spec.split(";") if x.strip()]
    actuators: List[VirtualActuator] = []
    for chunk in parts:
        fields = [x.strip() for x in chunk.split(",")]
        if len(fields) != 3:
            raise ValueError("Invalid virtual actuator format. Use 'amp,freq,phase;...'")
        amp, freq, phase = map(float, fields)
        actuators.append(VirtualActuator(amplitude=amp, frequency_hz=freq, phase_rad=phase))
    if not actuators:
        raise ValueError("At least one virtual actuator is required")
    return actuators


def _parse_muscle_units(spec: str) -> List[MuscleUnitPattern]:
    """Parse muscle spec: 'fmax,vmax,amp,freq,phase,bias;...'"""
    parts = [x.strip() for x in spec.split(";") if x.strip()]
    units: List[MuscleUnitPattern] = []
    for chunk in parts:
        fields = [x.strip() for x in chunk.split(",")]
        if len(fields) != 6:
            raise ValueError("Invalid muscle format. Use 'fmax,vmax,amp,freq,phase,bias;...'")
        fmax, vmax, amp, freq, phase, bias = map(float, fields)
        units.append(
            MuscleUnitPattern(
                fmax=fmax,
                vmax=vmax,
                amplitude=amp,
                frequency_hz=freq,
                phase_rad=phase,
                bias=bias,
            )
        )
    if not units:
        raise ValueError("At least one muscle unit is required")
    return units


def _clip_torque_value(torque_nm: float, torque_limit_nm: Optional[float]) -> float:
    if torque_limit_nm is None:
        return float(torque_nm)
    return float(np.clip(torque_nm, -torque_limit_nm, torque_limit_nm))


def _virtual_reference(
    t_s: float,
    actuators: Sequence[VirtualActuator],
    offset_rad: float,
) -> Tuple[float, float]:
    q = 0.0
    qd = 0.0
    for a in actuators:
        omega = 2.0 * math.pi * a.frequency_hz
        q += a.amplitude * math.sin(omega * t_s + a.phase_rad)
        qd += a.amplitude * omega * math.cos(omega * t_s + a.phase_rad)
    return offset_rad + q, qd


def _resolve_csv_column(columns: Sequence[str], explicit: str, candidates: Sequence[str], kind: str) -> str:
    lowered_map = {c.strip().lower(): c for c in columns}
    if explicit:
        key = explicit.strip().lower()
        if key not in lowered_map:
            raise ValueError(f"CSV {kind} column '{explicit}' not found. Available: {list(columns)}")
        return lowered_map[key]

    for name in candidates:
        if name in lowered_map:
            return lowered_map[name]

    raise ValueError(
        "Could not infer {} column from CSV header {}. Set --trajectory-csv-{}-column explicitly.".format(
            kind,
            list(columns),
            "phase" if kind == "phase" else "angle",
        )
    )


def _load_csv_reference(args: argparse.Namespace) -> CsvReferenceTrajectory:
    csv_path = args.trajectory_csv.strip()
    if not csv_path:
        raise ValueError("--trajectory-csv is required when ref source is CSV")
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")

    def _parse_localized_float(text: str) -> float:
        token = str(text).strip().replace("\ufeff", "")
        # Accept decimal comma (e.g., 0,123) used in many EU exports.
        token = token.replace(",", ".")
        return float(token)

    phase_values = []
    angle_values = []

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        raw_lines = [ln.strip() for ln in f if ln.strip()]

    if not raw_lines:
        raise ValueError(f"Trajectory CSV '{csv_path}' is empty.")

    first_line = raw_lines[0]
    has_header = any(ch.isalpha() for ch in first_line)

    if has_header:
        with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"CSV has no header: {csv_path}")

            phase_col = _resolve_csv_column(
                reader.fieldnames,
                args.trajectory_csv_phase_column,
                ("phase", "phase_pct", "gait_cycle", "gait_cycle_pct", "cycle_pct", "percent", "x"),
                kind="phase",
            )
            angle_col = _resolve_csv_column(
                reader.fieldnames,
                args.trajectory_csv_angle_column,
                ("hip_angle", "hip_angle_deg", "angle", "angle_deg", "y"),
                kind="angle",
            )

            for row in reader:
                try:
                    phase_values.append(_parse_localized_float(row[phase_col]))
                    angle_values.append(_parse_localized_float(row[angle_col]))
                except Exception:
                    continue
    else:
        # Headerless format: assume first column = phase, second column = angle.
        # Accept ';' separator (common with decimal comma exports) and fallback to ','.
        for line in raw_lines:
            parts = [p.strip() for p in line.split(";")] if ";" in line else [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                continue
            try:
                phase_values.append(_parse_localized_float(parts[0]))
                angle_values.append(_parse_localized_float(parts[1]))
            except Exception:
                continue

    if len(phase_values) < 4:
        raise ValueError(f"Trajectory CSV '{csv_path}' has too few valid rows ({len(phase_values)}).")

    phase = np.asarray(phase_values, dtype=np.float64)
    angle = np.asarray(angle_values, dtype=np.float64)

    # Accept either [0,100] percentage or [0,1] normalized cycle.
    if float(np.nanmax(np.abs(phase))) <= 1.5:
        phase = phase * 100.0

    phase = np.mod(phase, 100.0)
    order = np.argsort(phase)
    phase = phase[order]
    angle = angle[order]

    unique_phase, unique_idx = np.unique(phase, return_index=True)
    phase = unique_phase
    angle = angle[unique_idx]
    if phase.size < 4:
        raise ValueError("Trajectory CSV must provide at least 4 distinct phase points in [0,100).")

    if args.trajectory_y_normalized:
        y_min_rad = math.radians(float(args.trajectory_y_min_deg))
        y_max_rad = math.radians(float(args.trajectory_y_max_deg))
        angle = y_min_rad + (y_max_rad - y_min_rad) * angle
    elif args.trajectory_angle_unit == "deg":
        angle = np.deg2rad(angle)

    angle = angle * float(args.trajectory_scale)

    phase3 = np.concatenate([phase - 100.0, phase, phase + 100.0])
    angle3 = np.concatenate([angle, angle, angle])
    slope3 = np.gradient(angle3, phase3)
    slope = slope3[phase.size : 2 * phase.size]

    return CsvReferenceTrajectory(
        phase_pct=phase,
        angle_rad=angle,
        slope_rad_per_pct=slope,
        period_s=float(args.trajectory_period_s),
        phase_offset_pct=float(args.trajectory_phase_offset_pct),
        ref_offset_rad=float(args.ref_offset),
    )


def _csv_reference(t_s: float, traj: CsvReferenceTrajectory) -> Tuple[float, float]:
    period_s = max(1e-6, float(traj.period_s))
    phase_pct = ((100.0 * t_s / period_s) + traj.phase_offset_pct) % 100.0

    phase3 = np.concatenate([traj.phase_pct - 100.0, traj.phase_pct, traj.phase_pct + 100.0])
    angle3 = np.concatenate([traj.angle_rad, traj.angle_rad, traj.angle_rad])
    slope3 = np.concatenate([traj.slope_rad_per_pct, traj.slope_rad_per_pct, traj.slope_rad_per_pct])

    q = float(np.interp(phase_pct, phase3, angle3)) + traj.ref_offset_rad
    dq_dphase = float(np.interp(phase_pct, phase3, slope3))
    qd = dq_dphase * (100.0 / period_s)
    return q, qd


def _build_reference_sampler(
    args: argparse.Namespace,
    virtual_actuators: Sequence[VirtualActuator],
) -> Callable[[float], Tuple[float, float]]:
    csv_requested = bool(args.trajectory_csv.strip())
    source = str(args.ref_source).strip().lower()

    if source == "csv" and not csv_requested:
        raise ValueError("--ref-source csv requires --trajectory-csv <path>")

    if source == "virtual" or (source == "auto" and not csv_requested):
        return lambda t_s: _virtual_reference(t_s, virtual_actuators, args.ref_offset)

    if source in {"csv", "auto"} and csv_requested:
        traj = _load_csv_reference(args)
        print(
            "Using CSV trajectory: path='{}' points={} period={:.3f}s phase_offset={:+.2f}% ref_offset={:+.4f}rad".format(
                args.trajectory_csv,
                traj.phase_pct.size,
                traj.period_s,
                traj.phase_offset_pct,
                traj.ref_offset_rad,
            )
        )
        return lambda t_s: _csv_reference(t_s, traj)

    raise ValueError(f"Unsupported --ref-source '{args.ref_source}'")


def _auto_align_csv_phase_from_initial_position(
    args: argparse.Namespace,
    initial_motor_pos_rad: float,
) -> Optional[Tuple[float, float, float]]:
    """Align CSV phase offset so phase=0 time starts near current motor position.

    Returns:
        (matched_phase_pct, combined_phase_offset_pct, angle_error_rad) or None when not applied.
    """
    source = str(args.ref_source).strip().lower()
    csv_requested = bool(args.trajectory_csv.strip())
    if not bool(args.auto_phase_align):
        return None
    if source == "virtual" or (source == "auto" and not csv_requested):
        return None

    traj = _load_csv_reference(args)
    if traj.phase_pct.size == 0:
        return None

    # "First match" policy: np.argmin returns the first index on ties.
    idx = int(np.argmin(np.abs(traj.angle_rad - float(initial_motor_pos_rad))))
    matched_phase_pct = float(traj.phase_pct[idx])
    matched_angle = float(traj.angle_rad[idx])
    angle_error_rad = float(initial_motor_pos_rad - matched_angle)

    manual_phase_offset = float(args.trajectory_phase_offset_pct)
    combined_phase_offset = (matched_phase_pct + manual_phase_offset) % 100.0
    args.trajectory_phase_offset_pct = combined_phase_offset

    return matched_phase_pct, combined_phase_offset, angle_error_rad


def _synthesize_muscle_observables_from_torque(
    torque_nm: float,
    muscle_controller: HipMuscleController,
    dt: float,
    prev_a_flex: np.ndarray,
    prev_a_ext: np.ndarray,
    torque_scale_nm: float,
    q: float = 0.0,
    qd: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
    """Derive muscle activations from a torque command using inverse of the paper's muscle model.

    This implements the exact inverse of the muscle model described in arxiv.org/abs/2207.03952.
    Given a desired torque and current joint state (q, qd), it solves for the muscle activations
    that would produce that torque.

    In trajectory-driven mode, torque is the primary control output and muscle
    activations are derived for logging/visualization.
    """
    p = muscle_controller.params
    flexors = muscle_controller.flexors
    extensors = muscle_controller.extensors

    fmax_flex = np.asarray([m.fmax for m in flexors], dtype=np.float64)
    fmax_ext = np.asarray([m.fmax for m in extensors], dtype=np.float64)

    # Compute virtual muscle lengths and velocities from current joint state
    # (Same as in the paper's forward model)
    phi = q - p.q_neutral
    lce_flex = np.full_like(prev_a_flex, muscle_controller.moment_flex * phi + muscle_controller.lce_flex_ref)
    lce_ext = np.full_like(prev_a_ext, muscle_controller.moment_ext * phi + muscle_controller.lce_ext_ref)
    lce_dot_flex = np.full_like(prev_a_flex, muscle_controller.moment_flex * qd)
    lce_dot_ext = np.full_like(prev_a_ext, muscle_controller.moment_ext * qd)

    # Helper functions matching the paper's curves (copied from HipMuscleController)
    def _bump(length: np.ndarray, a: float, mid: float, b: float) -> np.ndarray:
        left = 0.5 * (a + mid)
        right = 0.5 * (mid + b)
        a_dif = np.square(length - a) * 0.5
        b_dif = np.square(length - b) * 0.5
        mid_dif = np.square(length - mid) * 0.5
        output = b_dif / ((b - right) * (b - right))
        output[length < right] = 1 - mid_dif[length < right] / ((right - mid) * (right - mid))
        output[length < mid] = 1 - mid_dif[length < mid] / ((mid - left) * (mid - left))
        output[length < left] = a_dif[length < left] / ((left - a) * (left - a))
        output[(length <= a) | (length >= b)] = 0
        return output

    def _fl(lce: np.ndarray) -> np.ndarray:
        """Force-length curve."""
        return _bump(lce, p.lmin, 1.0, p.lmax) + 0.15 * _bump(lce, p.lmin, 0.5 * (p.lmin + 0.95), 0.95)

    def _fv(lce_dot: np.ndarray, vmax: np.ndarray) -> np.ndarray:
        """Force-velocity curve."""
        c = p.fvmax - 1.0
        eff_vel = lce_dot / np.maximum(1e-6, vmax)
        eff_vel_con1 = eff_vel[eff_vel <= c]
        eff_vel_con2 = eff_vel[eff_vel <= 0]
        output = np.full(eff_vel.shape, p.fvmax, dtype=np.float64)
        if c > 1e-6:
            output[eff_vel <= c] = p.fvmax - (c - eff_vel_con1) * (c - eff_vel_con1) / c
        output[eff_vel <= 0] = (eff_vel_con2 + 1.0) * (eff_vel_con2 + 1.0)
        output[eff_vel < -1.0] = 0.0
        return output

    def _fp(lce: np.ndarray) -> np.ndarray:
        """Passive force component."""
        b = 0.5 * (p.lmax + 1.0)
        tmp = (lce[lce <= b] - 1.0) / (b - 1.0)
        output = 0.25 * p.fpmax * (1.0 + 3.0 * (lce - b) / (b - 1.0))
        output[lce <= b] = 0.25 * p.fpmax * tmp * tmp * tmp
        output[lce <= 1.0] = 0.0
        return output

    # Compute force multipliers from current state (independent of activation)
    vmax_flex = np.asarray([m.vmax for m in flexors], dtype=np.float64)
    vmax_ext = np.asarray([m.vmax for m in extensors], dtype=np.float64)

    fl_flex = _fl(lce_flex)
    fl_ext = _fl(lce_ext)
    fv_flex = _fv(lce_dot_flex, vmax_flex)
    fv_ext = _fv(lce_dot_ext, vmax_ext)
    fp_flex = _fp(lce_flex)
    fp_ext = _fp(lce_ext)

    # Inverse of the paper's forward model (1-DoF, underdetermined by group):
    #   tau = -(m_f * (sum(C_f * a_f) + P_f) + m_e * (sum(C_e * a_e) + P_e))
    # where C = fl*fv*fmax and P = sum(fp*fmax).
    # We activate one agonist group at a time and solve a scalar gain for a
    # weighted activation profile (non-uniform across units).
    flex_drive = np.zeros_like(prev_a_flex)
    ext_drive = np.zeros_like(prev_a_ext)

    eps = 1e-9
    m_f = float(muscle_controller.moment_flex)
    m_e = float(muscle_controller.moment_ext)

    c_f = fl_flex * fv_flex * fmax_flex
    c_e = fl_ext * fv_ext * fmax_ext
    p_f = float(np.sum(fp_flex * fmax_flex))
    p_e = float(np.sum(fp_ext * fmax_ext))

    # Torque with zero neural drive (only passive muscle contribution)
    tau_passive = -(m_f * p_f + m_e * p_e)

    # Per-muscle torque gains for activation units
    gain_vec_f = -(m_f * c_f)
    gain_vec_e = -(m_e * c_e)

    # Aggregate gain for all-ones activation (kept for diagnostics)
    gain_f = float(np.sum(gain_vec_f))
    gain_e = float(np.sum(gain_vec_e))

    target_tau = float(torque_nm)
    
    # Limit torque to maximum allowed scale (torque_scale_nm parameter)
    max_torque = float(torque_scale_nm) if torque_scale_nm > 1e-8 else float('inf')
    if abs(target_tau) > max_torque:
        target_tau = np.clip(target_tau, -max_torque, max_torque)

    if target_tau >= tau_passive:
        # Need more flexion torque than passive baseline -> use flexors.
        # Weighted profile based on instantaneous effectiveness.
        w = np.maximum(gain_vec_f, 0.0)
        if np.sum(w) <= eps:
            w = np.maximum(fmax_flex, 0.0)
        if np.sum(w) > eps:
            w = w / np.sum(w)
            den = float(np.dot(gain_vec_f, w))
            if abs(den) > eps:
                s = (target_tau - tau_passive) / den
                s = float(np.clip(s, 0.0, 1.0 / (float(np.max(w)) + eps)))
                flex_drive = np.clip(s * w, 0.0, 1.0)
    else:
        # Need more extension torque than passive baseline -> use extensors.
        # Weighted profile based on instantaneous effectiveness.
        w = np.maximum(-gain_vec_e, 0.0)
        if np.sum(w) <= eps:
            w = np.maximum(fmax_ext, 0.0)
        if np.sum(w) > eps:
            w = w / np.sum(w)
            den = float(np.dot(gain_vec_e, w))
            if abs(den) > eps:
                s = (target_tau - tau_passive) / den
                s = float(np.clip(s, 0.0, 1.0 / (float(np.max(w)) + eps)))
                ext_drive = np.clip(s * w, 0.0, 1.0)

    # Apply first-order low-pass filter (tau dynamics)
    tau_dyn = max(1e-6, p.tau)
    a_flex = prev_a_flex + dt * (flex_drive - prev_a_flex) / tau_dyn
    a_ext = prev_a_ext + dt * (ext_drive - prev_a_ext) / tau_dyn
    a_flex = np.clip(a_flex, 0.0, 1.0)
    a_ext = np.clip(a_ext, 0.0, 1.0)

    dbg = {
        "u_flex": flex_drive.copy(),
        "u_ext": ext_drive.copy(),
        "a_flex": a_flex.copy(),
        "a_ext": a_ext.copy(),
        "tau_passive": float(tau_passive),
        "gain_flex": float(gain_f),
        "gain_ext": float(gain_e),
        "u_flex_mean": float(np.mean(flex_drive)) if flex_drive.size else 0.0,
        "u_ext_mean": float(np.mean(ext_drive)) if ext_drive.size else 0.0,
        "a_flex_mean": float(np.mean(a_flex)) if a_flex.size else 0.0,
        "a_ext_mean": float(np.mean(a_ext)) if a_ext.size else 0.0,
    }

    return flex_drive, ext_drive, a_flex, a_ext, fmax_flex, fmax_ext, dbg


def _save_real_run_csv(
    csv_path: str,
    time_s: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    q_des: np.ndarray,
    torque_raw: np.ndarray,
    torque_clip: np.ndarray,
    torque_cmd: np.ndarray,
    torque_scale_gain: np.ndarray,
    u_flex: Optional[np.ndarray],
    u_ext: Optional[np.ndarray],
    a_flex: Optional[np.ndarray],
    a_ext: Optional[np.ndarray],
) -> None:
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)

    header = [
        "time_s",
        "q_rad",
        "qd_rad_s",
        "q_des_rad",
        "torque_raw_nm",
        "torque_clip_nm",
        "torque_cmd_nm",
        "torque_scale_gain",
    ]

    if u_flex is not None:
        header.extend([f"u_flex_{i}" for i in range(u_flex.shape[1])])
    if u_ext is not None:
        header.extend([f"u_ext_{i}" for i in range(u_ext.shape[1])])
    if a_flex is not None:
        header.extend([f"a_flex_{i}" for i in range(a_flex.shape[1])])
    if a_ext is not None:
        header.extend([f"a_ext_{i}" for i in range(a_ext.shape[1])])

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(time_s.shape[0]):
            row = [
                float(time_s[i]),
                float(q[i]),
                float(qd[i]),
                float(q_des[i]),
                float(torque_raw[i]),
                float(torque_clip[i]),
                float(torque_cmd[i]),
                float(torque_scale_gain[i]),
            ]
            if u_flex is not None:
                row.extend([float(x) for x in u_flex[i]])
            if u_ext is not None:
                row.extend([float(x) for x in u_ext[i]])
            if a_flex is not None:
                row.extend([float(x) for x in a_flex[i]])
            if a_ext is not None:
                row.extend([float(x) for x in a_ext[i]])
            writer.writerow(row)


def _write_run_config_txt(
    run_dir: str,
    args: argparse.Namespace,
    initial_motor_pos_rad: Optional[float] = None,
    flexors: Optional[Sequence[MuscleUnitPattern]] = None,
    extensors: Optional[Sequence[MuscleUnitPattern]] = None,
) -> str:
    """Save effective CLI configuration for reproducibility."""
    os.makedirs(run_dir, exist_ok=True)
    config_path = os.path.join(run_dir, "run_config.txt")

    lines = [
        f"timestamp: {dt.datetime.now().isoformat(timespec='seconds')}",
        f"command: {' '.join(sys.argv)}",
        "",
        "# effective arguments",
    ]
    for key in sorted(vars(args).keys()):
        lines.append(f"{key}: {getattr(args, key)}")

    if initial_motor_pos_rad is not None:
        lines.extend(
            [
                "",
                "# initial real motor state",
                f"initial_motor_pos_rad: {initial_motor_pos_rad:.9f}",
                f"initial_motor_pos_deg: {math.degrees(initial_motor_pos_rad):.6f}",
            ]
        )

    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

        # If muscle unit patterns were provided, also write explicit flex/ext muscle specs
        try:
            if flexors is not None and extensors is not None:
                # Format: fmax,vmax,amp,freq,phase,bias;... (matches parser _parse_muscle_units)
                def _format_units(units: Sequence[MuscleUnitPattern]) -> str:
                    parts = []
                    for m in units:
                        parts.append(
                            f"{float(m.fmax):.6g},{float(m.vmax):.6g},{float(m.amplitude):.6g},{float(m.frequency_hz):.6g},{float(m.phase_rad):.6g},{float(m.bias):.6g}"
                        )
                    return ";".join(parts)

                flex_spec = _format_units(flexors)
                ext_spec = _format_units(extensors)

                flex_fmax_scalar = float(sum(float(m.fmax) for m in flexors))
                flex_vmax_scalar = float(sum(float(m.vmax) for m in flexors) / max(1, len(flexors)))
                ext_fmax_scalar = float(sum(float(m.fmax) for m in extensors))
                ext_vmax_scalar = float(sum(float(m.vmax) for m in extensors) / max(1, len(extensors)))

                f.write("\n# explicit actuator muscle specs (scalar values applied in simulation)\n")
                for actuator_name in ("act:right_hip_flex", "act:left_hip_flex"):
                    f.write(f"actuator: {actuator_name}\n")
                    f.write(f"fmax_neg: {flex_fmax_scalar:.6g}\n")
                    f.write(f"vmax_neg: {flex_vmax_scalar:.6g}\n")
                    f.write(f"fmax_pos: {ext_fmax_scalar:.6g}\n")
                    f.write(f"vmax_pos: {ext_vmax_scalar:.6g}\n")
                    f.write(f"source_units_neg: {flex_spec}\n")
                    f.write(f"source_units_pos: {ext_spec}\n")
                    f.write("\n")

                # Backward-compatible summary for older readers.
                f.write(f"flex_muscles: {flex_spec}\n")
                f.write(f"ext_muscles: {ext_spec}\n")
        except Exception:
            # Non-fatal: best-effort write
            pass

    # Ensure PID gains are explicitly recorded (required)
    try:
        kp_val = float(getattr(args, "kp", PIDGains.kp))
    except Exception:
        kp_val = PIDGains.kp
    try:
        ki_val = float(getattr(args, "ki", PIDGains.ki))
    except Exception:
        ki_val = PIDGains.ki
    try:
        kd_val = float(getattr(args, "kd", PIDGains.kd))
    except Exception:
        kd_val = PIDGains.kd

    try:
        with open(config_path, "a", encoding="utf-8") as f:
            f.write("\n# PID gains\n")
            f.write(f"kp: {kp_val}\n")
            f.write(f"ki: {ki_val}\n")
            f.write(f"kd: {kd_val}\n")
    except Exception:
        pass

    return config_path


def _aligned_angles_for_plot(
    q: np.ndarray,
    q_des: np.ndarray,
    align_mode: str,
    align_anchor: str,
) -> Tuple[np.ndarray, np.ndarray, float, str]:
    """Return angle arrays for plotting with optional constant offset alignment."""
    if q.size == 0 or q_des.size == 0 or align_mode == "none":
        return q, q_des, 0.0, ""

    if align_anchor == "initial":
        q_anchor = float(q[0])
        q_des_anchor = float(q_des[0])
    else:
        q_anchor = float(np.mean(q))
        q_des_anchor = float(np.mean(q_des))

    if align_mode == "real_to_des":
        offset = q_des_anchor - q_anchor
        note = f"real_plot = real + {offset:+.4f} rad"
        return q + offset, q_des, offset, note

    if align_mode == "des_to_real":
        offset = q_anchor - q_des_anchor
        note = f"des_plot = des + {offset:+.4f} rad"
        return q, q_des + offset, offset, note

    return q, q_des, 0.0, ""


def _plot_real_run(
    plot_path: str,
    show_plot: bool,
    control_mode: str,
    time_s: np.ndarray,
    q: np.ndarray,
    q_des: np.ndarray,
    q_raw: np.ndarray,
    q_des_raw: np.ndarray,
    torque_raw: np.ndarray,
    torque_clip: np.ndarray,
    torque_cmd: np.ndarray,
    safe_torque_limit: Optional[float],
    a_flex: Optional[np.ndarray],
    a_ext: Optional[np.ndarray],
) -> bool:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.backend_bases import MouseEvent
    except Exception as exc:
        print(f"Matplotlib no disponible, no se pudo generar grafica: {exc}")
        return False

    os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)

    n_rows = 3 if a_flex is not None and a_ext is not None else 2
    fig, axes = plt.subplots(n_rows, 1, figsize=(13, 8), sharex=True)
    if n_rows == 2:
        ax1, ax2 = axes
        ax3 = None
    else:
        ax1, ax2, ax3 = axes

    ax1.plot(time_s, q, label="q real [rad]", color="#1f77b4", linewidth=1.8)
    ax1.plot(time_s, q_des, label="q objetivo [rad]", color="#d62728", linewidth=1.4, alpha=0.9)
    ax1.set_ylabel("Angulo [rad]")
    ax1.set_title(f"Hip motor real run ({control_mode})")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    ax2.plot(time_s, torque_raw, label="torque bruto [Nm]", color="#2ca02c", linewidth=1.2)
    #ax2.plot(time_s, torque_clip, label="torque limitado [Nm]", color="#ff7f0e", linewidth=1.2)
    ax2.plot(time_s, torque_cmd, label="torque enviado [Nm]", color="#9467bd", linewidth=1.6)
    if safe_torque_limit is not None:
        ax2.axhline(safe_torque_limit, color="#7f7f7f", linestyle="--", linewidth=1.1, alpha=0.9, label="límite +")
        ax2.axhline(-safe_torque_limit, color="#7f7f7f", linestyle="--", linewidth=1.1, alpha=0.9, label="límite -")
    ax2.set_ylabel("Torque [Nm]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")

    if ax3 is not None:
        for i in range(a_flex.shape[1]):
            ax3.plot(time_s, a_flex[:, i], linewidth=1.0, alpha=0.85, label=f"flex_{i}")
        for i in range(a_ext.shape[1]):
            ax3.plot(time_s, a_ext[:, i], linewidth=1.0, alpha=0.85, linestyle="--", label=f"ext_{i}")
        ax3.set_ylabel("Activacion [-]")
        ax3.set_xlabel("Tiempo [s]")
        ax3.set_ylim(-0.05, 1.05)
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="upper right", ncol=2, fontsize=8)
    else:
        ax2.set_xlabel("Tiempo [s]")

    # Modo interactivo: click en puntos para ver valores exactos
    annotations_list = []

    def on_click(event):
        if event.inaxes is None or event.button != 1:  # Solo click izquierdo
            return

        ax = event.inaxes
        x_clicked = event.xdata
        y_clicked = event.ydata

        # Buscar el punto más cercano en el eje actual
        if ax == ax1:
            # Eje de ángulo
            idx_q = np.argmin(np.abs(time_s - x_clicked))
            t_pt = time_s[idx_q]
            q_real = q_raw[idx_q]
            q_target = q_des_raw[idx_q]
            q_real_plot = q[idx_q]
            q_target_plot = q_des[idx_q]
            label_text = (
                f"t={t_pt:.3f}s\n"
                f"q_real={q_real:+.4f} rad\n"
                f"q_des={q_target:+.4f} rad\n"
                f"err={q_target - q_real:+.4f} rad\n"
                f"q_real_plot={q_real_plot:+.4f}\n"
                f"q_des_plot={q_target_plot:+.4f}"
            )
        elif ax == ax2:
            # Eje de torque
            idx_tau = np.argmin(np.abs(time_s - x_clicked))
            t_pt = time_s[idx_tau]
            tau_raw = torque_raw[idx_tau]
            tau_clip = torque_clip[idx_tau]
            tau_cmd = torque_cmd[idx_tau]
            label_text = f"t={t_pt:.3f}s\nraw={tau_raw:+.4f} Nm\nclip={tau_clip:+.4f} Nm\ncmd={tau_cmd:+.4f} Nm"
        elif ax == ax3:
            # Eje de activaciones
            idx_act = np.argmin(np.abs(time_s - x_clicked))
            t_pt = time_s[idx_act]
            flex_vals = a_flex[idx_act, :] if a_flex is not None else []
            ext_vals = a_ext[idx_act, :] if a_ext is not None else []
            label_text = f"t={t_pt:.3f}s\n"
            for i, val in enumerate(flex_vals):
                label_text += f"flex_{i}={val:.3f}\n"
            for i, val in enumerate(ext_vals):
                label_text += f"ext_{i}={val:.3f}"
        else:
            return

        # Crear anotación
        annot = ax.annotate(
            label_text,
            xy=(x_clicked, y_clicked),
            xytext=(10, 10),
            textcoords="offset points",
            fontsize=8,
            bbox=dict(boxstyle="round,pad=0.5", fc="yellow", alpha=0.7),
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0", color="black"),
        )
        annotations_list.append(annot)
        fig.canvas.draw()

    # Conectar event handler si se muestra el plot
    if show_plot:
        fig.canvas.mpl_connect("button_press_event", on_click)
        # Instrucciones
        print("\n=== MODO INTERACTIVO ACTIVADO ===")
        print("Click izquierdo en cualquier punto para ver valores exactos.")
        print("Cierra la ventana para terminar.")
        print("================================\n")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    if show_plot:
        plt.show()
    else:
        plt.close(fig)
    return True


def _compute_real_run_metrics(
    q: np.ndarray,
    q_des: np.ndarray,
    torque_cmd: np.ndarray,
    torque_clip: np.ndarray,
    safe_torque_limit: Optional[float],
    a_flex: Optional[np.ndarray],
    a_ext: Optional[np.ndarray],
) -> dict:
    if q.size == 0:
        return {}

    err = q_des - q
    metrics = {
        "rmse_q_rad": float(np.sqrt(np.mean(np.square(err)))),
        "mae_q_rad": float(np.mean(np.abs(err))),
        "max_abs_q_err_rad": float(np.max(np.abs(err))),
        "torque_cmd_rms_nm": float(np.sqrt(np.mean(np.square(torque_cmd)))),
        "torque_cmd_peak_nm": float(np.max(np.abs(torque_cmd))),
    }
    if safe_torque_limit is not None:
        metrics["saturation_ratio"] = float(np.mean(np.abs(torque_clip) >= 0.999 * max(1e-9, safe_torque_limit)))
    if a_flex is not None:
        metrics["mean_a_flex"] = float(np.mean(a_flex))
    if a_ext is not None:
        metrics["mean_a_ext"] = float(np.mean(a_ext))
    return metrics


def _plot_real_run_paper(
    plot_path: str,
    show_plot: bool,
    control_mode: str,
    time_s: np.ndarray,
    q: np.ndarray,
    q_des: np.ndarray,
    q_raw: np.ndarray,
    q_des_raw: np.ndarray,
    torque_raw: np.ndarray,
    torque_cmd: np.ndarray,
    safe_torque_limit: Optional[float],
    metrics: dict,
    a_flex: Optional[np.ndarray],
    a_ext: Optional[np.ndarray],
) -> bool:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.backend_bases import MouseEvent
    except Exception as exc:
        print(f"Matplotlib no disponible, no se pudo generar grafica paper: {exc}")
        return False

    os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)

    fig = plt.figure(figsize=(14, 7.8))
    gs = fig.add_gridspec(2, 2, height_ratios=[2.2, 1.2], width_ratios=[2.8, 1.2], hspace=0.25, wspace=0.2)
    ax_main = fig.add_subplot(gs[0, 0])
    ax_torque = fig.add_subplot(gs[1, 0], sharex=ax_main)
    ax_metrics = fig.add_subplot(gs[:, 1])

    ax_main.plot(time_s, q_des, color="#c43c39", linewidth=2.2, label="Objetivo")
    ax_main.plot(time_s, q, color="#1f5c9a", linewidth=1.8, label="Real")
    ax_main.set_ylabel("Angulo cadera [rad]")
    ax_main.set_title(f"Hip motor behavior ({control_mode})")
    ax_main.grid(True, alpha=0.22)
    ax_main.legend(loc="upper right", frameon=False)

    ax_torque.plot(time_s, torque_raw, color="#2ca02c", linewidth=1.1, alpha=0.9, label="Bruto")
    ax_torque.plot(time_s, torque_cmd, color="#2b8a3e", linewidth=1.4, alpha=0.95, label="Enviado")
    if safe_torque_limit is not None:
        ax_torque.axhline(safe_torque_limit, color="#7f7f7f", linestyle="--", linewidth=1.1, alpha=0.9, label="Límite +")
        ax_torque.axhline(-safe_torque_limit, color="#7f7f7f", linestyle="--", linewidth=1.1, alpha=0.9, label="Límite -")
    ax_torque.set_ylabel("Torque [Nm]")
    ax_torque.set_xlabel("Tiempo [s]")
    ax_torque.grid(True, alpha=0.22)
    ax_torque.legend(loc="best", frameon=False)

    ax_metrics.axis("off")
    lines = [
        "Resumen",
        "",
        f"RMSE angulo: {metrics.get('rmse_q_rad', float('nan')):.4f} rad",
        f"MAE angulo: {metrics.get('mae_q_rad', float('nan')):.4f} rad",
        f"Error max abs: {metrics.get('max_abs_q_err_rad', float('nan')):.4f} rad",
        "",
        f"Torque RMS: {metrics.get('torque_cmd_rms_nm', float('nan')):.4f} Nm",
        f"Torque pico: {metrics.get('torque_cmd_peak_nm', float('nan')):.4f} Nm",
        f"Ratio saturacion: {metrics['saturation_ratio'] * 100:.1f} %" if "saturation_ratio" in metrics else "Ratio saturacion: n/a",
    ]
    if "mean_a_flex" in metrics:
        lines.append(f"Activacion flex media: {metrics['mean_a_flex']:.3f}")
    if "mean_a_ext" in metrics:
        lines.append(f"Activacion ext media: {metrics['mean_a_ext']:.3f}")

    if a_flex is not None and a_ext is not None and a_flex.size > 0 and a_ext.size > 0:
        lines.extend(
            [
                "",
                f"N flexores: {a_flex.shape[1]}",
                f"N extensores: {a_ext.shape[1]}",
            ]
        )

    ax_metrics.text(
        0.02,
        0.98,
        "\n".join(lines),
        va="top",
        ha="left",
        fontsize=10,
        family="monospace",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#f8f9fb", "edgecolor": "#d0d7e2"},
    )

    # Modo interactivo: click en puntos para ver valores exactos
    annotations_list = []

    def on_click_paper(event):
        if event.inaxes is None or event.button != 1:  # Solo click izquierdo
            return

        ax = event.inaxes
        x_clicked = event.xdata
        y_clicked = event.ydata

        if ax == ax_main:
            # Eje principal: ángulo
            idx = np.argmin(np.abs(time_s - x_clicked))
            t_pt = time_s[idx]
            q_real = q_raw[idx]
            q_target = q_des_raw[idx]
            error = q_target - q_real
            label_text = f"t={t_pt:.3f}s\nq_real={q_real:+.4f} rad\nq_des={q_target:+.4f} rad\nerror={error:+.4f} rad"
            annot = ax.annotate(
                label_text,
                xy=(t_pt, q_real),
                xytext=(10, 10),
                textcoords="offset points",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.5", fc="lightyellow", alpha=0.9),
                arrowprops=dict(arrowstyle="->", color="black", lw=1),
            )
        elif ax == ax_torque:
            # Eje de torque
            idx = np.argmin(np.abs(time_s - x_clicked))
            t_pt = time_s[idx]
            tau = torque_cmd[idx]
            label_text = f"t={t_pt:.3f}s\nτ={tau:+.4f} Nm"
            annot = ax.annotate(
                label_text,
                xy=(t_pt, tau),
                xytext=(10, 10),
                textcoords="offset points",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.5", fc="lightgreen", alpha=0.9),
                arrowprops=dict(arrowstyle="->", color="black", lw=1),
            )
        else:
            return

        annotations_list.append(annot)
        fig.canvas.draw()

    if show_plot:
        fig.canvas.mpl_connect("button_press_event", on_click_paper)
        print("\n=== MODO INTERACTIVO - PAPER PLOT ===")
        print("Click izquierdo en los gráficos para ver valores exactos.")
        print("======================================\n")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=220)
    if show_plot:
        plt.show()
    else:
        plt.close(fig)
    return True


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MIMo hip sim2real trajectory runner")
    p.add_argument("--mode", choices=["sim", "real"], default="sim")
    p.add_argument(
        "--control-mode",
        choices=["pid", "muscle"],
        default="pid",
        help="Torque source in real mode: classic PID tracking or muscle activation model",
    )

    # Shared control config, initial trayectory
    p.add_argument(
        "--virtual-actuators",
        default="0.575,1,1.5707963268",
        help=(
            "Reference components 'amp,freq,phase;amp,freq,phase;...' . Default is a single sine"
            " that starts at the current motor position and swings toward extension"
        ),
    )
    p.add_argument(
        "--ref-source",
        choices=["auto", "virtual", "csv"],
        default="auto",
        help="Reference source: virtual sine actuators or periodic trajectory from CSV.",
    )
    p.add_argument(
        "--trajectory-csv",
        default="",
        help="Path to CSV with gait phase and hip angle columns. If set with ref-source auto, CSV is used.",
    )
    p.add_argument(
        "--trajectory-csv-phase-column",
        default="",
        help="Optional CSV phase column name (0..100 or 0..1 gait cycle).",
    )
    p.add_argument(
        "--trajectory-csv-angle-column",
        default="",
        help="Optional CSV hip-angle column name.",
    )
    p.add_argument(
        "--trajectory-angle-unit",
        choices=["deg", "rad"],
        default="deg",
        help="Angle unit in trajectory CSV.",
    )
    p.add_argument(
        "--trajectory-y-normalized",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Interpret trajectory Y as normalized [0,1] and map to [trajectory-y-min-deg, trajectory-y-max-deg].",
    )
    p.add_argument(
        "--trajectory-y-min-deg",
        type=float,
        default=-40.0,
        help="Physical lower angle in degrees when --trajectory-y-normalized is enabled.",
    )
    p.add_argument(
        "--trajectory-y-max-deg",
        type=float,
        default=30.0,
        help="Physical upper angle in degrees when --trajectory-y-normalized is enabled.",
    )
    p.add_argument(
        "--trajectory-period-s",
        type=float,
        default=1.063,
        help="Period of one gait cycle in seconds (for 1.3 m/s from paper, start with 1.063s).",
    )
    p.add_argument(
        "--trajectory-phase-offset-pct",
        type=float,
        default=0.0,
        help="Additional phase offset in gait-cycle percent points.",
    )
    p.add_argument(
        "--auto-phase-align",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In real mode with CSV reference, auto-align initial gait phase to current motor position.",
    )
    p.add_argument(
        "--trajectory-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to CSV trajectory amplitude.",
    )
    p.add_argument("--ref-offset", type=float, default=0.0)
    # PID in trajectory_to_torque flow:
    # torque_raw = kp * (q_ref - q) + ki * integral_error + kd * (qd_ref - qd)
    # - kp: position tracking strength
    # - ki: steady-state bias correction (friction/load)
    # - kd: damping from velocity mismatch
    p.add_argument("--kp", type=float, default=1.8)
    p.add_argument("--ki", type=float, default=0.15)
    p.add_argument("--kd", type=float, default=0.08)
    p.add_argument("--verbose-every", type=int, default=100)

    # Muscle mode (real only)
    p.add_argument(
        "--flex-muscles",
        default="10.0,1.2,0.25,0.8,0.0,0.30;8.0,1.2,0.18,1.4,0.8,0.25;6.0,1.2,0.15,0.5,-0.6,0.20",
        help="Flexor units as 'fmax,vmax,amp,freq,phase,bias;...'",
    )
    p.add_argument(
        "--ext-muscles",
        default="10.0,1.2,0.25,0.8,3.14,0.30;8.0,1.2,0.18,1.4,3.94,0.25;6.0,1.2,0.15,0.5,2.54,0.20",
        help="Extensor units as 'fmax,vmax,amp,freq,phase,bias;...'",
    )
    default_hip = HipMuscleParams()
    p.add_argument("--q-neutral", type=float, default=default_hip.q_neutral)
    p.add_argument("--q-min", type=float, default=default_hip.q_min)
    p.add_argument("--q-max", type=float, default=default_hip.q_max)
    p.add_argument("--muscle-tau", type=float, default=0.02)
    p.add_argument("--lce-min", type=float, default=0.75)
    p.add_argument("--lce-max", type=float, default=1.05)
    p.add_argument("--lmin", type=float, default=0.5)
    p.add_argument("--lmax", type=float, default=1.6)
    p.add_argument("--fvmax", type=float, default=1.2)
    p.add_argument("--fpmax", type=float, default=1.3)

    # Sim mode
    p.add_argument("--env", default="MIMoStandup-v0")
    p.add_argument("--steps", type=int, default=2000)
    p.add_argument("--warmup-steps", type=int, default=120)
    p.add_argument("--render", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--seed", type=int, default=7)

    # Real mode
    p.add_argument("--rate-hz", type=float, default=100.0, help="Control loop rate in real mode")
    p.add_argument("--duration-s", type=float, default=30.0, help="Real run duration")
    p.add_argument("--motor-id", type=int, default=349, help="Motor ID (prompted if not provided in real mode)")
    p.add_argument("--joint-state-topic", default="/md80/joint_states")
    p.add_argument("--command-topic", default="/md80/motion_command")
    p.add_argument("--safe-torque-limit", type=float, default=None, help="Optional hard torque limit in Nm")
    p.add_argument(
        "--torque-limit-mode",
        choices=["clip", "scale"],
        default=None,
        help="Optional torque limiting strategy; omit together with safe-torque-limit to disable limiting",
    )
    p.add_argument(
        "--torque-scale-decay-s",
        type=float,
        default=0.6,
        help="Only in scale mode: release time constant [s] for adaptive peak estimator",
    )
    p.add_argument(
        "--torque-scale-softness",
        type=float,
        default=1.6,
        help="Only in scale mode: soft saturation strength (>0, larger = more compression near limit)",
    )
    p.add_argument(
        "--torque-output-source",
        choices=["processed", "raw_scaled"],
        default="processed",
        help="Torque sent to hardware: processed limiter output or scaled raw torque",
    )
    p.add_argument(
        "--torque-raw-scale",
        type=float,
        default=(1.0 / 3.0),
        help="Only for torque-output-source=raw_scaled: command = torque_raw * this factor",
    )
    p.add_argument(
        "--torque-raw-scale-pos",
        type=float,
        default=None,
        help="Optional override for positive raw torque scaling when torque-output-source=raw_scaled",
    )
    p.add_argument(
        "--torque-raw-scale-neg",
        type=float,
        default=None,
        help="Optional override for negative raw torque scaling when torque-output-source=raw_scaled",
    )
    p.add_argument(
        "--torque-cmd-smoothing-s",
        type=float,
        default=0.0,
        help="Optional first-order smoothing time constant [s] for SENT torque command (0 disables)",
    )
    p.add_argument(
        "--muscle-flow",
        choices=["trajectory_to_torque", "actuators_to_torque"],
        default="trajectory_to_torque",
        help=(
            "Only when control-mode=muscle: choose whether the main control input is the "
            "trajectory (torque is computed from tracking error) or the legacy actuator-driven model"
        ),
    )
    p.add_argument("--watchdog-timeout-s", type=float, default=0.25)
    p.add_argument(
        "--apply-torque",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Actually send torque to hardware (default false = inference-only)",
    )
    p.add_argument(
        "--save-csv",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save real run telemetry to CSV",
    )
    p.add_argument(
        "--csv-path",
        default="",
        help="Output CSV path for real mode. If empty, auto-generates in outputs/",
    )
    p.add_argument(
        "--save-plot",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save run plot to PNG",
    )
    p.add_argument(
        "--plot-path",
        default="",
        help="Output plot path for real mode. If empty, auto-generates in outputs/",
    )
    p.add_argument(
        "--show-plot",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Display plot window at end of run",
    )
    p.add_argument(
        "--save-paper-plot",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Save a second documentation-oriented figure with embedded metrics",
    )
    p.add_argument(
        "--paper-plot-path",
        default="",
        help="Output paper-style plot path. If empty, auto-generates in outputs/",
    )
    p.add_argument(
        "--run-dir",
        default="",
        help="Per-run output folder. If empty, uses outputs/mimo/old_workflow/hip_motor_real/<timestamp>/",
    )
    p.add_argument(
        "--plot-angle-align",
        choices=["none", "real_to_des", "des_to_real"],
        default="real_to_des",
        help="Solo visualizacion: aplica offset constante para alinear angulos en la grafica.",
    )
    p.add_argument(
        "--plot-angle-align-anchor",
        choices=["mean", "initial"],
        default="mean",
        help="Referencia para calcular el offset de alineacion visual (media o primer punto).",
    )
    p.add_argument(
        "--track-relative-to-initial",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Solo en modo real con control por trayectoria: alinea la referencia al arranque del motor "
            "para calcular el error en coordenadas relativas y evitar picos por desfase inicial"
        ),
    )

    return p


def _run_sim_mode(args, gains: PIDGains, virtual_actuators: Sequence[VirtualActuator]) -> int:
    if gym is None or get_joint_qpos_addr is None or get_joint_qvel_addr is None:
        raise RuntimeError("Simulation dependencies unavailable (gymnasium/mimoEnv).")

    env = gym.make(args.env, render_mode="human" if args.render else None)
    sim = env.unwrapped
    env.reset(seed=args.seed)

    def _find_actuator(name: str) -> int:
        for i in range(sim.model.nu):
            if sim.model.actuator(i).name == name:
                return i
        raise RuntimeError(f"Actuator '{name}' not found")

    def _find_joint_idx(joint_name: str, vel: bool) -> int:
        j_id = sim.model.joint(joint_name).id
        idxs = list(get_joint_qvel_addr(sim.model, j_id) if vel else get_joint_qpos_addr(sim.model, j_id))
        if len(idxs) != 1:
            raise RuntimeError(f"Joint '{joint_name}' is not scalar")
        return idxs[0]

    right_act = _find_actuator("act:right_hip_flex")
    left_act = _find_actuator("act:left_hip_flex")
    qpos_r_idx = _find_joint_idx("robot:right_hip1", vel=False)
    qpos_l_idx = _find_joint_idx("robot:left_hip1", vel=False)
    qvel_r_idx = _find_joint_idx("robot:right_hip1", vel=True)
    qvel_l_idx = _find_joint_idx("robot:left_hip1", vel=True)

    action = np.zeros(env.action_space.shape, dtype=np.float32)
    int_e_r = 0.0
    int_e_l = 0.0
    ref_sampler = _build_reference_sampler(args, virtual_actuators)

    for _ in range(max(0, args.warmup_steps)):
        action[:] = 0.0
        env.step(action)
        if args.render:
            env.render()

    for i in range(args.steps):
        t_s = i * sim.dt
        q_ref, qd_ref = ref_sampler(t_s)

        q_r = float(sim.data.qpos[qpos_r_idx])
        q_l = float(sim.data.qpos[qpos_l_idx])
        qd_r = float(sim.data.qvel[qvel_r_idx])
        qd_l = float(sim.data.qvel[qvel_l_idx])

        e_r = q_ref - q_r
        e_l = q_ref - q_l
        int_e_r += e_r * sim.dt
        int_e_l += e_l * sim.dt

        u_r = gains.kp * e_r + gains.ki * int_e_r + gains.kd * (qd_ref - qd_r)
        u_l = gains.kp * e_l + gains.ki * int_e_l + gains.kd * (qd_ref - qd_l)

        action[:] = 0.0
        action[right_act] = np.clip(u_r, env.action_space.low[right_act], env.action_space.high[right_act])
        action[left_act] = np.clip(u_l, env.action_space.low[left_act], env.action_space.high[left_act])

        env.step(action)
        if args.render:
            env.render()

        if args.verbose_every > 0 and (i % args.verbose_every == 0):
            print(
                "[SIM step={:5d}] ref={:+.3f} q_r={:+.3f} q_l={:+.3f} e_r={:+.3f} e_l={:+.3f} u_r={:+.3f} u_l={:+.3f}".format(
                    i,
                    q_ref,
                    q_r,
                    q_l,
                    e_r,
                    e_l,
                    float(action[right_act]),
                    float(action[left_act]),
                ),
                flush=True,
            )

    env.close()
    print("Simulation run finished.")
    return 0


def _run_real_mode(args, gains: PIDGains, virtual_actuators: Sequence[VirtualActuator]) -> int:
    torque_limit = None
    if args.safe_torque_limit is not None and args.torque_limit_mode is not None:
        torque_limit = float(args.safe_torque_limit)

    safety = RealSafety(
        safe_torque_limit=torque_limit,
        watchdog_timeout_s=float(args.watchdog_timeout_s),
    )

    # Interactive motor_id prompt if not provided
    if args.motor_id is None:
        try:
            motor_id_str = input("Enter motor ID (default=308): ").strip()
            args.motor_id = int(motor_id_str) if motor_id_str else 308
        except ValueError:
            print("Invalid motor ID. Using default: 308")
            args.motor_id = 308
    
    dt_s = 1.0 / max(1e-3, float(args.rate_hz))
    max_steps = int(max(1.0, args.duration_s) * args.rate_hz)

    muscle_controller = None
    if args.control_mode == "muscle":
        muscle_params = HipMuscleParams(
            q_neutral=float(args.q_neutral),
            q_min=float(args.q_min),
            q_max=float(args.q_max),
            lce_min=float(args.lce_min),
            lce_max=float(args.lce_max),
            tau=float(args.muscle_tau),
            lmin=float(args.lmin),
            lmax=float(args.lmax),
            fvmax=float(args.fvmax),
            fpmax=float(args.fpmax),
        )
        muscle_controller = HipMuscleController(
            flexors=_parse_muscle_units(args.flex_muscles),
            extensors=_parse_muscle_units(args.ext_muscles),
            params=muscle_params,
        )

    interface = Ros2MotorInterface(
        motor_id=args.motor_id,
        joint_state_topic=args.joint_state_topic,
        command_topic=args.command_topic,
        verbose=True,
    )

    print("Waiting for first joint state...")
    if not interface.wait_for_state(timeout_s=5.0):
        interface.close()
        raise RuntimeError("No joint state received. Check candle_ros2 and topic names.")

    initial_motor_pos_rad = float(interface.current_pos if interface.current_pos is not None else 0.0)

    aligned_phase = _auto_align_csv_phase_from_initial_position(
        args=args,
        initial_motor_pos_rad=initial_motor_pos_rad,
    )
    if aligned_phase is not None:
        matched_phase_pct, combined_phase_offset, angle_error_rad = aligned_phase
        print(
            (
                "Auto phase align: matched_phase={:.2f}% combined_phase_offset={:.2f}% "
                "angle_error={:+.4f} rad ({:+.2f} deg)"
            ).format(
                matched_phase_pct,
                combined_phase_offset,
                angle_error_rad,
                math.degrees(angle_error_rad),
            )
        )

    ref_sampler = _build_reference_sampler(args, virtual_actuators)

    ref_initial_offset = 0.0
    if args.track_relative_to_initial and (
        args.control_mode == "pid"
        or (args.control_mode == "muscle" and args.muscle_flow == "trajectory_to_torque")
    ):
        q_ref0_raw, _ = ref_sampler(0.0)
        ref_initial_offset = initial_motor_pos_rad - float(q_ref0_raw)

    print(
        "REAL mode started: control_mode={} apply_torque={} safe_limit={} rate={:.1f}Hz duration={:.1f}s".format(
            args.control_mode,
            args.apply_torque,
            f"{safety.safe_torque_limit:.3f}Nm" if safety.safe_torque_limit is not None else "n/a",
            args.rate_hz,
            args.duration_s,
        )
    )
    if args.torque_cmd_smoothing_s > 0.0:
        print(
            "Command smoothing enabled: tau={:.4f}s (first-order LPF on sent torque)".format(
                float(args.torque_cmd_smoothing_s)
            )
        )
    print(
        "Initial motor position: {:+.6f} rad ({:+.3f} deg)".format(
            initial_motor_pos_rad,
            math.degrees(initial_motor_pos_rad),
        )
    )
    if args.track_relative_to_initial and abs(ref_initial_offset) > 1e-9:
        print(
            "Reference aligned to initial motor position: q_ref = q_ref_raw {:+.6f} rad".format(
                ref_initial_offset
            )
        )

    int_e = 0.0
    t0 = time.time()
    next_tick = t0
    cmd_prev = 0.0

    run_tag = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    default_run_name = f"{run_tag}"
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    run_dir = args.run_dir.strip() if args.run_dir else os.path.join(repo_root, "outputs", "mimo", "old_workflow", "real", default_run_name)
    os.makedirs(run_dir, exist_ok=True)

    csv_path = args.csv_path.strip() if args.csv_path else os.path.join(run_dir, "telemetry.csv")
    plot_path = args.plot_path.strip() if args.plot_path else os.path.join(run_dir, "timeseries.png")
    paper_plot_path = (
        args.paper_plot_path.strip()
        if args.paper_plot_path
        else os.path.join(run_dir, "paper_summary.png")
    )

    try:
        config_path = _write_run_config_txt(
            run_dir=run_dir,
            args=args,
            initial_motor_pos_rad=initial_motor_pos_rad,
            flexors=(muscle_controller.flexors if muscle_controller is not None else None),
            extensors=(muscle_controller.extensors if muscle_controller is not None else None),
        )
        print(f"Saved run config: {config_path}")
    except Exception as exc:
        print(f"No se pudo guardar run_config.txt: {exc}")

    t_hist: List[float] = []
    q_hist: List[float] = []
    qd_hist: List[float] = []
    q_des_hist: List[float] = []
    tau_raw_hist: List[float] = []
    tau_clip_hist: List[float] = []
    cmd_hist: List[float] = []
    scale_gain_hist: List[float] = []
    u_flex_hist: List[np.ndarray] = []
    u_ext_hist: List[np.ndarray] = []
    a_flex_hist: List[np.ndarray] = []
    a_ext_hist: List[np.ndarray] = []

    synthetic_a_flex = None
    synthetic_a_ext = None
    if args.control_mode == "muscle" and args.muscle_flow == "trajectory_to_torque":
        assert muscle_controller is not None
        synthetic_a_flex = np.zeros((len(muscle_controller.flexors),), dtype=np.float64)
        synthetic_a_ext = np.zeros((len(muscle_controller.extensors),), dtype=np.float64)

    try:
        # Adaptive peak estimate for scale mode (tracks recent envelope, not global max).
        running_abs_peak = max(1e-9, torque_limit if torque_limit is not None else 1.0)
        for i in range(max_steps):
            now = time.time()
            interface.spin_once(timeout_s=0.0)

            # Watchdog for stale motor state
            if now - interface.last_state_ts > safety.watchdog_timeout_s:
                interface.publish_torque(0.0)
                if i % max(1, args.verbose_every) == 0:
                    print(
                        f"[REAL step={i:5d}] watchdog stale state ({now - interface.last_state_ts:.3f}s) -> torque=0",
                        flush=True,
                    )
                time.sleep(dt_s)
                continue

            q = float(interface.current_pos if interface.current_pos is not None else 0.0)
            qd = float(interface.current_vel if interface.current_vel is not None else 0.0)
            t_rel = now - t0

            if args.control_mode == "muscle":
                assert muscle_controller is not None
                if args.muscle_flow == "actuators_to_torque":
                    tau_raw, dbg = muscle_controller.step(q=q, qd=qd, t_s=t_rel, dt=dt_s)
                    u_raw = float(tau_raw)
                    q_ref = float(dbg["q_des"])
                    qd_ref = float("nan")
                    err = q_ref - q
                else:
                    q_ref_raw, qd_ref = ref_sampler(t_rel)
                    q_ref = float(q_ref_raw + ref_initial_offset)
                    # Position error against desired trajectory.
                    err = q_ref - q
                    # Integral state accumulates tracking error over time.
                    int_e += err * dt_s
                    # Raw control torque before optional limiting/scaling.
                    u_raw = float(gains.kp * err + gains.ki * int_e + gains.kd * (qd_ref - qd))
                    
                    assert synthetic_a_flex is not None and synthetic_a_ext is not None
                    u_flex, u_ext, a_flex, a_ext, _, _, dbg = _synthesize_muscle_observables_from_torque(
                        torque_nm=u_raw,
                        muscle_controller=muscle_controller,
                        dt=dt_s,
                        prev_a_flex=synthetic_a_flex,
                        prev_a_ext=synthetic_a_ext,
                        torque_scale_nm=max(1e-9, abs(torque_limit) if torque_limit is not None else 1.0),
                        q=q,
                        qd=qd,
                    )
                    synthetic_a_flex = a_flex
                    synthetic_a_ext = a_ext
                    dbg["q_des"] = q_ref
                    dbg["tau_raw"] = float(u_raw)
            else:
                q_ref_raw, qd_ref = ref_sampler(t_rel)
                q_ref = float(q_ref_raw + ref_initial_offset)
                # Position error against desired trajectory.
                err = q_ref - q
                # Integral state accumulates tracking error over time.
                int_e += err * dt_s
                # Raw control torque before optional limiting/scaling.
                u_raw = float(gains.kp * err + gains.ki * int_e + gains.kd * (qd_ref - qd))

            if torque_limit is not None and args.torque_limit_mode == "scale":
                abs_raw = abs(u_raw)
                # Fast attack on new peaks + exponential release so gain recovers after transients.
                if abs_raw >= running_abs_peak:
                    running_abs_peak = abs_raw
                else:
                    decay_tau = max(1e-3, float(args.torque_scale_decay_s))
                    decay = math.exp(-dt_s / decay_tau)
                    running_abs_peak = max(abs_raw, running_abs_peak * decay)

                base_gain = min(1.0, torque_limit / max(1e-9, running_abs_peak))
                u_scaled = float(u_raw * base_gain)

                # Smooth compression close to limits to reduce jerk while preserving small commands.
                softness = max(1e-6, float(args.torque_scale_softness))
                x = u_scaled / max(1e-9, torque_limit)
                u = float(torque_limit * (math.tanh(softness * x) / math.tanh(softness)))

                # Effective gain after nonlinear compression (for telemetry).
                scale_gain = (u / u_raw) if abs_raw > 1e-9 else 1.0
                u = _clip_torque_value(u, torque_limit)
            else:
                scale_gain = 1.0
                u = _clip_torque_value(u_raw, torque_limit)

            if args.torque_output_source == "raw_scaled":
                # Final command from raw torque with optional sign-specific scaling.
                raw_scale = float(args.torque_raw_scale)
                if u_raw >= 0.0 and args.torque_raw_scale_pos is not None:
                    raw_scale = float(args.torque_raw_scale_pos)
                elif u_raw < 0.0 and args.torque_raw_scale_neg is not None:
                    raw_scale = float(args.torque_raw_scale_neg)

                cmd_candidate = float(u_raw * raw_scale)
                # Keep the same hard safety envelope regardless of output source.
                cmd_safety = _clip_torque_value(cmd_candidate, torque_limit)
            else:
                cmd_safety = float(u)

            cmd_target = cmd_safety if args.apply_torque else 0.0
            smoothing_tau = max(0.0, float(args.torque_cmd_smoothing_s))
            if smoothing_tau > 0.0:
                alpha = 1.0 - math.exp(-dt_s / max(1e-9, smoothing_tau))
                cmd = float(cmd_prev + alpha * (cmd_target - cmd_prev))
            else:
                cmd = float(cmd_target)

            cmd_prev = cmd
            interface.publish_torque(cmd)

            t_hist.append(float(t_rel))
            q_hist.append(float(q))
            qd_hist.append(float(qd))
            q_des_hist.append(float(q_ref))
            tau_clip_hist.append(float(u))
            cmd_hist.append(float(cmd))
            scale_gain_hist.append(float(scale_gain))
            if args.control_mode == "muscle":
                tau_raw_hist.append(float(dbg["tau_raw"]))
                u_flex_hist.append(np.asarray(dbg["u_flex"], dtype=np.float64))
                u_ext_hist.append(np.asarray(dbg["u_ext"], dtype=np.float64))
                a_flex_hist.append(np.asarray(dbg["a_flex"], dtype=np.float64))
                a_ext_hist.append(np.asarray(dbg["a_ext"], dtype=np.float64))
            else:
                tau_raw_hist.append(float(u))

            if args.verbose_every > 0 and (i % args.verbose_every == 0):
                if args.control_mode == "muscle":
                    print(
                        "[REAL step={:5d}] q={:+.3f} qd={:+.3f} a_flex={:.2f} a_ext={:.2f} tau_raw={:+.3f} gain={:.3f} cmd={:+.3f}".format(
                            i,
                            q,
                            qd,
                            dbg["a_flex_mean"],
                            dbg["a_ext_mean"],
                            dbg["tau_raw"],
                            scale_gain,
                            cmd,
                        ),
                        flush=True,
                    )
                else:
                    print(
                        "[REAL step={:5d}] ref={:+.3f} q={:+.3f} qd={:+.3f} e={:+.3f} u_raw={:+.3f} gain={:.3f} cmd={:+.3f}".format(
                            i,
                            q_ref,
                            q,
                            qd,
                            err,
                            u_raw,
                            scale_gain,
                            cmd,
                        ),
                        flush=True,
                    )

            next_tick += dt_s
            sleep_s = next_tick - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("Interrupted by user, forcing zero torque...")
    finally:
        interface.close()

    if t_hist:
        t_arr = np.asarray(t_hist, dtype=np.float64)
        q_arr = np.asarray(q_hist, dtype=np.float64)
        qd_arr = np.asarray(qd_hist, dtype=np.float64)
        q_des_arr = np.asarray(q_des_hist, dtype=np.float64)
        tau_raw_arr = np.asarray(tau_raw_hist, dtype=np.float64)
        tau_clip_arr = np.asarray(tau_clip_hist, dtype=np.float64)
        cmd_arr = np.asarray(cmd_hist, dtype=np.float64)
        scale_gain_arr = np.asarray(scale_gain_hist, dtype=np.float64)

        u_flex_arr = np.vstack(u_flex_hist) if u_flex_hist else None
        u_ext_arr = np.vstack(u_ext_hist) if u_ext_hist else None
        a_flex_arr = np.vstack(a_flex_hist) if a_flex_hist else None
        a_ext_arr = np.vstack(a_ext_hist) if a_ext_hist else None

        q_plot_arr, q_des_plot_arr, plot_offset_rad, plot_offset_note = _aligned_angles_for_plot(
            q=q_arr,
            q_des=q_des_arr,
            align_mode=args.plot_angle_align,
            align_anchor=args.plot_angle_align_anchor,
        )
        if plot_offset_note:
            print(f"Visual angle alignment: {plot_offset_note} (anchor={args.plot_angle_align_anchor})")

        if args.save_csv:
            _save_real_run_csv(
                csv_path=csv_path,
                time_s=t_arr,
                q=q_arr,
                qd=qd_arr,
                q_des=q_des_arr,
                torque_raw=tau_raw_arr,
                torque_clip=tau_clip_arr,
                torque_cmd=cmd_arr,
                torque_scale_gain=scale_gain_arr,
                u_flex=u_flex_arr,
                u_ext=u_ext_arr,
                a_flex=a_flex_arr,
                a_ext=a_ext_arr,
            )
            print(f"Saved CSV telemetry: {csv_path}")

        if args.save_plot:
            plot_ok = _plot_real_run(
                plot_path=plot_path,
                show_plot=bool(args.show_plot),
                control_mode=args.control_mode,
                time_s=t_arr,
                q=q_plot_arr,
                q_des=q_des_plot_arr,
                q_raw=q_arr,
                q_des_raw=q_des_arr,
                torque_raw=tau_raw_arr,
                torque_clip=tau_clip_arr,
                torque_cmd=cmd_arr,
                safe_torque_limit=safety.safe_torque_limit,
                a_flex=a_flex_arr,
                a_ext=a_ext_arr,
            )
            if plot_ok:
                print(f"Saved plot: {plot_path}")
            else:
                print("Plot no generado.")

        metrics = _compute_real_run_metrics(
            q=q_arr,
            q_des=q_des_arr,
            torque_cmd=cmd_arr,
            torque_clip=tau_clip_arr,
            safe_torque_limit=safety.safe_torque_limit,
            a_flex=a_flex_arr,
            a_ext=a_ext_arr,
        )

        if args.save_paper_plot:
            paper_ok = _plot_real_run_paper(
                plot_path=paper_plot_path,
                show_plot=bool(args.show_plot),
                control_mode=args.control_mode,
                time_s=t_arr,
                q=q_plot_arr,
                q_des=q_des_plot_arr,
                q_raw=q_arr,
                q_des_raw=q_des_arr,
                torque_raw=tau_raw_arr,
                torque_cmd=cmd_arr,
                safe_torque_limit=safety.safe_torque_limit,
                metrics=metrics,
                a_flex=a_flex_arr,
                a_ext=a_ext_arr,
            )
            if paper_ok:
                print(f"Saved paper plot: {paper_plot_path}")
            else:
                print("Paper plot no generado.")

    print("Real run finished.")
    return 0


def main() -> int:
    args = _build_parser().parse_args()
    gains = PIDGains(kp=args.kp, ki=args.ki, kd=args.kd)
    virtual_actuators = _parse_virtual_actuators(args.virtual_actuators)

    if args.mode == "sim":
        return _run_sim_mode(args, gains, virtual_actuators)
    return _run_real_mode(args, gains, virtual_actuators)


if __name__ == "__main__":
    raise SystemExit(main())
