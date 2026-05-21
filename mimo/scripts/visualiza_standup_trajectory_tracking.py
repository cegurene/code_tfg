#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import importlib.util
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

import mimoEnv  # Registers custom environments.
from mimoActuation.muscle import MuscleModel




@dataclass
class ReferenceTrajectory:
    phase_pct: np.ndarray
    angle_rad: np.ndarray
    period_s: float

@dataclass
class MuscleUnitPattern:
    """Pattern for a single muscle unit with sinusoidal activation."""
    fmax: float
    vmax: float
    amplitude: float
    frequency_hz: float
    phase_rad: float
    bias: float


@dataclass
class HipMuscleParams:
    """Parameters for 1-DoF hip muscle model."""
    q_neutral: float = 0.01
    q_min: float = 1.45
    q_max: float = -1.00
    lce_min: float = 0.75
    lce_max: float = 1.05
    tau: float = 0.02
    lmin: float = 0.5
    lmax: float = 1.6
    fvmax: float = 1.2
    fpmax: float = 1.3


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
        phi_flex = params.q_min - params.q_neutral
        phi_ext = params.q_max - params.q_neutral
        span = phi_ext - phi_flex
        if abs(span) < 1e-6:
            raise ValueError("Invalid hip geometry: q_min and q_max are too close")
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
        u_flex = [m.bias + m.amplitude * math.sin(2.0 * math.pi * m.frequency_hz * t_s + m.phase_rad) for m in self.flexors]
        u_ext = [m.bias + m.amplitude * math.sin(2.0 * math.pi * m.frequency_hz * t_s + m.phase_rad) for m in self.extensors]
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
        return self._bump(lce, p.lmin, 1.0, p.lmax) + 0.15 * self._bump(lce, p.lmin, 0.5 * (p.lmin + 0.95), 0.95)

    def _fv(self, lce_dot: np.ndarray, vmax: np.ndarray) -> np.ndarray:
        p = self.params
        c = p.fvmax - 1.0
        eff_vel = lce_dot / np.maximum(1e-6, vmax)
        output = np.full(eff_vel.shape, p.fvmax, dtype=np.float64)
        if c > 1e-6:
            output[eff_vel <= c] = p.fvmax - (c - eff_vel[eff_vel <= c]) * (c - eff_vel[eff_vel <= c]) / c
        output[eff_vel <= 0] = (eff_vel[eff_vel <= 0] + 1.0) * (eff_vel[eff_vel <= 0] + 1.0)
        output[eff_vel < -1.0] = 0.0
        return output

    def _fp(self, lce: np.ndarray) -> np.ndarray:
        p = self.params
        b = 0.5 * (p.lmax + 1.0)
        output = 0.25 * p.fpmax * (1.0 + 3.0 * (lce - b) / (b - 1.0))
        output[lce <= b] = 0.25 * p.fpmax * ((lce[lce <= b] - 1.0) / (b - 1.0)) ** 3
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
        torque_nm = -(self.moment_flex * np.sum(force_flex * fmax_flex) + self.moment_ext * np.sum(force_ext * fmax_ext))
        dbg = {
            "u_flex": u_flex.copy(),
            "u_ext": u_ext.copy(),
            "a_flex": self.activity_flex.copy(),
            "a_ext": self.activity_ext.copy(),
            "u_flex_mean": float(np.mean(u_flex)),
            "u_ext_mean": float(np.mean(u_ext)),
            "a_flex_mean": float(np.mean(self.activity_flex)),
            "a_ext_mean": float(np.mean(self.activity_ext)),
            "q_des": float(p.q_min + (p.q_max - p.q_min) * (float(np.mean(u_ext)) / (1e-6 + float(np.mean(u_ext)) + float(np.mean(u_flex))))),
            "tau_raw": float(torque_nm),
        }
        return float(torque_nm), dbg


def _parse_muscle_units(spec: str) -> list[MuscleUnitPattern]:
    """Parse muscle spec: 'fmax,vmax,amp,freq,phase,bias;...'"""
    parts = [x.strip() for x in spec.split(";") if x.strip()]
    units: list[MuscleUnitPattern] = []
    for chunk in parts:
        fields = [x.strip() for x in chunk.split(",")]
        if len(fields) != 6:
            raise ValueError("Invalid muscle format. Use 'fmax,vmax,amp,freq,phase,bias;...'")
        fmax, vmax, amp, freq, phase, bias = map(float, fields)
        units.append(MuscleUnitPattern(fmax=fmax, vmax=vmax, amplitude=amp, frequency_hz=freq, phase_rad=phase, bias=bias))
    if not units:
        raise ValueError("At least one muscle unit is required")
    return units


@dataclass
class HipTelemetryRow:
    time_s: float
    target_right_rad: float
    actual_right_rad: float
    torque_right_nm: float
    right_flex_activation: float
    right_ext_activation: float
    target_left_rad: float
    actual_left_rad: float
    torque_left_nm: float
    left_flex_activation: float
    left_ext_activation: float


def _parse_float(text: str) -> float:
    return float(str(text).strip().replace("\ufeff", "").replace(",", "."))


def _load_reference_csv(
    csv_path: str,
    phase_column: str,
    angle_column: str,
    angle_unit: str,
    scale: float,
    angle_sign: float,
) -> ReferenceTrajectory:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"No existe el CSV de referencia: {csv_path}")

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"El CSV de referencia '{csv_path}' no tiene cabecera.")

        lowered = {name.strip().lower(): name for name in reader.fieldnames}

        def resolve(name: str, candidates: Sequence[str]) -> str:
            if name:
                key = name.strip().lower()
                if key not in lowered:
                    raise ValueError(
                        f"No se encontró la columna '{name}' en {list(reader.fieldnames)}"
                    )
                return lowered[key]
            for candidate in candidates:
                if candidate in lowered:
                    return lowered[candidate]
            raise ValueError(f"No se pudo inferir la columna en {list(reader.fieldnames)}.")

        phase_name = resolve(phase_column, ["percent_cycle", "phase", "phase_pct", "cycle_pct"])
        angle_name = resolve(angle_column, ["hip_angle_mean", "angle", "hip_angle", "q_rad", "q_deg"])
        phase_values = []
        angle_values = []
        for row in reader:
            phase_values.append(_parse_float(row[phase_name]))
            angle_values.append(_parse_float(row[angle_name]))

        phase = np.asarray(phase_values, dtype=np.float64)
        angle = np.asarray(angle_values, dtype=np.float64)

        # Accept phases in 0..1 as fraction and convert to percent (0..100)
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
            raise ValueError("La referencia necesita al menos 4 puntos distintos de fase.")

        if angle_unit == "deg":
            angle = np.deg2rad(angle)

        angle = angle * float(scale)
        angle = angle * float(angle_sign)

        return ReferenceTrajectory(phase_pct=phase, angle_rad=angle, period_s=1.063)


def _reference_at_phase(traj: ReferenceTrajectory, phase_pct: float) -> float:
    phase3 = np.concatenate([traj.phase_pct - 100.0, traj.phase_pct, traj.phase_pct + 100.0])
    angle3 = np.concatenate([traj.angle_rad, traj.angle_rad, traj.angle_rad])
    return float(np.interp(phase_pct % 100.0, phase3, angle3))


def _find_initial_phase(traj: ReferenceTrajectory, current_pos: float) -> float:
    """Find which phase in the trajectory matches the current position.
    
    Args:
        traj: Reference trajectory
        current_pos: Current joint position in radians
        
    Returns:
        Phase (0-100) where trajectory matches current position closest
    """
    # Sample trajectory at many phases to find closest match
    phases = np.linspace(0, 100, 1000)
    angles = np.array([_reference_at_phase(traj, p) for p in phases])
    
    # Find phase with minimum distance to current position
    idx_closest = np.argmin(np.abs(angles - current_pos))
    return float(phases[idx_closest])


def _find_initial_phase_with_offset(traj: ReferenceTrajectory, current_pos: float, phase_offset_pct: float) -> float:
    phases = np.linspace(0, 100, 1000)
    angles = np.array([_reference_at_phase(traj, p + phase_offset_pct) for p in phases])
    idx_closest = np.argmin(np.abs(angles - current_pos))
    return float(phases[idx_closest])


def _find_phase_delay_for_position(traj: ReferenceTrajectory, current_pos: float, start_phase_pct: float) -> float:
    phases = np.linspace(0.0, 100.0, 1000)
    angles = np.array([_reference_at_phase(traj, start_phase_pct + p) for p in phases])
    idx_closest = np.argmin(np.abs(angles - current_pos))
    return float(phases[idx_closest])


def _resolve_joint_scalar(model, joint_name: str, *, qvel: bool) -> int:
    joint_id = model.joint(joint_name).id
    if qvel:
        return int(model.jnt_dofadr[joint_id])
    return int(model.jnt_qposadr[joint_id])


def _find_actuator_index(actuator_names: Sequence[str], target_name: str) -> Optional[int]:
    try:
        return actuator_names.index(target_name)
    except ValueError:
        return None


def _weighted_mean(values: np.ndarray, weights: Optional[Sequence[float]]) -> float:
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return 0.0
    if weights is None:
        return float(np.mean(arr))
    w = np.asarray(weights, dtype=np.float64)
    if w.size != arr.size:
        return float(np.mean(arr))
    denom = float(np.sum(w))
    if abs(denom) < 1e-12:
        return float(np.mean(arr))
    return float(np.sum(arr * w) / denom)


def _combine_action_from_muscle_outputs(u_flex: np.ndarray, u_ext: np.ndarray, fmax_flex, fmax_ext) -> Tuple[float, float]:
    return _weighted_mean(u_flex, fmax_flex), _weighted_mean(u_ext, fmax_ext)


def _scale_muscle_units(units: Sequence, scale: float):
    if abs(float(scale) - 1.0) < 1e-12:
        return list(units)
    scaled = []
    for unit in units:
        scaled.append(
            MuscleUnitPattern(
                fmax=float(unit.fmax) * float(scale),
                vmax=float(unit.vmax),
                amplitude=float(unit.amplitude),
                frequency_hz=float(unit.frequency_hz),
                phase_rad=float(unit.phase_rad),
                bias=float(unit.bias),
            )
        )
    return scaled


def _row_to_list(row: HipTelemetryRow) -> list[float]:
    return [
        float(row.time_s),
        float(row.target_right_rad),
        float(row.actual_right_rad),
        float(row.torque_right_nm),
        float(row.right_flex_activation),
        float(row.right_ext_activation),
        float(row.target_left_rad),
        float(row.actual_left_rad),
        float(row.torque_left_nm),
        float(row.left_flex_activation),
        float(row.left_ext_activation),
    ]


def _write_run_config_txt(
    run_dir: str,
    args: argparse.Namespace,
    reference_csv: str,
    reference: ReferenceTrajectory,
    env_id: str,
    actuator_names: Sequence[str],
) -> str:
    os.makedirs(run_dir, exist_ok=True)
    config_path = os.path.join(run_dir, "run_config.txt")

    lines = [
        f"timestamp: {dt.datetime.now().isoformat(timespec='seconds')}",
        f"command: {' '.join(sys.argv)}",
        "",
        f"env_id: {env_id}",
        f"reference_csv: {reference_csv}",
        f"reference_period_s: {reference.period_s}",
        f"reference_points: {reference.phase_pct.size}",
        f"reference_phase_offset_pct: {args.trajectory_phase_offset_pct}",
        f"left_phase_offset_pct: {args.left_phase_offset_pct}",
        f"target_cycles: {args.cycles}",
        f"fps_assumed_for_plot: {args.plot_fps}",
        "",
        "# effective arguments",
    ]
    for key in sorted(vars(args).keys()):
        lines.append(f"{key}: {getattr(args, key)}")

    lines.extend(["", "# actuator names"])
    for i, name in enumerate(actuator_names):
        lines.append(f"{i}: {name}")

    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return config_path


def _save_csv(csv_path: str, rows: Sequence[HipTelemetryRow]) -> None:
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    header = [
        "time_s",
        "target_right_rad",
        "actual_right_rad",
        "torque_right_nm",
        "right_flex",
        "right_ext",
        "target_left_rad",
        "actual_left_rad",
        "torque_left_nm",
        "left_flex",
        "left_ext",
    ]

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row in rows:
            writer.writerow(_row_to_list(row))


def _plot_run(
    plot_path: str,
    show_plot: bool,
    time_s: np.ndarray,
    target_right: np.ndarray,
    actual_right: np.ndarray,
    torque_right: np.ndarray,
    right_flex: np.ndarray,
    right_ext: np.ndarray,
    target_left: np.ndarray,
    actual_left: np.ndarray,
    torque_left: np.ndarray,
    left_flex: np.ndarray,
    left_ext: np.ndarray,
) -> None:
    os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    ax0 = axes[0]
    ax0.plot(time_s, target_right, color="#1f5c9a", linestyle="--", linewidth=1.8, label="Derecha objetivo")
    ax0.plot(time_s, actual_right, color="#1f5c9a", linewidth=2.0, label="Derecha simulada")
    ax0.plot(time_s, target_left, color="#c43c39", linestyle="--", linewidth=1.8, label="Izquierda objetivo")
    ax0.plot(time_s, actual_left, color="#c43c39", linewidth=2.0, label="Izquierda simulada")
    ax0.set_ylabel("Ángulo [rad]")
    ax0.set_title("Seguimiento de caderas en simulación")
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="best", frameon=False, ncol=2)
    ax0.invert_yaxis()

    ax1 = axes[1]
    ax1.plot(time_s, torque_right, color="#2b8a3e", linewidth=1.8, label="Torque derecha")
    ax1.plot(time_s, torque_left, color="#8a5b2b", linewidth=1.8, label="Torque izquierda")
    ax1.set_ylabel("Torque [Nm]")
    ax1.set_title("Torque aplicado por cadera")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="best", frameon=False)
    ax1.invert_yaxis()

    labels = ["flex", "ext"]
    colors = ["#264653", "#e76f51"]

    ax2 = axes[2]
    for idx, label in enumerate(labels):
        ax2.plot(time_s, right_flex if idx == 0 else right_ext, label=label, linewidth=1.2, color=colors[idx])
    ax2.set_ylabel("Activación [-]")
    ax2.set_title("Activaciones de la cadera derecha")
    ax2.set_ylim(-0.05, 1.05)
    ax2.grid(True, alpha=0.25)
    ax2.legend(loc="upper right", ncol=2, fontsize=8, frameon=False)

    ax3 = axes[3]
    for idx, label in enumerate(labels):
        ax3.plot(time_s, left_flex if idx == 0 else left_ext, label=label, linewidth=1.2, color=colors[idx])
    ax3.set_ylabel("Activación [-]")
    ax3.set_xlabel("Tiempo [s]")
    ax3.set_title("Activaciones de la cadera izquierda")
    ax3.set_ylim(-0.05, 1.05)
    ax3.grid(True, alpha=0.25)
    ax3.legend(loc="upper right", ncol=2, fontsize=8, frameon=False)

    plt.tight_layout()
    plt.savefig(plot_path, dpi=220)
    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def _get_default_args(default_reference: str) -> dict:
    """Return a dictionary of default argument values."""
    return {
        'env': 'MIMoVelocityLowerBody-v0',
        'reference_csv': default_reference,
        'reference_phase_column': 'percent_cycle',
        'reference_angle_column': 'hip_angle_mean',
        'reference_angle_unit': 'deg',
        'trajectory_scale': 1.0,
        'reference_angle_sign': -1.0,
        'trajectory_phase_offset_pct': 0.0,
        'left_phase_offset_pct': 50.0,
        'trajectory_period_s': 1.063,
        'cycles': 5.0,
        'plot_fps': 60.0,
        'render': True,
        'seed': 7,
        'kp': 1.8,
        'ki': 0.15,
        'kd': 0.08,
        'pid_activation_scale': 1.0,
        'safe_torque_limit': 10.0,
        'mass_scale': 1.0,
        'joint_stiffness_scale': 1.0,
        'joint_damping_scale': 1.0,
        'joint_frictionloss_scale': 1.0,
        'joint_armature_scale': 1.0,
        'fmax_scale': 1.0,
        'flex_muscles': '10.0,1.2,0.25,0.8,0.0,0.30;8.0,1.2,0.18,1.4,0.8,0.25;6.0,1.2,0.15,0.5,-0.6,0.20',
        'ext_muscles': '10.0,1.2,0.25,0.8,3.14,0.30;8.0,1.2,0.18,1.4,3.94,0.25;6.0,1.2,0.15,0.5,2.54,0.20',
        'muscle_tau': 0.02,
        'lce_min': 0.75,
        'lce_max': 1.05,
        'lmin': 0.5,
        'lmax': 1.6,
        'fvmax': 1.2,
        'fpmax': 1.3,
        'run_dir': '',
        'csv_path': '',
        'plot_path': '',
        'show_plot': False,
    }


def _build_parser() -> argparse.ArgumentParser:
    default_reference = Path(__file__).resolve().parent / "healthy_hip_reference.csv"

    p = argparse.ArgumentParser(description="MIMo standup trajectory tracking simulation")
    p.add_argument("--env", default="MIMoVelocityLowerBody-v0")
    p.add_argument("--reference-csv", default=str(default_reference))
    p.add_argument("--reference-phase-column", default="percent_cycle")
    p.add_argument("--reference-angle-column", default="hip_angle_mean")
    p.add_argument("--reference-angle-unit", choices=["deg", "rad"], default="deg")
    p.add_argument("--trajectory-scale", type=float, default=1.0)
    p.add_argument("--reference-angle-sign", type=float, default=-1.0)
    p.add_argument("--trajectory-phase-offset-pct", type=float, default=0.0)
    p.add_argument("--left-phase-offset-pct", type=float, default=50.0)
    p.add_argument("--trajectory-period-s", type=float, default=2.1) # 1.063
    p.add_argument("--cycles", type=float, default=5.0)
    p.add_argument("--plot-fps", type=float, default=60.0)
    p.add_argument("--render", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--kp", type=float, default=0.9) # 1.8
    p.add_argument("--ki", type=float, default=0.0) # 0.15
    p.add_argument("--kd", type=float, default=0.0) # 0.08
    p.add_argument("--pid-activation-scale", type=float, default=1.0)
    p.add_argument("--safe-torque-limit", type=float, default=10.0)
    p.add_argument("--mass-scale", type=float, default=1.0)  # Body mass scaling factor
    p.add_argument("--joint-stiffness-scale", type=float, default=1.0)  # Joint stiffness scaling factor
    p.add_argument("--joint-damping-scale", type=float, default=1.0)  # Joint damping scaling factor
    p.add_argument("--joint-frictionloss-scale", type=float, default=1.0)  # Joint friction loss scaling factor
    p.add_argument("--joint-armature-scale", type=float, default=1.0)  # Joint armature scaling factor
    p.add_argument("--fmax-scale", type=float, default=1.0)
    p.add_argument(
        "--flex-muscles",
        default="10.0,1.2,0.25,0.8,0.0,0.30;8.0,1.2,0.18,1.4,0.8,0.25;6.0,1.2,0.15,0.5,-0.6,0.20",
        help="Flexor units as 'fmax,vmax,amp,freq,phase,bias;...'.",
    )
    p.add_argument(
        "--ext-muscles",
        default="10.0,1.2,0.25,0.8,3.14,0.30;8.0,1.2,0.18,1.4,3.94,0.25;6.0,1.2,0.15,0.5,2.54,0.20",
        help="Extensor units as 'fmax,vmax,amp,freq,phase,bias;...'.",
    )
    p.add_argument("--muscle-tau", type=float, default=0.02)
    p.add_argument("--lce-min", type=float, default=0.75)
    p.add_argument("--lce-max", type=float, default=1.05)
    p.add_argument("--lmin", type=float, default=0.5)
    p.add_argument("--lmax", type=float, default=1.6)
    p.add_argument("--fvmax", type=float, default=1.2)
    p.add_argument("--fpmax", type=float, default=1.3)
    p.add_argument("--run-dir", default="")
    p.add_argument("--csv-path", default="")
    p.add_argument("--plot-path", default="")
    p.add_argument("--show-plot", action=argparse.BooleanOptionalAction, default=False)

    return p


def _apply_mass_scale(env, factor: float) -> None:
    if abs(float(factor) - 1.0) < 1e-12:
        return
    try:
        env.model.body_mass[:] = env.model.body_mass[:] * float(factor)
    except Exception:
        pass


def _apply_earth_gravity(env) -> None:
    try:
        env.unwrapped.model.opt.gravity[:] = np.array([0.0, 0.0, -9.81])
    except Exception:
        pass


def _apply_joint_param_scales(
    env,
    *,
    stiffness_scale: float,
    damping_scale: float,
    frictionloss_scale: float,
    armature_scale: float,
) -> None:
    try:
        if abs(float(stiffness_scale) - 1.0) >= 1e-12:
            env.model.jnt_stiffness[:] = env.model.jnt_stiffness[:] * float(stiffness_scale)
        if abs(float(damping_scale) - 1.0) >= 1e-12:
            env.model.dof_damping[:] = env.model.dof_damping[:] * float(damping_scale)
        if abs(float(frictionloss_scale) - 1.0) >= 1e-12:
            env.model.jnt_frictionloss[:] = env.model.jnt_frictionloss[:] * float(frictionloss_scale)
        if abs(float(armature_scale) - 1.0) >= 1e-12:
            env.model.dof_armature[:] = env.model.dof_armature[:] * float(armature_scale)
    except Exception:
        pass


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    default_reference_path = Path(__file__).resolve().parent / "healthy_hip_reference.csv"
    defaults = _get_default_args(str(default_reference_path))
    reference = _load_reference_csv(
        csv_path=args.reference_csv,
        phase_column=args.reference_phase_column,
        angle_column=args.reference_angle_column,
        angle_unit=args.reference_angle_unit,
        scale=args.trajectory_scale,
        angle_sign=args.reference_angle_sign,
    )
    reference.period_s = float(args.trajectory_period_s)

    run_tag = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_trajectory_tracking"
    repo_root = Path(__file__).resolve().parents[2]
    run_dir = args.run_dir.strip() if args.run_dir else str(repo_root / "outputs" / "mimo" / "new_workflow" / "simulator" / run_tag)
    os.makedirs(run_dir, exist_ok=True)

    csv_path = args.csv_path.strip() if args.csv_path else os.path.join(run_dir, "hip_tracking_telemetry.csv")
    plot_path = args.plot_path.strip() if args.plot_path else os.path.join(run_dir, "hip_tracking_summary.png")

    env = gym.make(args.env, render_mode="human" if args.render else None)
    sim = env.unwrapped
    env.reset(seed=args.seed)
    _apply_earth_gravity(env)
    _apply_mass_scale(env, args.mass_scale)
    _apply_joint_param_scales(
        env,
        stiffness_scale=args.joint_stiffness_scale,
        damping_scale=args.joint_damping_scale,
        frictionloss_scale=args.joint_frictionloss_scale,
        armature_scale=args.joint_armature_scale,
    )

    if not isinstance(sim.actuation_model, MuscleModel):
        env.close()
        raise RuntimeError("Este flujo nuevo espera un entorno con MuscleModel activo.")

    actuator_ids = getattr(env, "mimo_actuators", None)
    if actuator_ids is None:
        actuator_ids = np.arange(sim.model.nu)
    actuator_names = [sim.model.actuator(i).name for i in actuator_ids]

    right_act_idx = _find_actuator_index(actuator_names, "act:right_hip_flex")
    left_act_idx = _find_actuator_index(actuator_names, "act:left_hip_flex")
    if right_act_idx is None or left_act_idx is None:
        env.close()
        raise RuntimeError("No se localizaron los actuadores de cadera esperados en el modelo.")

    right_qpos_idx = _resolve_joint_scalar(sim.model, "robot:right_hip1", qvel=False)
    left_qpos_idx = _resolve_joint_scalar(sim.model, "robot:left_hip1", qvel=False)
    right_qvel_idx = _resolve_joint_scalar(sim.model, "robot:right_hip1", qvel=True)
    left_qvel_idx = _resolve_joint_scalar(sim.model, "robot:left_hip1", qvel=True)

    # Estimate per-hip available torque to keep safe-torque-limit as a pure limiter.
    try:
        max_iso = np.asarray(sim.actuation_model.maximum_isometric_forces, dtype=np.float64)
        hip_torque_cap_right = float(np.max(np.abs(max_iso[right_act_idx])))
        hip_torque_cap_left = float(np.max(np.abs(max_iso[left_act_idx])))
    except Exception:
        hip_torque_cap_right = float(abs(sim.model.actuator_gear[right_act_idx, 0]))
        hip_torque_cap_left = float(abs(sim.model.actuator_gear[left_act_idx, 0]))

    hip_torque_cap_right = max(1e-9, hip_torque_cap_right)
    hip_torque_cap_left = max(1e-9, hip_torque_cap_left)

    flex_units = _parse_muscle_units(args.flex_muscles)
    ext_units = _parse_muscle_units(args.ext_muscles)
    flex_units = _scale_muscle_units(flex_units, args.fmax_scale)
    ext_units = _scale_muscle_units(ext_units, args.fmax_scale)
    hip_params = HipMuscleParams(
        tau=float(args.muscle_tau),
        lce_min=float(args.lce_min),
        lce_max=float(args.lce_max),
        lmin=float(args.lmin),
        lmax=float(args.lmax),
        fvmax=float(args.fvmax),
        fpmax=float(args.fpmax),
    )
    right_controller = HipMuscleController(flexors=flex_units, extensors=ext_units, params=hip_params)
    left_controller = HipMuscleController(flexors=flex_units, extensors=ext_units, params=hip_params)

    sim_dt = float(getattr(sim, "dt", 1.0 / 60.0))
    total_steps = max(1, int(math.ceil(float(args.cycles) * float(reference.period_s) / sim_dt)))

    # Get current hip positions and calculate initial phases that match them
    q_right_init = float(sim.data.qpos[right_qpos_idx])
    q_left_init = float(sim.data.qpos[left_qpos_idx])
    initial_phase_right = _find_initial_phase(reference, q_right_init)
    left_phase_offset_pct = float(args.left_phase_offset_pct)
    left_start_delay_phase = _find_phase_delay_for_position(
        reference,
        q_left_init,
        initial_phase_right + left_phase_offset_pct,
    )
    left_start_delay_s = left_start_delay_phase * reference.period_s / 100.0

    print(f"Referencia cargada: {reference.phase_pct.size} puntos, periodo={reference.period_s:.3f}s")
    print(f"Simulación: {total_steps} pasos, dt={sim_dt:.6f}s")
    print(f"Actuadores relevantes: derecha={actuator_names[right_act_idx]} izquierda={actuator_names[left_act_idx]}")
    print(f"Fases iniciales: derecha={initial_phase_right:.1f}%, izquierda=desfasada {left_phase_offset_pct:.1f}%")
    print(f"Retardo inicial izquierda: {left_start_delay_s:.3f}s ({left_start_delay_phase:.1f}%)")
    print(f"Posiciones iniciales: derecha={q_right_init:.4f} rad, izquierda={q_left_init:.4f} rad")

    rows: list[HipTelemetryRow] = []
    right_prev_flex = np.zeros((len(flex_units),), dtype=np.float64)
    right_prev_ext = np.zeros((len(ext_units),), dtype=np.float64)
    left_prev_flex = np.zeros((len(flex_units),), dtype=np.float64)
    left_prev_ext = np.zeros((len(ext_units),), dtype=np.float64)
    int_err_right = 0.0
    int_err_left = 0.0

    action = np.zeros(env.action_space.shape, dtype=np.float32)
    half_action = action.shape[0] // 2 if action.shape[0] % 2 == 0 else None

    for step in range(total_steps):
        t_s = step * sim_dt
        phase_right = (initial_phase_right + 100.0 * t_s / reference.period_s) % 100.0
        phase_left = (phase_right + left_phase_offset_pct) % 100.0

        target_right = _reference_at_phase(reference, phase_right)
        target_left = q_left_init if t_s < left_start_delay_s else _reference_at_phase(reference, phase_left)

        q_right = float(sim.data.qpos[right_qpos_idx])
        q_left = float(sim.data.qpos[left_qpos_idx])
        qd_right = float(sim.data.qvel[right_qvel_idx])
        qd_left = float(sim.data.qvel[left_qvel_idx])

        # Position error for both hips
        err_right = q_right - target_right
        err_left = q_left - target_left

        # Integrate error
        int_err_right += err_right * sim_dt
        int_err_left += err_left * sim_dt

        # PID controller
        pid_r = args.kp * err_right + args.ki * int_err_right + args.kd * (0.0 - qd_right)
        pid_l = args.kp * err_left + args.ki * int_err_left + args.kd * (0.0 - qd_left)

        # Convert PID output into normalized muscle command [-1, 1].
        # This gain is explicit and independent of safe-torque-limit.
        pid_act_scale = max(1e-9, float(args.pid_activation_scale))
        act_r_norm = float(np.clip(pid_r / pid_act_scale, -1.0, 1.0))
        act_l_norm = float(np.clip(pid_l / pid_act_scale, -1.0, 1.0))

        # Apply safe torque as a pure clamp in equivalent torque domain.
        safe_limit = max(1e-9, float(args.safe_torque_limit))
        act_cap_r = min(1.0, safe_limit / hip_torque_cap_right)
        act_cap_l = min(1.0, safe_limit / hip_torque_cap_left)
        act_r_norm = float(np.clip(act_r_norm, -act_cap_r, act_cap_r))
        act_l_norm = float(np.clip(act_l_norm, -act_cap_l, act_cap_l))

        # Commanded equivalent torque proxy for telemetry.
        tau_right_cmd = act_r_norm * hip_torque_cap_right
        tau_left_cmd = act_l_norm * hip_torque_cap_left

        # Split into flexor/extensor activations in [0,1]
        # If the signal is positive, we activate flexors; if negative, we activate extensors
        right_flex_cmd = max(0.0, act_r_norm)
        right_ext_cmd = max(0.0, -act_r_norm)
        left_flex_cmd = max(0.0, act_l_norm)
        left_ext_cmd = max(0.0, -act_l_norm)

        action[:] = 0.0
        if half_action is not None:
            if right_act_idx < half_action:
                action[right_act_idx] = float(
                    np.clip(right_flex_cmd, env.action_space.low[right_act_idx], env.action_space.high[right_act_idx])
                )
                action[right_act_idx + half_action] = float(
                    np.clip(right_ext_cmd, env.action_space.low[right_act_idx + half_action], env.action_space.high[right_act_idx + half_action])
                )
            if left_act_idx < half_action:
                action[left_act_idx] = float(
                    np.clip(left_flex_cmd, env.action_space.low[left_act_idx], env.action_space.high[left_act_idx])
                )
                action[left_act_idx + half_action] = float(
                    np.clip(left_ext_cmd, env.action_space.low[left_act_idx + half_action], env.action_space.high[left_act_idx + half_action])
                )
        else:
            if right_act_idx < action.shape[0]:
                action[right_act_idx] = float(
                    np.clip(-(right_flex_cmd - right_ext_cmd), env.action_space.low[right_act_idx], env.action_space.high[right_act_idx])
                )
            if left_act_idx < action.shape[0]:
                action[left_act_idx] = float(
                    np.clip(-(left_flex_cmd - left_ext_cmd), env.action_space.low[left_act_idx], env.action_space.high[left_act_idx])
                )

        obs, reward, terminated, truncated, info = env.step(action)
        if args.render:
            env.render()

        # Read the actual torque applied by the MuscleModel (actuator_gear * control_input)
        try:
            applied_torques = sim.actuation_model.simulation_torque()
            applied_torque_right = float(applied_torques[right_act_idx])
            applied_torque_left = float(applied_torques[left_act_idx])
        except Exception:
            # Fallback to the PID proxy if muscle model data is unavailable
            applied_torque_right = float(tau_right_cmd)
            applied_torque_left = float(tau_left_cmd)

        rows.append(
            HipTelemetryRow(
                time_s=t_s,
                target_right_rad=target_right,
                actual_right_rad=float(sim.data.qpos[right_qpos_idx]),
                torque_right_nm=applied_torque_right,
                right_flex_activation=right_flex_cmd,
                right_ext_activation=right_ext_cmd,
                target_left_rad=target_left,
                actual_left_rad=float(sim.data.qpos[left_qpos_idx]),
                torque_left_nm=applied_torque_left,
                left_flex_activation=left_flex_cmd,
                left_ext_activation=left_ext_cmd,
            )
        )

        if step < 2 or step % 100 == 0:
            print(
                "step={} q_r={:+.3f} q_l={:+.3f} t_r={:+.3f} t_l={:+.3f} tau_r={:+.3f} tau_l={:+.3f}".format(
                    step,
                    q_right,
                    q_left,
                    target_right,
                    target_left,
                    tau_right_cmd,
                    tau_left_cmd,
                )
            )

        if terminated or truncated:
            obs, info = env.reset()
            _apply_earth_gravity(env)

    env.close()

    _save_csv(csv_path, rows)

    time_s = np.asarray([row.time_s for row in rows], dtype=np.float64)
    target_right = np.asarray([row.target_right_rad for row in rows], dtype=np.float64)
    actual_right = np.asarray([row.actual_right_rad for row in rows], dtype=np.float64)
    torque_right = np.asarray([row.torque_right_nm for row in rows], dtype=np.float64)
    right_flex = np.asarray([row.right_flex_activation for row in rows], dtype=np.float64)
    right_ext = np.asarray([row.right_ext_activation for row in rows], dtype=np.float64)
    target_left = np.asarray([row.target_left_rad for row in rows], dtype=np.float64)
    actual_left = np.asarray([row.actual_left_rad for row in rows], dtype=np.float64)
    torque_left = np.asarray([row.torque_left_nm for row in rows], dtype=np.float64)
    left_flex = np.asarray([row.left_flex_activation for row in rows], dtype=np.float64)
    left_ext = np.asarray([row.left_ext_activation for row in rows], dtype=np.float64)

    _plot_run(
        plot_path=plot_path,
        show_plot=bool(args.show_plot),
        time_s=time_s,
        target_right=target_right,
        actual_right=actual_right,
        torque_right=torque_right,
        right_flex=right_flex,
        right_ext=right_ext,
        target_left=target_left,
        actual_left=actual_left,
        torque_left=torque_left,
        left_flex=left_flex,
        left_ext=left_ext,
    )

    # Write combined configuration and summary file
    summary_path = os.path.join(run_dir, "simulation_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("=" * 80 + "\n")
        f.write("MIMo TRAJECTORY TRACKING SIMULATION - SUMMARY\n")
        f.write("=" * 80 + "\n\n")
        
        f.write("EXECUTION INFO\n")
        f.write("-" * 80 + "\n")
        f.write(f"timestamp: {dt.datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"command: {' '.join(sys.argv)}\n")
        f.write(f"run_dir: {run_dir}\n\n")
        
        f.write("ENVIRONMENT SETUP\n")
        f.write("-" * 80 + "\n")
        f.write(f"env_id: {args.env}\n")
        f.write(f"reference_csv: {args.reference_csv}\n")
        f.write(f"reference_period_s: {reference.period_s}\n")
        f.write(f"reference_points: {reference.phase_pct.size}\n\n")
        
        f.write("TRAJECTORY CONFIGURATION\n")
        f.write("-" * 80 + "\n")
        f.write(f"trajectory_phase_offset_pct: {args.trajectory_phase_offset_pct}\n")
        f.write(f"left_phase_offset_pct: {args.left_phase_offset_pct}\n")
        f.write(f"target_cycles: {args.cycles}\n")
        f.write(f"initial_phase_right: {initial_phase_right:.1f}%\n")
        f.write(f"left_start_delay_s: {left_start_delay_s:.3f}s\n\n")
        
        f.write("CONTROL PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write(f"kp: {args.kp}\n")
        f.write(f"ki: {args.ki}\n")
        f.write(f"kd: {args.kd}\n")
        f.write(f"safe_torque_limit: {args.safe_torque_limit}\n\n")
        
        f.write("MODEL PARAMETERS (SCALING FACTORS)\n")
        f.write("-" * 80 + "\n")
        f.write(f"mass_scale: {args.mass_scale}\n")
        f.write(f"joint_stiffness_scale: {args.joint_stiffness_scale}\n")
        f.write(f"joint_damping_scale: {args.joint_damping_scale}\n")
        f.write(f"joint_frictionloss_scale: {args.joint_frictionloss_scale}\n")
        f.write(f"joint_armature_scale: {args.joint_armature_scale}\n\n")
        
        f.write("SIMULATION RESULTS\n")
        f.write("-" * 80 + "\n")
        f.write(f"n_steps: {len(rows)}\n")
        f.write(f"sim_dt: {sim_dt:.9f}s\n")
        f.write(f"duration_s: {len(rows) * sim_dt:.6f}s\n")
        f.write(f"csv_path: {csv_path}\n")
        f.write(f"plot_path: {plot_path}\n\n")
        
        f.write("MUSCLE MODEL PARAMETERS\n")
        f.write("-" * 80 + "\n")
        f.write(f"muscle_tau: {args.muscle_tau}\n")
        f.write(f"lce_min: {args.lce_min}\n")
        f.write(f"lce_max: {args.lce_max}\n")
        f.write(f"lmin: {args.lmin}\n")
        f.write(f"lmax: {args.lmax}\n")
        f.write(f"fvmax: {args.fvmax}\n")
        f.write(f"fpmax: {args.fpmax}\n")
        f.write(f"fmax_scale: {args.fmax_scale}\n\n")
        
        f.write("ACTUATOR NAMES\n")
        f.write("-" * 80 + "\n")
        for i, name in enumerate(actuator_names):
            f.write(f"{i}: {name}\n")
        f.write("\n")
        
        f.write("ALL ARGUMENTS\n")
        f.write("-" * 80 + "\n")
        for key in sorted(vars(args).keys()):
            value = getattr(args, key)
            default_value = defaults.get(key)
            if default_value is None:
                f.write(f"{key}: {value}\n")
            elif value == default_value:
                f.write(f"{key}: {value}          (default)\n")
            else:
                f.write(f"{key}: {value}          (default: {default_value})\n")

    print(f"CSV guardado en {csv_path}")
    print(f"Imagen guardada en {plot_path}")
    print(f"Resumen guardado en {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())