# This script contains the functions to compute the Muscle Trajectory following (MTF) project
# for the MimoGym environments based on the publication of:
#  MIMo: A Multimodal Infant Model for Studying Cognitive Development. 
# Mattern, Dominik and Schumacher, Pierre and Lopez, Francisco M and Raabe, Marcel C and Ernst, Markus R and Aubret, Arthur and Triesch, Jochen
# https://arxiv.org/abs/2211.13227

from __future__ import annotations

import os, sys
import argparse
import csv
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple
import numpy as np
from pyparsing import Optional

@dataclass
class ReferenceTrajectory:
    phase_pct: np.ndarray
    angle_rad: np.ndarray
    period_s: float


def _build_parser() -> argparse.ArgumentParser:
    default_reference = Path(__file__).resolve().parent / "healthy_hip_reference.csv"

    p = argparse.ArgumentParser(description="MIMo standup trajectory tracking simulation")
    p.add_argument("--env", default="MIMoVelocityLowerBody-v0")
    p.add_argument("--reference-csv", default=str(default_reference))
    p.add_argument("--reference-phase-column", default="percent_cycle")
    p.add_argument("--reference-angle-column", default="hip_angle_mean")
    p.add_argument("--reference-angle-unit", choices=["deg", "rad"], default="deg")
    p.add_argument("--trajectory-scale", type=float, default=1.0)
    p.add_argument("--reference-angle-sign", type=float, default=1.0)
    p.add_argument("--trajectory-phase-offset-pct", type=float, default=0.0)
    p.add_argument("--left-phase-offset-pct", type=float, default=50.0)
    p.add_argument("--trajectory-period-s", type=float, default=2.1) # 1.063
    p.add_argument("--cycles", type=float, default=5.0)
    p.add_argument("--plot-fps", type=float, default=60.0)
    p.add_argument("--render", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--kp", type=float, default=2) # 1.8
    p.add_argument("--ki", type=float, default=2) # 0.15
    p.add_argument("--kd", type=float, default=0.15) # 0.08
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
        'kp': 10,
        'ki': 0.5,
        'kd': 0.0,
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


def _reference_at_phase(traj, phase_pct: float) -> float:
    phase3 = np.concatenate([traj.phase_pct - 100.0, traj.phase_pct, traj.phase_pct + 100.0])
    angle3 = np.concatenate([traj.angle_rad, traj.angle_rad, traj.angle_rad])
    return float(np.interp(phase_pct % 100.0, phase3, angle3))

def _find_initial_phase(traj, current_pos: float) -> float:
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

def _find_phase_delay_for_position(traj: ReferenceTrajectory, current_pos: float, start_phase_pct: float) -> float:
    phases = np.linspace(0.0, 100.0, 1000)
    angles = np.array([_reference_at_phase(traj, start_phase_pct + p) for p in phases])
    idx_closest = np.argmin(np.abs(angles - current_pos))
    return float(phases[idx_closest])


def _find_actuator_index(actuator_names: Sequence[str], target_name: str) -> Optional[int]:
    try:
        return actuator_names.index(target_name)
    except ValueError:
        return None
    
"""
def _apply_joint_param_scales(
    model_file: str,
    *,
    stiffness_scale: float,
    damping_scale: float,
    frictionloss_scale: float,
    armature_scale: float,
) -> None:
    
    model_hfd = model_file + "/models"
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
"""