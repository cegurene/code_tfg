#!/usr/bin/env python3 python hip_motor_torque_replay_runner_pycandle.py --source-csv /ruta/al/hip_tracking_telemetry.csv --motor-id 11 --apply-torque
from __future__ import annotations

import argparse
import csv
import datetime as dt
import importlib.util
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _load_module_from_path(module_name: str, relative_file: str):
    file_path = Path(__file__).resolve().parent / relative_file
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"No se pudo cargar {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Ros2MotorInterface:
    """ROS2-based motor interface for real hardware control."""

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
        self.node = self._node_cls("mimo_hip_torque_replay_runner")

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


@dataclass
class SourceReplay:
    time_s: np.ndarray
    q_des_rad: np.ndarray
    torque_nm: np.ndarray


def _parse_float(text: str) -> float:
    return float(str(text).strip().replace("\ufeff", "").replace(",", "."))


def _resolve_column(columns: Sequence[str], explicit: str, candidates: Sequence[str], kind: str) -> str:
    lowered = {c.strip().lower(): c for c in columns}
    if explicit:
        key = explicit.strip().lower()
        if key not in lowered:
            raise ValueError(f"No se encontró la columna '{explicit}' para {kind} en {list(columns)}")
        return lowered[key]
    for candidate in candidates:
        if candidate in lowered:
            return lowered[candidate]
    raise ValueError(f"No se pudo inferir la columna de {kind} en {list(columns)}")


def _load_source_csv(csv_path: str, source_hip: str) -> SourceReplay:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"No existe el CSV fuente: {csv_path}")

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"El CSV fuente '{csv_path}' no tiene cabecera.")

        time_col = _resolve_column(reader.fieldnames, "time_s", ["time_s", "t", "time"], "tiempo")
        q_col = _resolve_column(
            reader.fieldnames,
            f"target_{source_hip}_rad",
            [f"target_{source_hip}_rad", f"{source_hip}_target_rad", f"q_des_{source_hip}_rad"],
            "posición objetivo",
        )
        tau_col = _resolve_column(
            reader.fieldnames,
            f"torque_{source_hip}_nm",
            [f"torque_{source_hip}_nm", f"tau_{source_hip}_nm", f"{source_hip}_torque_nm"],
            "torque",
        )

        time_values = []
        q_values = []
        tau_values = []
        for row in reader:
            time_values.append(_parse_float(row[time_col]))
            q_values.append(_parse_float(row[q_col]))
            tau_values.append(_parse_float(row[tau_col]))

    time_arr = np.asarray(time_values, dtype=np.float64)
    q_arr = np.asarray(q_values, dtype=np.float64)
    tau_arr = np.asarray(tau_values, dtype=np.float64)

    order = np.argsort(time_arr)
    time_arr = time_arr[order]
    q_arr = q_arr[order]
    tau_arr = tau_arr[order]

    time_arr = time_arr - float(time_arr[0])
    return SourceReplay(time_s=time_arr, q_des_rad=q_arr, torque_nm=tau_arr)


def _sample_source(source: SourceReplay, t_s: float) -> Tuple[float, float]:
    t = float(np.clip(t_s, float(source.time_s[0]), float(source.time_s[-1])))
    q_des = float(np.interp(t, source.time_s, source.q_des_rad))
    tau = float(np.interp(t, source.time_s, source.torque_nm))
    return q_des, tau


def _write_run_config_txt(run_dir: str, args: argparse.Namespace, source: SourceReplay) -> str:
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

    lines.extend(
        [
            "",
            f"source_duration_s: {float(source.time_s[-1]):.6f}",
            f"source_samples: {int(source.time_s.size)}", 
            f"source_hip: {args.source_hip}",
        ]
    )

    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return config_path


def _save_csv(csv_path: str, time_s: np.ndarray, q: np.ndarray, qd: np.ndarray, q_des: np.ndarray, torque: np.ndarray) -> None:
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "q_rad", "qd_rad_s", "q_des_rad", "applied_torque_nm"])
        for i in range(time_s.size):
            writer.writerow([float(time_s[i]), float(q[i]), float(qd[i]), float(q_des[i]), float(torque[i])])


def _plot_run(plot_path: str, show_plot: bool, time_s: np.ndarray, q: np.ndarray, q_des: np.ndarray, torque: np.ndarray) -> None:
    os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(13, 7.2), sharex=True)

    axes[0].plot(time_s, q_des, color="#d62728", linewidth=1.8, label="Objetivo")
    axes[0].plot(time_s, q, color="#1f5c9a", linewidth=2.0, label="Real")
    axes[0].set_ylabel("Ángulo [rad]")
    axes[0].set_title("Motor real: seguimiento de la referencia generada en simulación")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best", frameon=False)

    axes[1].plot(time_s, torque, color="#2b8a3e", linewidth=1.8, label="Torque aplicado")
    axes[1].set_ylabel("Torque [Nm]")
    axes[1].set_xlabel("Tiempo [s]")
    axes[1].set_title("Torque aplicado al motor")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best", frameon=False)

    plt.tight_layout()
    plt.savefig(plot_path, dpi=220)
    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def _load_backend(interface: str):
    if interface == "ros2":
        return Ros2MotorInterface

    if interface == "pycandle":
        wrapper = _load_module_from_path("mimo_hip_motor_torque_replay_runner_pycandle", "hip_motor_torque_replay_runner_pycandle.py")
        return wrapper.PyCandleTorqueReplayMotorInterface

    raise ValueError(f"Interfaz no soportada: {interface}")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Replay runner for hip torque exported by the simulation")
    p.add_argument("--source-csv", default="", help="CSV generado por la simulación que contiene la referencia y el torque.")
    p.add_argument("--source-hip", choices=["right", "left"], default="right")
    p.add_argument("--interface", choices=["ros2", "pycandle"], default="ros2")
    p.add_argument("--motor-id", type=int, default=None, help="Motor ID; se pedirá si no se pasa en modo real.")
    p.add_argument("--joint-state-topic", default="/md80/joint_states")
    p.add_argument("--command-topic", default="/md80/motion_command")
    p.add_argument("--rate-hz", type=float, default=100.0)
    p.add_argument("--apply-torque", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--safe-torque-limit", type=float, default=None)
    p.add_argument("--save-csv", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--save-plot", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--show-plot", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--run-dir", default="")
    p.add_argument("--csv-path", default="")
    p.add_argument("--plot-path", default="")
    p.add_argument("--verbose", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-baud", default="1M")
    p.add_argument("--pycandle-fdcan", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-max-torque", type=float, default=1.0)
    p.add_argument("--pycandle-add-all-drives", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-strict-id-match", action=argparse.BooleanOptionalAction, default=False)
    return p


def main() -> int:
    args = _build_parser().parse_args()

    if not args.source_csv.strip():
        raise ValueError("--source-csv es obligatorio para este runner de replay.")

    source = _load_source_csv(args.source_csv.strip(), args.source_hip)

    if args.motor_id is None:
        try:
            motor_id_str = input("Enter motor ID (default=308): ").strip()
            args.motor_id = int(motor_id_str) if motor_id_str else 308
        except ValueError:
            print("Motor ID inválido. Usando 308.")
            args.motor_id = 308

    run_tag = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + f"_id={args.motor_id}"
    repo_root = Path(__file__).resolve().parents[2]
    run_dir = args.run_dir.strip() if args.run_dir else str(repo_root / "outputs" / "mimo" / "new_workflow" / "runner" / run_tag)
    os.makedirs(run_dir, exist_ok=True)

    csv_path = args.csv_path.strip() if args.csv_path else os.path.join(run_dir, "telemetry.csv")
    plot_path = args.plot_path.strip() if args.plot_path else os.path.join(run_dir, "timeseries.png")

    backend_cls = _load_backend(args.interface)
    if args.interface == "pycandle":
        backend_cls.configure_from_args(args)

    interface = backend_cls(
        motor_id=args.motor_id,
        joint_state_topic=args.joint_state_topic,
        command_topic=args.command_topic,
        verbose=bool(args.verbose),
    )

    if not interface.wait_for_state(timeout_s=5.0):
        interface.close()
        raise RuntimeError("No se recibió estado del motor. Revisa el bus y los topics.")

    initial_pos = float(interface.current_pos if interface.current_pos is not None else 0.0)
    print(f"Posición inicial: {initial_pos:+.6f} rad")
    print(f"Replay de torque desde {args.source_csv} usando la cadera {args.source_hip}")

    try:
        config_path = _write_run_config_txt(run_dir=run_dir, args=args, source=source)
        print(f"Configuración guardada en {config_path}")
    except Exception as exc:
        print(f"No se pudo guardar run_config.txt: {exc}")

    dt_s = 1.0 / max(1e-3, float(args.rate_hz))
    t0 = time.time()
    next_tick = t0

    t_hist = []
    q_hist = []
    qd_hist = []
    q_des_hist = []
    torque_hist = []

    try:
        while True:
            now = time.time()
            t_rel = now - t0
            if t_rel > float(source.time_s[-1]):
                break

            interface.spin_once(timeout_s=0.0)
            q_des, torque = _sample_source(source, t_rel)
            if args.safe_torque_limit is not None:
                torque = float(np.clip(torque, -float(args.safe_torque_limit), float(args.safe_torque_limit)))

            q = float(interface.current_pos if interface.current_pos is not None else 0.0)
            qd = float(interface.current_vel if interface.current_vel is not None else 0.0)

            command_torque = float(torque) if args.apply_torque else 0.0
            interface.publish_torque(command_torque)

            t_hist.append(t_rel)
            q_hist.append(q)
            qd_hist.append(qd)
            q_des_hist.append(q_des)
            torque_hist.append(command_torque)

            if len(t_hist) < 5 or len(t_hist) % 100 == 0:
                print(
                    "t={:.3f}s q={:+.3f} q_des={:+.3f} torque={:+.3f}".format(
                        t_rel,
                        q,
                        q_des,
                        command_torque,
                    )
                )

            next_tick += dt_s
            sleep_s = next_tick - time.time()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("Replay interrumpido por el usuario")
    finally:
        try:
            interface.publish_torque(0.0)
        except Exception:
            pass
        interface.close()

    t_arr = np.asarray(t_hist, dtype=np.float64)
    q_arr = np.asarray(q_hist, dtype=np.float64)
    qd_arr = np.asarray(qd_hist, dtype=np.float64)
    q_des_arr = np.asarray(q_des_hist, dtype=np.float64)
    torque_arr = np.asarray(torque_hist, dtype=np.float64)

    if args.save_csv:
        _save_csv(csv_path, t_arr, q_arr, qd_arr, q_des_arr, torque_arr)
        print(f"CSV guardado en {csv_path}")

    if args.save_plot:
        _plot_run(plot_path, bool(args.show_plot), t_arr, q_arr, q_des_arr, torque_arr)
        print(f"Imagen guardada en {plot_path}")

    meta_path = os.path.join(run_dir, "simulation_info.txt")
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write(f"run_dir: {run_dir}\n")
        f.write(f"source_csv: {args.source_csv}\n")
        f.write(f"source_hip: {args.source_hip}\n")
        f.write(f"interface: {args.interface}\n")
        f.write(f"motor_id: {args.motor_id}\n")
        f.write(f"rate_hz: {args.rate_hz}\n")
        f.write(f"apply_torque: {args.apply_torque}\n")
        f.write(f"safe_torque_limit: {args.safe_torque_limit}\n")
        f.write(f"n_samples: {len(t_arr)}\n")
        if len(t_arr) > 0:
            f.write(f"duration_s: {float(t_arr[-1]):.6f}\n")
        f.write("csv_columns: time_s, q_rad, qd_rad_s, q_des_rad, applied_torque_nm\n")
        f.write("plot_top: q_des_rad vs q_rad\n")
        f.write("plot_bottom: applied_torque_nm\n")
        f.write(f"csv_path: {csv_path}\n")
        f.write(f"plot_path: {plot_path}\n")

    print(f"Metadatos guardados en {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())