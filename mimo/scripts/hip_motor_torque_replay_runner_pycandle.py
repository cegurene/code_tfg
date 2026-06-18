#!/usr/bin/env python3
"""Replay runner for hip torque exported by the simulation using pyCandle.

This variant reuses the new replay pipeline and swaps only the real hardware
interface layer to communicate directly with pyCandle MD80 commands.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import sys
import time
import faulthandler
from pathlib import Path

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
_base_path = os.path.join(here, "hip_motor_torque_replay_runner.py")
base = None

# Habilitar faulthandler de inmediato para trazar posibles fallos
faulthandler.enable()


def _find_open_serial_devices() -> list[str]:
    device_prefixes = ("/dev/ttyACM", "/dev/ttyUSB")
    current_pid = os.getpid()
    holders = []
    proc_root = Path("/proc")
    for proc_dir in proc_root.iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        if pid == current_pid:
            continue
        fd_dir = proc_dir / "fd"
        try:
            fds = list(fd_dir.iterdir())
        except (FileNotFoundError, PermissionError):
            continue
        seen_devices = set()
        for fd in fds:
            try:
                target = os.readlink(fd)
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if target.startswith(device_prefixes):
                seen_devices.add(target)
        if not seen_devices:
            continue
        try:
            cmdline = (proc_dir / "cmdline").read_text(encoding="utf-8", errors="replace").replace("\x00", " ").strip()
        except (FileNotFoundError, PermissionError, OSError):
            cmdline = ""
        holders.append(f"pid={pid} devices={sorted(seen_devices)} cmd='{cmdline or '?'}'")
    return holders


def _load_base_module():
    global base
    if base is not None:
        return base

    base_spec = importlib.util.spec_from_file_location("mimo_hip_motor_torque_replay_runner_base", _base_path)
    if base_spec is None or base_spec.loader is None:
        raise RuntimeError(f"No se pudo cargar el runner base desde {_base_path}")
    base = importlib.util.module_from_spec(base_spec)
    sys.modules[base_spec.name] = base
    base_spec.loader.exec_module(base)
    return base


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Replay runner for hip torque exported by the simulation")
    p.add_argument("--source-csv", default="", help="CSV generado por la simulación que contiene la referencia y el torque.")
    p.add_argument("--source-hip", choices=["right", "left"], default="right")
    p.add_argument("--interface", choices=["pycandle"], default="pycandle", help="Compatibilidad con la interfaz real pyCandle.")
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


class PyCandleTorqueReplayMotorInterface:
    """Drop-in replacement for Ros2MotorInterface using pyCandle RAW_TORQUE mode."""

    _baud_label: str = "1M"
    _fdcan_enabled: bool = True
    _max_torque_nm = 1.0
    _add_all_drives: bool = True
    _strict_id_match: bool = False

    @classmethod
    def configure_from_args(cls, args) -> None:
        cls._baud_label = str(args.pycandle_baud)
        cls._fdcan_enabled = bool(args.pycandle_fdcan)
        cls._max_torque_nm = float(args.pycandle_max_torque) if args.pycandle_max_torque is not None else None
        cls._add_all_drives = bool(args.pycandle_add_all_drives)
        cls._strict_id_match = bool(args.pycandle_strict_id_match)

    def __init__(self, joint_state_topic: str, command_topic: str, verbose: bool) -> None:
        del joint_state_topic
        del command_topic

        try:
            pyCandle = importlib.import_module("pyCandle")
        except Exception as exc:
            raise RuntimeError("pyCandle no disponible. Instala/activa pyCandle antes de usar real mode.") from exc

        self._pycandle = pyCandle
        self._verbose = bool(verbose)
        self.motor_ids = [348, 349]

        baud = self._resolve_baud_constant(self._baud_label)

        holders = _find_open_serial_devices()
        if holders:
            raise RuntimeError(
                "Hay otro proceso usando un dispositivo serie CANdle. "
                "Cierra primero candle_ros2_node/otros runners y vuelve a probar:\n  "
                + "\n  ".join(holders)
            )
        self.candle = pyCandle.Candle(baud, self._fdcan_enabled)

        ids = list(self.candle.ping())
        if not ids:
            raise RuntimeError("No se detectaron drives en el bus CAN (pyCandle ping vacio).")

        ids_to_add = ids if self._add_all_drives else self.motor_ids
        for drive_id in ids_to_add:
            if drive_id in ids:
                self.candle.addMd80(drive_id)

        if not getattr(self.candle, "md80s", None):
            raise RuntimeError("No se pudo inicializar ningun MD80 en pyCandle.")

        self.drives = {}
        for mid in self.motor_ids:
            drive = self._select_drive(mid, ids)
            self.drives[mid] = drive
            self.candle.controlMd80Mode(drive, pyCandle.RAW_TORQUE)
            self.candle.controlMd80Enable(drive, True)
            if self._max_torque_nm is not None:
                drive.setMaxTorque(float(self._max_torque_nm))

        self.candle.begin()

        self.current_pos = {mid: None for mid in self.motor_ids}
        self.current_vel = {mid: 0.0 for mid in self.motor_ids}
        self.current_effort = {mid: 0.0 for mid in self.motor_ids}
        self.last_state_ts = 0.0

        self._poll_state()

        if self._verbose:
            print(
                "pyCandle replay interface ready: motor_ids={} baud={} fdcan={} max_torque={}Nm".format(
                    self.motor_ids,
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
    def _get_drive_id(drive):
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

    def _select_drive(self, motor_id: int, ping_ids):
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
            for mid in self.motor_ids:
                drive = self.drives[mid]
                self.current_pos[mid] = float(drive.getPosition())
                self.current_vel[mid] = float(drive.getVelocity())
                self.current_effort[mid] = float(drive.getTorque())
            self.last_state_ts = time.time()
        except Exception:
            pass

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._poll_state()
        if timeout_s > 0.0:
            time.sleep(min(float(timeout_s), 0.002))

    def wait_for_state(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 <= timeout_s:
            self.spin_once(timeout_s=0.01)
            if all(v is not None for v in self.current_pos.values()):
                return True
        return False

    def publish_torque(self, torque_l: float, torque_r: float) -> None:
        if 348 in self.drives:
            self.drives[348].setTargetTorque(float(torque_l))
        if 349 in self.drives:
            self.drives[349].setTargetTorque(float(torque_r))

    def close(self) -> None:
        try:
            self.publish_torque(0.0, 0.0)
            time.sleep(0.02)
        except Exception:
            pass
        try:
            self.candle.end()
        except Exception:
            pass
        for drive in self.drives.values():
            try:
                self.candle.controlMd80Enable(drive, False)
            except Exception:
                pass

def main() -> int:
    args = _build_parser().parse_args()

    if not args.source_csv.strip():
        raise ValueError("--source-csv es obligatorio para este runner de replay.")

    PyCandleTorqueReplayMotorInterface.configure_from_args(args)

    interface = PyCandleTorqueReplayMotorInterface(
        joint_state_topic=args.joint_state_topic,
        command_topic=args.command_topic,
        verbose=bool(args.verbose),
    )

    if not interface.wait_for_state(timeout_s=10.0):
        interface.close()
        raise RuntimeError("No se recibió estado de los motores 348/349. Revisa el bus.")

    base_mod = _load_base_module()
    source = base_mod._load_source_csv(args.source_csv.strip())

    run_tag = base_mod.dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_dual_pycandle_replay"
    repo_root = Path(__file__).resolve().parents[2]
    run_dir = args.run_dir.strip() if args.run_dir else str(repo_root / "outputs" / "mimo" / "new_workflow" /"runner" / run_tag)
    os.makedirs(run_dir, exist_ok=True)

    csv_path = args.csv_path.strip() if args.csv_path else os.path.join(run_dir, "telemetry.csv")
    plot_path = args.plot_path.strip() if args.plot_path else os.path.join(run_dir, "timeseries.png")
    
    print("Motores listos. Iniciando replay de torque dual con pyCandle.")
    print(f"Replay de torque desde {args.source_csv} usando la cadera {args.source_hip}")

    try:
        config_path = base_mod._write_run_config_txt(run_dir=run_dir, args=args, source=source)
        print(f"Configuración guardada en {config_path}")
    except Exception as exc:
        print(f"No se pudo guardar run_config.txt: {exc}")

    dt_s = 1.0 / max(1e-3, float(args.rate_hz))
    t0 = time.time()
    next_tick = t0

    t_hist = []
    ql_hist = []
    qr_hist = []
    qdl_hist = []
    qdr_hist = []
    ql_des_hist = []
    qr_des_hist = []
    taul_hist = []
    taur_hist = []

    try:
        while True:
            now = time.time()
            t_rel = now - t0
            if t_rel > float(source.time_s[-1]):
                break

            interface.spin_once(timeout_s=0.0)
            q_des_l, q_des_r, tau_l, tau_r = base_mod._sample_source(source, t_rel)
            
            if args.safe_torque_limit is not None:
                tau_l = float(np.clip(tau_l, -float(args.safe_torque_limit), float(args.safe_torque_limit)))
                tau_r = float(np.clip(tau_r, -float(args.safe_torque_limit), float(args.safe_torque_limit)))

            ql = float(interface.current_pos[348] if interface.current_pos[348] is not None else 0.0)
            qr = float(interface.current_pos[349] if interface.current_pos[349] is not None else 0.0)
            qdl = float(interface.current_vel[348])
            qdr = float(interface.current_vel[349])

            command_torque_l = float(tau_l) if args.apply_torque else 0.0
            # Inversión de polaridad para el motor derecho (ID 349): Positive torque = forward
            command_torque_r = float(-tau_r) if args.apply_torque else 0.0
            
            interface.publish_torque(command_torque_l, command_torque_r)

            t_hist.append(t_rel)
            ql_hist.append(ql); qr_hist.append(qr)
            qdl_hist.append(qdl); qdr_hist.append(qdr)
            ql_des_hist.append(q_des_l); qr_des_hist.append(q_des_r)
            taul_hist.append(command_torque_l); taur_hist.append(command_torque_r)

            if len(t_hist) < 5 or len(t_hist) % 100 == 0:
                print(
                    "t={:.3f}s L(348): q={:+.3f} tau={:+.3f} | R(349): q={:+.3f} tau={:+.3f}".format(
                        t_rel,
                        ql,
                        command_torque_l,
                        qr,
                        command_torque_r,
                    ),
                    flush=True,
                )

            # control del bucle temporal
            next_tick += dt_s
            sleep_s = next_tick - time.time()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("Replay interrumpido por el usuario")
    finally:
        try:
            interface.publish_torque(0.0, 0.0)
        except Exception:
            pass
        interface.close()

    t_arr = np.asarray(t_hist, dtype=np.float64)
    ql_arr = np.asarray(ql_hist, dtype=np.float64); qr_arr = np.asarray(qr_hist, dtype=np.float64)
    qdl_arr = np.asarray(qdl_hist, dtype=np.float64); qdr_arr = np.asarray(qdr_hist, dtype=np.float64)
    ql_des_arr = np.asarray(ql_des_hist, dtype=np.float64); qr_des_arr = np.asarray(qr_des_hist, dtype=np.float64)
    taul_arr = np.asarray(taul_hist, dtype=np.float64); taur_arr = np.asarray(taur_hist, dtype=np.float64)

    if args.save_csv:
        base_mod._save_csv(csv_path, t_arr, ql_arr, qr_arr, qdl_arr, qdr_arr, ql_des_arr, qr_des_arr, taul_arr, taur_arr)
        print(f"CSV guardado en {csv_path}")

    if args.save_plot:
        base_mod._plot_run(plot_path, bool(args.show_plot), t_arr, ql_arr, qr_arr, ql_des_arr, qr_des_arr, taul_arr, taur_arr)
        print(f"Imagen guardada en {plot_path}")

    meta_path = os.path.join(run_dir, "simulation_info.txt")
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write(f"run_dir: {run_dir}\n")
        f.write(f"source_csv: {args.source_csv}\n")
        f.write(f"source_hip: {args.source_hip}\n")
        f.write(f"interface: pycandle\n")
        f.write("motor_ids: [348, 349]\n")
        f.write(f"rate_hz: {args.rate_hz}\n")
        f.write(f"apply_torque: {args.apply_torque}\n")
        f.write(f"safe_torque_limit: {args.safe_torque_limit}\n")
        f.write(f"n_samples: {len(t_arr)}\n")
        if len(t_arr) > 0:
            f.write(f"duration_s: {float(t_arr[-1]):.6f}\n")
        f.write("csv_columns: time_s, q_l_rad, q_r_rad, qd_l_rad_s, qd_r_rad_s, q_des_l_rad, q_des_r_rad, applied_torque_l_nm, applied_torque_r_nm\n")
        f.write(f"csv_path: {csv_path}\n")
        f.write(f"plot_path: {plot_path}\n")

    print(f"Metadatos guardados en {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())