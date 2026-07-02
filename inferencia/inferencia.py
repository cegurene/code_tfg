"""Script de inferencia para motores reales combinando PID y agente SAC.

Este script carga un agente SAC entrenado y una trayectoria de referencia.
En cada paso de tiempo, calcula un torque PID y predice un torque del agente.
Ambos torques se suman y se aplican a los motores de cadera reales a través
de la interfaz pyCandle.

Se incluye una inversión de polaridad para el torque del agente en la cadera
izquierda, según la descripción del entrenamiento.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import faulthandler
import importlib
import importlib.util
import multiprocessing as mp
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

# 1. Habilitar faulthandler de inmediato para trazar posibles fallos
faulthandler.enable()

# 2. Aseguramos numpy
import numpy as np

# pyCandle se carga antes que PyTorch/SB3. En algunas instalaciones la libreria
# nativa de CANdle es sensible al orden de carga de modulos con dependencias C/C++.
_PYCANDLE_MODULE = None
_PYCANDLE_IMPORT_ERROR: Optional[BaseException] = None


# =============================================================================
# PROCESO AISLADO PARA LA INTELIGENCIA ARTIFICIAL (SAC)
# =============================================================================
def proceso_agente_sac(pipe_conn, agent_path: Path, verbose: bool) -> None:
    """Este proceso se encarga ÚNICAMENTE de ejecutar PyTorch y stable_baselines3.

    Aísla las dependencias C++ de PyTorch del entorno de pyCandle para evitar SegFaults.
    """
    # Forzar a PyTorch a usar un solo hilo para evitar overhead y conflictos
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    
    import numpy as np
    from stable_baselines3 import SAC
    
    print(f"[IA] Cargando agente SAC desde: {agent_path}", flush=True)
    try:
        model = SAC.load(agent_path, print_system_info=verbose)
        print("[IA] Agente SAC cargado con éxito.", flush=True)
        pipe_conn.send("READY")
    except Exception as exc:
        print(f"[IA] ERROR crítico al cargar el agente: {exc}", flush=True)
        pipe_conn.send("ERROR")
        return
        
    # Bucle infinito de inferencia síncrona
    while True:
        try:
            # Esperamos bloqueados hasta que el controlador PID nos mande la observación
            obs = pipe_conn.recv()
            
            # Si recibimos None, es la señal de apagado seguro
            if obs is None:
                break
                
            # Predecir torque
            action, _ = model.predict(obs, deterministic=True)
            #print("ACTION:", action)
            
            # Enviar resultado de vuelta
            pipe_conn.send(action)
            
        except EOFError:
            break
        except Exception as e:
            print(f"[IA] Error durante la predicción: {e}")
            break
            
    print("[IA] Proceso finalizado correctamente.", flush=True)


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


# --- Helper functions and classes from hip_motor_torque_replay_runner.py ---
@dataclass
class SourceReplay:
    time_s: np.ndarray
    q_des_l_rad: np.ndarray
    q_des_r_rad: np.ndarray
    qd_des_l_rad_s: np.ndarray
    qd_des_r_rad_s: np.ndarray


def _parse_float(text: str) -> float:
    return float(str(text).strip().replace("\ufeff", "").replace(",", "."))


def _resolve_column(columns: Sequence[str], explicit: str, candidates: Sequence[str], kind: str) -> str:
    lowered = {c.strip().lower(): c for c in columns}
    if explicit:
        key = explicit.strip().lower()
        if key not in lowered:
            return ""
        return lowered[key]
    for candidate in candidates:
        if candidate in lowered:
            return lowered[candidate]
    return ""


def _load_source_csv(csv_path: str, period_s: float = 2.1) -> SourceReplay:
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"No existe el CSV fuente: {csv_path}")
    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"El CSV fuente '{csv_path}' no tiene cabecera.")
        time_col = _resolve_column(reader.fieldnames, "", ["time_s", "t", "time", "percent_cycle", "percent"], "tiempo")
        if not time_col:
            raise ValueError(f"No se encontró columna de tiempo en {csv_path}")
            
        ql_col = _resolve_column(
            reader.fieldnames,
            "",
            ["target_left_rad", "left_target_rad", "q_des_left_rad", "hip_pos_target_l"],
            "posición objetivo L",
        )
        qr_col = _resolve_column(
            reader.fieldnames,
            "",
            ["target_right_rad", "right_target_rad", "q_des_right_rad", "hip_pos_target_r", "hip_angle_mean"],
            "posición objetivo R",
        )
        time_raw = []
        q_raw = []
        ql_raw = []
        is_percent = "percent" in time_col.lower()
        is_degrees = "angle" in qr_col.lower() or "mean" in qr_col.lower()
        for row in reader:
            time_raw.append(_parse_float(row[time_col]))
            q_raw.append(_parse_float(row[qr_col]))
            if ql_col:
                ql_raw.append(_parse_float(row[ql_col]))
    time_arr = np.array(time_raw)
    qr_arr = np.deg2rad(np.array(q_raw)) if is_degrees else np.array(q_raw)
    if is_percent:
        time_arr = (time_arr / 100.0) * period_s
        if not ql_raw:
            n = len(qr_arr)
            ql_arr = np.roll(qr_arr, n // 2)
        else:
            ql_arr = np.deg2rad(np.array(ql_raw)) if is_degrees else np.array(ql_raw)
    else:
        ql_arr = np.array(ql_raw) if ql_raw else qr_arr.copy()
        time_arr = time_arr - time_arr[0]
    qdl_arr = np.gradient(ql_arr, time_arr)
    qdr_arr = np.gradient(qr_arr, time_arr)
    return SourceReplay(
        time_s=time_arr, 
        q_des_l_rad=-ql_arr, 
        q_des_r_rad=qr_arr, 
        qd_des_l_rad_s=-qdl_arr, 
        qd_des_r_rad_s=qdr_arr
    )


def _sample_source(source: SourceReplay, t_s: float) -> Tuple[float, float, float, float]:
    t = float(np.clip(t_s, float(source.time_s[0]), float(source.time_s[-1])))
    q_des_l = float(np.interp(t, source.time_s, source.q_des_l_rad))
    q_des_r = float(np.interp(t, source.time_s, source.q_des_r_rad))
    qd_des_l = float(np.interp(t, source.time_s, source.qd_des_l_rad_s))
    qd_des_r = float(np.interp(t, source.time_s, source.qd_des_r_rad_s))
    return q_des_l, q_des_r, qd_des_l, qd_des_r


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
            "source_hip: dual (left+right)",
        ]
    )
    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return config_path


def _save_csv(
    csv_path: str,
    time_s: np.ndarray,
    q_real_l: np.ndarray,
    q_real_r: np.ndarray,
    qd_real_l: np.ndarray,
    qd_real_r: np.ndarray,
    q_des_l: np.ndarray,
    q_des_r: np.ndarray,
    tau_pid_l: np.ndarray,
    tau_pid_r: np.ndarray,
    tau_agent_l: np.ndarray,
    tau_agent_r: np.ndarray,
    tau_combined_l: np.ndarray,
    tau_combined_r: np.ndarray,
) -> None:
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            "hip_pos_target_r",
            "hip_pos_target_l",
            "hip_pos_r",
            "hip_pos_l",
            "hip_vel_target_r",
            "hip_vel_target_l",
            "hip_vel_r",
            "hip_vel_l",
            "pid_output_r",
            "pid_output_l",
            "agent_torque_r",
            "agent_torque_l",
            "combined_torque_r",
            "combined_torque_l",
        ])
        # Calcular derivadas de velocidad deseadas usando gradientes numéricos
        qd_des_r = np.gradient(q_des_r, time_s)
        qd_des_l = np.gradient(q_des_l, time_s)
        
        for i in range(time_s.size):
            writer.writerow([
                float(time_s[i]),
                float(q_des_r[i]),
                float(q_des_l[i]),
                float(q_real_r[i]),
                float(q_real_l[i]),
                float(qd_des_r[i]),
                float(qd_des_l[i]),
                float(qd_real_r[i]),
                float(qd_real_l[i]),
                float(tau_pid_r[i]),
                float(tau_pid_l[i]),
                float(tau_agent_r[i]),
                float(tau_agent_l[i]),
                float(tau_combined_r[i]),
                float(tau_combined_l[i]),
            ])


def _plot_run(
    plot_path: str,
    show_plot: bool,
    time_s: np.ndarray,
    q_real_l: np.ndarray,
    q_real_r: np.ndarray,
    q_des_l: np.ndarray,
    q_des_r: np.ndarray,
    tau_pid_l: np.ndarray,
    tau_pid_r: np.ndarray,
    tau_agent_l: np.ndarray,
    tau_agent_r: np.ndarray,
    tau_combined_l: np.ndarray,
    tau_combined_r: np.ndarray,
) -> None:
    import matplotlib.pyplot as plt
    os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(20, 16), sharex=True)
    
    # Seguimiento Izquierdo
    axes[0, 0].plot(time_s, q_des_l, color="#d62728", linewidth=1.5, label="Objetivo L")
    axes[0, 0].plot(time_s, q_real_l, color="#1f5c9a", linewidth=2.0, label="Real L (348)")
    axes[0, 0].set_ylabel("Ángulo [rad]")
    axes[0, 0].set_title("Seguimiento Cadera Izquierda")
    axes[0, 0].grid(True, alpha=0.25)
    axes[0, 0].legend(loc="best", frameon=False)
    axes[0, 0].invert_yaxis()  
    
    # Seguimiento Derecho
    axes[0, 1].plot(time_s, q_des_r, color="#d62728", linewidth=1.5, label="Objetivo R")
    axes[0, 1].plot(time_s, q_real_r, color="#1f5c9a", linewidth=2.0, label="Real R (349)")
    axes[0, 1].set_title("Seguimiento Cadera Derecha")
    axes[0, 1].grid(True, alpha=0.25)
    axes[0, 1].legend(loc="best", frameon=False)
    
    # Torque Izquierdo
    axes[1, 0].plot(time_s, tau_agent_l, color="#8a2be2", linewidth=1.5, label="Torque Agente L")
    axes[1, 0].plot(time_s, tau_combined_l, color="#ff7f0e", linewidth=2.0, label="Torque Combinado L")
    axes[1, 0].plot(time_s, tau_pid_l, color="#2b8a3e", linewidth=1.8, label="Torque PID L")
    axes[1, 0].set_ylabel("Torque [Nm]")
    axes[1, 0].set_xlabel("Tiempo [s]")
    axes[1, 0].set_title("Torque Izquierdo")
    axes[1, 0].grid(True, alpha=0.25)
    axes[1, 0].legend(loc="best", frameon=False)
    axes[1, 0].invert_yaxis()
    
    # Torque Derecho
    axes[1, 1].plot(time_s, tau_agent_r, color="#8a2be2", linewidth=1.5, label="Torque Agente R")
    axes[1, 1].plot(time_s, tau_combined_r, color="#ff7f0e", linewidth=2.0, label="Torque Combined R")
    axes[1, 1].plot(time_s, tau_pid_r, color="#2b8a3e", linewidth=1.8, label="Torque PID R")
    axes[1, 1].set_xlabel("Tiempo [s]")
    axes[1, 1].set_title("Torque Derecho")
    axes[1, 1].grid(True, alpha=0.25)
    axes[1, 1].legend(loc="best", frameon=False)
    
    plt.tight_layout()
    plt.savefig(plot_path, dpi=220)
    if show_plot:
        plt.show()
    else:
        plt.close(fig)


# --- PyCandle Motor Interface ---
class PyCandleTorqueReplayMotorInterface:
    _baud_label: str = "1M"
    _fdcan_enabled: bool = True
    _max_torque_nm = 1.0
    _add_all_drives: bool = True
    _strict_id_match: bool = False
    _constructor_mode: str = "verbose"

    @classmethod
    def configure_from_args(cls, args) -> None:
        cls._baud_label = str(args.pycandle_baud)
        cls._fdcan_enabled = bool(args.pycandle_fdcan)
        cls._max_torque_nm = float(args.pycandle_max_torque) if args.pycandle_max_torque is not None else None
        cls._add_all_drives = bool(args.pycandle_add_all_drives)
        cls._strict_id_match = bool(args.pycandle_strict_id_match)
        cls._constructor_mode = str(args.pycandle_constructor)

    def __init__(self, joint_state_topic: str, command_topic: str, verbose: bool) -> None:
        del joint_state_topic 
        del command_topic     
        pyCandle = importlib.import_module("pyCandle")
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
        self._debug(f"Creando objeto pyCandle.Candle con constructor='{self._constructor_mode}'...")
        self.candle = pyCandle.Candle(baud, self._fdcan_enabled)
        self._debug("Objeto pyCandle.Candle creado.")
        self._debug("Ejecutando ping CAN...")
        ids = list(self.candle.ping())
        self._debug(f"Ping CAN completado. IDs detectados: {ids}")
        if not ids:
            raise RuntimeError("No se detectaron drives en el bus CAN (pyCandle ping vacio).")
        ids_to_add = ids if self._add_all_drives else self.motor_ids
        for drive_id in ids_to_add:
            if drive_id in ids:
                self._debug(f"Anadiendo MD80 id={drive_id}...")
                self.candle.addMd80(drive_id)
        if not getattr(self.candle, "md80s", None):
            raise RuntimeError("No se pudo inicializar ningun MD80 en pyCandle.")
        self.drives = {}
        for mid in self.motor_ids:
            self._debug(f"Seleccionando objeto MD80 para motor {mid}...")
            drive = self._select_drive(mid, ids)
            self.drives[mid] = drive
            
            self._debug(f"Configurando motor {mid} en RAW_TORQUE...")
            self.candle.controlMd80Mode(drive, pyCandle.RAW_TORQUE)
            self._debug(f"Habilitando motor {mid}...")
            self.candle.controlMd80Enable(drive, True)
            if self._max_torque_nm is not None:
                self._debug(f"Fijando max torque motor {mid}: {self._max_torque_nm:.3f} Nm...")
                drive.setMaxTorque(float(self._max_torque_nm))
        self._debug("Iniciando hilos de comunicacion CANdle...")
        self.candle.begin()
        self.current_pos = {mid: None for mid in self.motor_ids}
        self.current_vel = {mid: 0.0 for mid in self.motor_ids}
        self.current_effort = {mid: 0.0 for mid in self.motor_ids}
        self.last_state_ts = 0.0
        self._poll_state()

    def _debug(self, message: str) -> None:
        if self._verbose:
            print(f"[pyCandle] {message}", flush=True)

    def _create_candle(self, baud):
        if self._constructor_mode == "baud":
            return self._pycandle.Candle(baud)
        if self._constructor_mode == "verbose":
            return self._pycandle.Candle(baud, True)
        if self._constructor_mode == "quiet":
            return self._pycandle.Candle(baud, False)
        if self._constructor_mode == "usb":
            bus_type = self._resolve_usb_bus_type()
            return self._pycandle.Candle(baud, True, bus_type)
        raise RuntimeError(f"Constructor pyCandle no soportado: {self._constructor_mode}")

    def _resolve_usb_bus_type(self):
        candidates = [
            ("BusType_E", "USB"),
            ("BusType", "USB"),
            ("BusType_E", "BusType_E_USB"),
        ]
        for enum_name, value_name in candidates:
            enum_obj = getattr(self._pycandle, enum_name, None)
            if enum_obj is not None and hasattr(enum_obj, value_name):
                return getattr(enum_obj, value_name)
        for attr_name in ("USB", "BUS_USB", "BusType_E_USB"):
            if hasattr(self._pycandle, attr_name):
                return getattr(self._pycandle, attr_name)
        names = ", ".join(name for name in dir(self._pycandle) if "USB" in name.upper() or "BUS" in name.upper())
        raise RuntimeError(
            "No pude encontrar el enum USB en pyCandle. Candidatos vistos: {}".format(names or "ninguno")
        )

    def _resolve_baud_constant(self, label: str):
        normalized = str(label).strip().upper()
        if normalized in {"1M", "1MBPS", "1000000"}:
            if hasattr(self._pycandle, "CAN_BAUD_1M"):
                return self._pycandle.CAN_BAUD_1M
            raise RuntimeError("pyCandle no expone CAN_BAUD_1M.")
        if normalized in {"500K", "500KBPS", "500000"} and hasattr(self._pycandle, "CAN_BAUD_500K"):
            return self._pycandle.CAN_BAUD_500K
        raise RuntimeError(f"Baudrate pyCandle no soportado: '{label}'. Usa 1M o 500K.")

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
        raise RuntimeError(f"No se pudo mapear motor-id={motor_id} a un objeto MD80.")

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


class Ros2MotorInterface:
    def __init__(self, joint_state_topic: str, command_topic: str, verbose: bool) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception as exc:
            raise RuntimeError("ROS2 no esta disponible.") from exc
        try:
            candle_msg_mod = importlib.import_module("candle_ros2.msg")
            self._motion_cmd_cls = getattr(candle_msg_mod, "MotionCommand")
            candle_srv_mod = importlib.import_module("candle_ros2.srv")
            self._add_md80s_cls = getattr(candle_srv_mod, "AddMd80s")
            self._generic_md80_cls = getattr(candle_srv_mod, "GenericMd80Msg")
            self._set_mode_cls = getattr(candle_srv_mod, "SetModeMd80s")
        except Exception as exc:
            raise RuntimeError("No pude importar dependencias de candle_ros2.") from exc
        self._rclpy = rclpy
        self._verbose = bool(verbose)
        self.motor_ids = [348, 349]
        self.current_pos: dict[int, Optional[float]] = {mid: None for mid in self.motor_ids}
        self.current_vel: dict[int, float] = {mid: 0.0 for mid in self.motor_ids}
        self.current_effort: dict[int, float] = {mid: 0.0 for mid in self.motor_ids}
        self.last_state_ts = 0.0
        self._rclpy.init(args=None)
        self.node = Node("mimo_sac_inference")
        self.pub = self.node.create_publisher(self._motion_cmd_cls, command_topic, 10)
        self.sub = self.node.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.add_client = self.node.create_client(self._add_md80s_cls, "/candle_ros2_node/add_md80s")
        self.set_mode_client = self.node.create_client(self._set_mode_cls, "/candle_ros2_node/set_mode_md80s")
        self.enable_client = self.node.create_client(self._generic_md80_cls, "/candle_ros2_node/enable_md80s")
        self.disable_client = self.node.create_client(self._generic_md80_cls, "/candle_ros2_node/disable_md80s")
        self._configure_motors()

    def _call_service(self, client, request, name: str, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"Servicio ROS2 no disponible: {name}")
        future = client.call_async(request)
        t0 = time.time()
        while self._rclpy.ok() and not future.done():
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.time() - t0 > timeout_s:
                raise RuntimeError(f"Timeout esperando respuesta del servicio ROS2: {name}")
        return future.result()

    def _require_success(self, response, name: str) -> None:
        successes = list(getattr(response, "drives_success", []))
        if len(successes) < len(self.motor_ids) or not all(bool(v) for v in successes[: len(self.motor_ids)]):
            raise RuntimeError(f"Servicio ROS2 {name} fallo.")

    def _configure_motors(self) -> None:
        add_req = self._add_md80s_cls.Request()
        add_req.drive_ids = [int(mid) for mid in self.motor_ids]
        self._require_success(self._call_service(self.add_client, add_req, "add_md80s"), "add_md80s")
        mode_req = self._set_mode_cls.Request()
        mode_req.drive_ids = [int(mid) for mid in self.motor_ids]
        mode_req.mode = ["RAW_TORQUE"] * len(self.motor_ids)
        self._require_success(self._call_service(self.set_mode_client, mode_req, "set_mode_md80s"), "set_mode_md80s")
        enable_req = self._generic_md80_cls.Request()
        enable_req.drive_ids = [int(mid) for mid in self.motor_ids]
        self._require_success(self._call_service(self.enable_client, enable_req, "enable_md80s"), "enable_md80s")

    def _on_joint_state(self, msg) -> None:
        for mid in self.motor_ids:
            idx = None
            expected_names = {f"Joint {mid}", str(mid), f"md80_{mid}", f"MD80 {mid}"}
            for i, name in enumerate(getattr(msg, "name", []) or []):
                if str(name) in expected_names:
                    idx = i
                    break
            if idx is None:
                continue
            self.current_pos[mid] = float(msg.position[idx]) if idx < len(msg.position) else None
            self.current_vel[mid] = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
            self.current_effort[mid] = float(msg.effort[idx]) if idx < len(msg.effort) else 0.0
        self.last_state_ts = time.time()

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def wait_for_state(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 <= timeout_s:
            self.spin_once(timeout_s=0.05)
            if all(v is not None for v in self.current_pos.values()):
                return True
        return False

    def publish_torque(self, torque_l: float, torque_r: float) -> None:
        msg = self._motion_cmd_cls()
        msg.drive_ids = self.motor_ids
        msg.target_position = [0.0, 0.0]
        msg.target_velocity = [0.0, 0.0]
        msg.target_torque = [float(torque_l), float(torque_r)]
        self.pub.publish(msg)

    def close(self) -> None:
        try:
            self.publish_torque(0.0, 0.0)
        except Exception:
            pass
        try:
            disable_req = self._generic_md80_cls.Request()
            disable_req.drive_ids = [int(mid) for mid in self.motor_ids]
            self._call_service(self.disable_client, disable_req, "disable_md80s", timeout_s=2.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        finally:
            self._rclpy.shutdown()


def _probe_pycandle_constructors(baud_label: str) -> int:
    probes = {
        "baud": "c = pyCandle.Candle(baud)",
        "verbose": "c = pyCandle.Candle(baud, True)",
    }
    exit_code = 0
    for name, constructor_code in probes.items():
        code = "\n".join([
                "import faulthandler", "faulthandler.enable()", "import pyCandle",
                f"baud_label = {baud_label!r}",
                "baud = pyCandle.CAN_BAUD_1M if baud_label.upper() in ('1M', '1MBPS', '1000000') else pyCandle.CAN_BAUD_500K",
                constructor_code, "del c", "print('PROBE OK', flush=True)"
        ])
        proc = subprocess.run([sys.executable, "-c", code], text=True, capture_output=True)
        if proc.returncode != 0:
            exit_code = 1
    return exit_code


# --- Main Inference Script Logic ---
def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Inferencia en motor real combinando PID y agente SAC.")
    p.add_argument("--agent-path", type=Path, default=None, help="Ruta al modelo SAC entrenado (.zip).")
    p.add_argument("--source-csv", default="/home/carlos/Escritorio/TFG/code_tfg/mimo/scripts/healthy_hip_reference.csv", help="CSV que contiene la trayectoria de referencia.")
    p.add_argument("--interface", choices=["pycandle", "ros2"], default="pycandle", help="Interfaz de hardware.")
    p.add_argument("--joint-state-topic", default="/md80/joint_states", help="Topic ROS2.")
    p.add_argument("--command-topic", default="/md80/motion_command", help="Topic ROS2.")
    p.add_argument("--rate-hz", type=float, default=100.0, help="Frecuencia del bucle de control en Hz.")
    p.add_argument("--kp", type=float, default=2.5, help="Ganancia proporcional PID.")
    p.add_argument("--ki", type=float, default=0.0, help="Ganancia integral PID.")
    p.add_argument("--kd", type=float, default=0.1, help="Ganancia derivativa PID.")
    p.add_argument("--period", type=float, default=2.1, help="Duración en segundos de un ciclo.")
    p.add_argument("--phase-offset-l", type=float, default=50.0, help="Desfase pierna izquierda.")
    p.add_argument("--num-cycles", type=float, default=1.0, help="Número de ciclos de la trayectoria a ejecutar.")
    p.add_argument("--safe-torque-limit", type=float, default=2.0, help="Límite seguro en Nm.")
    p.add_argument("--run-dir", default="", help="Directorio salida.")
    p.add_argument("--csv-path", default="", help="Ruta CSV.")
    p.add_argument("--plot-path", default="", help="Ruta gráfico.")
    p.add_argument("--save-csv", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--save-plot", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--show-plot", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--verbose", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-baud", default="1M")
    p.add_argument("--pycandle-fdcan", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-constructor", choices=["verbose", "quiet", "usb", "baud"], default="verbose")
    p.add_argument("--pycandle-max-torque", type=float, default=1.0)
    p.add_argument("--pycandle-add-all-drives", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pycandle-strict-id-match", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--diagnose-pycandle-only", action="store_true")
    p.add_argument("--probe-pycandle-constructors", action="store_true")
    return p


def _get_sac_observation(
    t_now: float, period: float, q_l: float, qd_l: float, q_r: float, qd_r: float,
    target_q_l: float, target_q_r: float, target_qd_l: float, target_qd_r: float,
    last_action: np.ndarray, phase_offset_l: float = 50.0,
) -> np.ndarray:
    phase_right = (100.0 * t_now / period) % 100.0
    phase_left = (phase_right + phase_offset_l) % 100.0
    err_right = target_q_r - q_r
    err_left = target_q_l - q_l
    vel_err_right = target_qd_r - qd_r
    vel_err_left = target_qd_l - qd_l
    exo_obs = np.array([q_r, qd_r, q_l, qd_l], dtype=np.float64)
    obs_main = np.array([
        np.sin(2.0 * np.pi * phase_right / 100.0),
        np.cos(2.0 * np.pi * phase_right / 100.0),
        np.sin(2.0 * np.pi * phase_left / 100.0),
        np.cos(2.0 * np.pi * phase_left / 100.0),
        q_r, q_l, qd_r, qd_l,
        target_q_r, target_q_l, target_qd_r, target_qd_l,
        err_right, err_left, vel_err_right, vel_err_left,
    ], dtype=np.float64)
    observation = np.concatenate([obs_main, exo_obs, last_action], dtype=np.float64)
    return observation[np.newaxis, :]


def main() -> int:
    args = _build_parser().parse_args()
    if args.probe_pycandle_constructors:
        return _probe_pycandle_constructors(args.pycandle_baud)
    if not args.source_csv.strip():
        raise ValueError("--source-csv es obligatorio.")
    if args.agent_path is None and not args.diagnose_pycandle_only:
        raise ValueError("--agent-path es obligatorio salvo que uses --diagnose-pycandle-only.")
    if args.agent_path is not None and not args.agent_path.exists():
        raise FileNotFoundError(f"Agente SAC no encontrado en: {args.agent_path}")
        
    # --- Configuración de directorios de salida ---
    run_tag = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.strip() if args.run_dir else str(repo_root / "outputs" / "inferencia" / run_tag)
    os.makedirs(run_dir, exist_ok=True)
    csv_path = args.csv_path.strip() if args.csv_path else os.path.join(run_dir, "telemetry.csv")
    plot_path = args.plot_path.strip() if args.plot_path else os.path.join(run_dir, "gait_tracking_plot.png")

    # 1. Cargar la trayectoria de referencia
    print(f"[Main] Cargando trayectoria de referencia desde: {args.source_csv}")
    source = _load_source_csv(args.source_csv, period_s=args.period)
    _write_run_config_txt(run_dir, args, source)

    # 2. Encontrar los desfases temporales óptimos para empezar en posición 0
    idx_start_r = np.argmin(np.abs(source.q_des_r_rad))
    t_offset_r = float(source.time_s[idx_start_r])
    
    print(f"[Main] Alineación por cruce por cero calculada:")
    print(f"       -> Offset temporal derecho (inicio en 0 rad): {t_offset_r:.4f} s")

    # La izquierda va exactamente medio ciclo por detrás
    t_l_init = (t_offset_r + (args.phase_offset_l / 100.0) * args.period) % args.period

    # Tiempo que tarda esa fase desplazada en volver a cruzar por cero
    delay_left_activation = (args.phase_offset_l / 100.0) * args.period

    print(f"       -> Offset temporal izquierdo: {t_l_init:.4f} s")
    print(f"       -> Activación izquierda tras {delay_left_activation:.4f} s")

    # 3. Lanzar el proceso aislado del agente SAC si no es modo diagnóstico de hardware
    pipe_main, pipe_agent = mp.Pipe()
    proceso_ia = None
    if not args.diagnose_pycandle_only:
        proceso_ia = mp.Process(
            target=proceso_agente_sac,
            args=(pipe_agent, args.agent_path, args.verbose),
            daemon=True
        )
        proceso_ia.start()
        
        # Esperar confirmación de carga del agente
        status = pipe_main.recv()
        if status != "READY":
            print("[Main] ERROR: El proceso del agente SAC no pudo inicializarse correctamente.")
            return 1

    # 4. Inicializar la interfaz de los motores reales
    print(f"[Main] Inicializando interfaz de hardware de tipo: {args.interface}")
    if args.interface == "pycandle":
        PyCandleTorqueReplayMotorInterface.configure_from_args(args)
        interface = PyCandleTorqueReplayMotorInterface(
            joint_state_topic=args.joint_state_topic,
            command_topic=args.command_topic,
            verbose=args.verbose
        )
    else:
        interface = Ros2MotorInterface(
            joint_state_topic=args.joint_state_topic,
            command_topic=args.command_topic,
            verbose=args.verbose
        )

    if not interface.wait_for_state(timeout_s=5.0):
        print("[Main] ERROR: No se recibieron estados iniciales de los motores. Abortando.")
        interface.close()
        if proceso_ia:
            pipe_main.send(None)
        return 1

    # 5. Configuración del bucle de control síncrono
    dt_loop = 1.0 / args.rate_hz
    duration_s = args.num_cycles * args.period
    steps = int(duration_s * args.rate_hz)
    
    # Listas para almacenar la telemetría histórica
    time_hist, ql_hist, qr_hist, qdl_hist, qdr_hist = [], [], [], [], []
    ql_des_hist, qr_des_hist = [], []
    taul_pid_hist, taur_pid_hist = [], []
    taul_agent_hist, taur_agent_hist = [], []
    taul_combined_hist, taur_combined_hist = [], []

    print(f"[Main] Iniciando ejecución de {args.num_cycles} ciclos ({duration_s:.2f} s, {steps} pasos)...")
    
    # Variables de control PID
    integral_l, integral_r = 0.0, 0.0
    last_err_l, last_err_r = 0.0, 0.0
    last_action = np.zeros(2, dtype=np.float64)

    t_start_loop = time.time()
    
    for step in range(steps):
        t_bucle = step * dt_loop
        
        # --- Cálculo de referencias con desfase dinámico y gating ---
        t_r = (t_offset_r + t_bucle) % args.period
        q_des_r = float(np.interp(t_r, source.time_s, source.q_des_r_rad))
        qd_des_r = float(np.interp(t_r, source.time_s, source.qd_des_r_rad_s))
        
        if t_bucle < delay_left_activation:
            q_des_l = 0.0
            qd_des_l = 0.0
        else:
            t_l = (t_l_init + (t_bucle - delay_left_activation)) % args.period

            q_des_l = float(np.interp(t_l, source.time_s, source.q_des_l_rad))
            qd_des_l = float(np.interp(t_l, source.time_s, source.qd_des_l_rad_s))

        # --- Lectura de sensores de los motores reales ---
        interface.spin_once(timeout_s=0.0)
        q_real_l = interface.current_pos[348]
        qd_real_l = interface.current_vel[348]
        q_real_r = interface.current_pos[349]
        qd_real_r = interface.current_vel[349]
        
        if q_real_l is None or q_real_r is None:
            # Fallback en caso de pérdida temporal de lectura de hardware
            q_real_l = q_real_l if q_real_l is not None else 0.0
            q_real_r = q_real_r if q_real_r is not None else 0.0

        # --- Controlador PID Clásico ---
        err_r = q_des_r - q_real_r
        err_l = q_des_l - q_real_l
        
        integral_r += err_r * dt_loop
        integral_l += err_l * dt_loop
        
        deriv_r = (err_r - last_err_r) / dt_loop if step > 0 else 0.0
        deriv_l = (err_l - last_err_l) / dt_loop if step > 0 else 0.0
        
        last_err_r, last_err_l = err_r, err_l
        
        tau_pid_r = args.kp * err_r + args.ki * integral_r + args.kd * deriv_r
        tau_pid_l = args.kp * err_l + args.ki * integral_l + args.kd * deriv_l

        # --- Inferencia del Agente SAC (Proceso aislado) ---
        tau_agent_r, tau_agent_l = 0.0, 0.0
        if not args.diagnose_pycandle_only:
            # Empaquetar la observación actual
            obs = _get_sac_observation(
                t_bucle, args.period, q_real_l, qd_real_l, q_real_r, qd_real_r,
                q_des_l, q_des_r, qd_des_l, qd_des_r, last_action, args.phase_offset_l
            )
            # Enviar por el Pipe al proceso de PyTorch
            pipe_main.send(obs)
            # Recibir la acción calculada de vuelta (bloqueante pero inmediato)
            action = pipe_main.recv()
            last_action = np.array(action, dtype=np.float64).flatten()
            
            # Mapeo de acciones del agente a torques físicos
            tau_agent_r = -float(last_action[0])
            tau_agent_l = -float(last_action[1])
            
            # Aplicar la inversión de polaridad en la cadera izquierda según el entrenamiento
            #tau_agent_l = -tau_agent_l

        # --- Combinación de Leyes de Control y Saturación Segura ---
        #tau_combined_r = np.clip(tau_pid_r, -args.safe_torque_limit, args.safe_torque_limit)
        #tau_combined_l = np.clip(tau_pid_l, -args.safe_torque_limit, args.safe_torque_limit)
        tau_combined_r = np.clip(tau_pid_r + tau_agent_r, -args.safe_torque_limit, args.safe_torque_limit)
        tau_combined_l = np.clip(tau_pid_l + tau_agent_l, -args.safe_torque_limit, args.safe_torque_limit)

        # Enviar comandos de torque directos al hardware
        interface.publish_torque(tau_combined_l, tau_combined_r)

        # --- Guardar Telemetría ---
        time_hist.append(t_bucle)
        qr_hist.append(q_real_r); ql_hist.append(q_real_l)
        qdr_hist.append(qd_real_r); qdl_hist.append(qd_real_l)
        qr_des_hist.append(q_des_r); ql_des_hist.append(q_des_l)
        taur_pid_hist.append(tau_pid_r); taul_pid_hist.append(tau_pid_l)
        taur_agent_hist.append(tau_agent_r); taul_agent_hist.append(tau_agent_l)
        taur_combined_hist.append(tau_combined_r); taul_combined_hist.append(tau_combined_l)

        # --- Control de tiempo estricto (frecuencia de muestreo en tiempo real) ---
        t_elapsed = time.time() - t_start_loop
        t_next_target = (step + 1) * dt_loop
        if t_elapsed < t_next_target:
            time.sleep(t_next_target - t_elapsed)

    print("[Main] Trayectoria finalizada. Deteniendo motores de forma segura...")
    interface.close()

    # Apagar el proceso hijo de la IA de forma segura
    if proceso_ia:
        try:
            pipe_main.send(None)
            proceso_ia.join(timeout=2.0)
        except Exception:
            pass

    # Verificación de datos guardados para evitar IndexError
    if len(time_hist) == 0:
        print("[Main] ERROR: El bucle de control finalizó sin registrar datos. Revisa la conexión física con los motores.")
        return 1

    # Convertir telemetría recopilada en arrays de numpy
    t_arr = np.array(time_hist, dtype=np.float64)
    ql_arr = np.array(ql_hist, dtype=np.float64); qr_arr = np.array(qr_hist, dtype=np.float64)
    qdl_arr = np.array(qdl_hist, dtype=np.float64); qdr_arr = np.array(qdr_hist, dtype=np.float64)
    ql_des_arr = np.array(ql_des_hist, dtype=np.float64); qr_des_arr = np.array(qr_des_hist, dtype=np.float64)
    taul_pid_arr = np.array(taul_pid_hist, dtype=np.float64); taur_pid_arr = np.array(taur_pid_hist, dtype=np.float64)
    taul_agent_arr = np.array(taul_agent_hist, dtype=np.float64); taur_agent_arr = np.array(taur_agent_hist, dtype=np.float64)
    taul_combined_arr = np.array(taul_combined_hist, dtype=np.float64); taur_combined_arr = np.array(taur_combined_hist, dtype=np.float64)

    if args.save_csv:
        print(f"[Main] Guardando datos de telemetría en: {csv_path}")
        # Llamamos exactamente con los 14 argumentos posicionales requeridos
        _save_csv(
            csv_path, t_arr, ql_arr, qr_arr, qdl_arr, qdr_arr, ql_des_arr, qr_des_arr,
            taul_pid_arr, taur_pid_arr, taul_agent_arr, taur_agent_arr, taul_combined_arr, taur_combined_arr
        )

    if args.save_plot:
        print(f"[Main] Generando gráfico de rendimiento en: {plot_path}")
        # Llamamos exactamente con los 13 argumentos requeridos para _plot_run
        _plot_run(
            plot_path, bool(args.show_plot), t_arr, ql_arr, qr_arr, ql_des_arr, qr_des_arr,
            taul_pid_arr, taur_pid_arr, taul_agent_arr, taur_agent_arr, taul_combined_arr, taur_combined_arr
        )

    print("[Main] Script de inferencia finalizado con éxito.")
    return 0


if __name__ == "__main__":
    sys.exit(main())