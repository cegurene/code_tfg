#!/usr/bin/env python3
"""Train SAC with Stable-Baselines3 on the MIMO hip-following environment."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from datetime import datetime
import random
import numpy as np
import torch

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent / "sconegym"))

import gymnasium as gym
import sconegym  # noqa: F401  # Registers the custom envs.
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed


ENV_ID = "nair_gait_h0404MimoExo-v0"

# ---------------------------------------------------------------------------
# Nombres de variables biomecánicas que se intentan leer del entorno SCONE.
# Cadera izquierda  → sufijo "_l" / "left"
# Cadera derecha    → sufijo "_r" / "right"
# Exo izquierdo     → sufijo "_exo_l" / similar
# ---------------------------------------------------------------------------
_CSV_COLUMNS = [
    "timestep",
    "episode",
    # --- Posición de cadera ---
    "hip_pos_l",
    "hip_pos_r",
    # --- Posición de exo ---
    "exo_pos_l",
    "exo_pos_r",
    # --- Velocidad de cadera ---
    "hip_vel_l",
    "hip_vel_r",
    # --- Velocidad de exo ---
    "exo_vel_l",
    "exo_vel_r",
    # --- Activaciones musculares ---
    "act_iliopsoas_l",
    "act_iliopsoas_r",
    "act_glut_l",
    "act_glut_r",
    # --- Longitudes musculares ---
    "muscle_length_iliopsoas_l",
    "muscle_length_iliopsoas_r",
    "muscle_length_glut_l",
    "muscle_length_glut_r",
    # --- Fuerzas musculares ---
    "muscle_force_iliopsoas_l",
    "muscle_force_iliopsoas_r",
    "muscle_force_glut_l",
    "muscle_force_glut_r",
    # --- Torque motor exo ---
    "motor_torque_l",
    "motor_torque_r",
    # --- Salida PID exo ---
    "pid_output_l",
    "pid_output_r",
    # --- Referencia (target) ---
    "hip_pos_target_l",
    "hip_pos_target_r",
    "hip_vel_target_l",
    "hip_vel_target_r",
]


def _collect_row(env_unwrapped, timestep: int, episode: int, info: dict) -> dict:
    """
    Extrae todas las variables biomecánicas del entorno en el paso actual.

    La API interna de sconegym/SCONE varía según la versión; por eso se
    prueban varias rutas alternativas para cada variable.  Si una variable
    no existe, se guarda NaN y el CSV sigue siendo válido.
    """
    e = env_unwrapped  # alias corto

    # ------------------------------------------------------------------
    # Acceso a arrays del modelo SCONE por índice numérico
    # (implementación del usuario, más directa y eficiente)
    # ------------------------------------------------------------------
    def joint_pos(position):
        """Posición (ángulo) de una articulación."""
        return float(e.model.dof_position_array()[position])

    def joint_vel(position):
        """Velocidad de una articulación."""
        return float(e.model.dof_velocity_array()[position])

    def muscle_act(position):
        """Activación de un músculo (0-1)."""
        return float(e.model.muscle_activation_array()[position])

    def muscle_length(position):
        """Longitud de fibra muscular."""
        return float(e.model.muscle_fiber_length_array()[position])

    def muscle_force(position):
        """Fuerza muscular (N)."""
        return float(e.model.muscle_force_array()[position])

    # ------------------------------------------------------------------
    # Posición/velocidad del exo: índices distintos a los de la cadera
    # ------------------------------------------------------------------
    def exo_pos(position):
        return float(e.model.dof_position_array()[position])

    def exo_vel(position):
        return float(e.model.dof_velocity_array()[position])

    def exo_torque(position):
        return float(e.model.actuators()[position].input())

    # ------------------------------------------------------------------
    # Targets y PID: llegan en el dict `info` devuelto por env.step()
    # ------------------------------------------------------------------
    def target_pos(position):
        """Posición objetivo de cadera (índice 0=r, 1=l según convención del entorno)."""
        arr = np.asarray(info.get("target_pos", [float("nan"), float("nan")]), dtype=float)
        return float(arr[position]) if position < len(arr) else float("nan")

    def target_vel(position):
        """Velocidad objetivo de cadera."""
        arr = np.asarray(info.get("target_vel", [float("nan"), float("nan")]), dtype=float)
        return float(arr[position]) if position < len(arr) else float("nan")

    def pid_output(position):
        """Salida del controlador PID del exo."""
        arr = info.get("pid_r", None)
        if arr is None:
            return float("nan")
        arr = np.asarray(arr, dtype=float)
        return float(arr[position]) if position < len(arr) else float("nan")

    # ------------------------------------------------------------------
    # Construcción de la fila
    # ------------------------------------------------------------------
    row = {
        "timestep": timestep,
        "episode": episode,
        # Posición cadera (índices en dof_position_array)
        "hip_pos_l": joint_pos(2),
        "hip_pos_r": joint_pos(0),
        # Posición exo
        "exo_pos_l": exo_pos(3),
        "exo_pos_r": exo_pos(1),
        # Velocidad cadera
        "hip_vel_l": joint_vel(2),
        "hip_vel_r": joint_vel(0),
        # Velocidad exo
        "exo_vel_l": exo_vel(3),
        "exo_vel_r": exo_vel(1),
        # Activaciones musculares (índices en muscle_activation_array)
        "act_iliopsoas_l": muscle_act(3),
        "act_iliopsoas_r": muscle_act(1),
        "act_glut_l":      muscle_act(2),
        "act_glut_r":      muscle_act(0),
        # Longitudes musculares
        "muscle_length_iliopsoas_l": muscle_length(3),
        "muscle_length_iliopsoas_r": muscle_length(1),
        "muscle_length_glut_l":      muscle_length(2),
        "muscle_length_glut_r":      muscle_length(0),
        # Fuerzas musculares
        "muscle_force_iliopsoas_l": muscle_force(3),
        "muscle_force_iliopsoas_r": muscle_force(1),
        "muscle_force_glut_l":      muscle_force(2),
        "muscle_force_glut_r":      muscle_force(0),
        # Torque motor exo (índices en dofs())
        "motor_torque_l": exo_torque(5),
        "motor_torque_r": exo_torque(4),
        # Salida PID (índices en info["pid_r"])
        "pid_output_l":   info.get("pid_r", None),
        "pid_output_r":   info.get("pid_l", None),
        # Targets (índices en info["target_pos"] / info["target_vel"])
        "hip_pos_target_l": target_pos(1),
        "hip_pos_target_r": target_pos(0),
        "hip_vel_target_l": target_vel(1),
        "hip_vel_target_r": target_vel(0),
    }
    return row


# ---------------------------------------------------------------------------
# Callback SB3 que graba un episodio completo cada vez que se guarda un ckpt
# ---------------------------------------------------------------------------
class EpisodeDataCallback(BaseCallback):
    """
    En cada checkpoint guarda un CSV con los datos biomecánicos del episodio
    más reciente completo.

    Flujo:
      1. Durante el entrenamiento acumula datos paso a paso en un buffer
         del episodio activo  (`_episode_buffer`).
      2. Al terminar un episodio copia el buffer a `_last_complete_episode`.
      3. Cuando `CheckpointCallback` dispara (cada `checkpoint_freq` pasos),
         este callback vuelca `_last_complete_episode` a disco como
         `episode_data_step_{N}.csv` en el mismo `run_dir`.
    """

    def __init__(self, run_dir: Path, checkpoint_freq: int, verbose: int = 0):
        super().__init__(verbose)
        self.run_dir = run_dir
        self.checkpoint_freq = checkpoint_freq

        self._episode_buffer: list[dict] = []
        self._last_complete_episode: list[dict] = []
        self._current_episode: int = 0
        self._last_checkpoint_step: int = 0

    # ------------------------------------------------------------------
    # Helpers para acceder al entorno a través de las capas de wrappers
    # ------------------------------------------------------------------
    def _get_unwrapped(self):
        try:
            return self.training_env.envs[0].unwrapped
        except Exception:
            try:
                return self.training_env.unwrapped
            except Exception:
                return None

    def _get_current_episode(self) -> int:
        e = self._get_unwrapped()
        if e is None:
            return self._current_episode
        return getattr(e, "episode", self._current_episode)

    # ------------------------------------------------------------------
    # Hooks de SB3
    # ------------------------------------------------------------------
    def _on_step(self) -> bool:
        e = self._get_unwrapped()
        if e is None:
            return True

        # Extraer info del paso actual (SB3 lo expone como lista de dicts,
        # uno por env; con un solo env tomamos el primero)
        infos = self.locals.get("infos", None)
        if infos is not None and len(infos) > 0:
            info = infos[0] if isinstance(infos[0], dict) else {}
        else:
            info = self.locals.get("info", {}) or {}

        row = _collect_row(e, self.num_timesteps, self._get_current_episode(), info)
        self._episode_buffer.append(row)

        # Detectar fin de episodio (dones viene de SB3 como array)
        dones = self.locals.get("dones", None)
        if dones is None:
            dones = self.locals.get("done", None)
        episode_done = False
        if dones is not None:
            try:
                episode_done = bool(np.any(dones))
            except Exception:
                episode_done = bool(dones)

        if episode_done:
            # Guardar episodio terminado y reiniciar buffer
            self._last_complete_episode = self._episode_buffer.copy()
            self._episode_buffer = []
            self._current_episode += 1

        # ---- Guardar CSV si toca checkpoint ----
        if (
            self.num_timesteps > 0
            and self.num_timesteps % self.checkpoint_freq == 0
            and self.num_timesteps != self._last_checkpoint_step
        ):
            self._save_episode_csv()
            self._last_checkpoint_step = self.num_timesteps

        return True

    def _on_training_end(self) -> None:
        """Al finalizar el entrenamiento guarda el último episodio completo."""
        if self._last_complete_episode:
            self._save_episode_csv(suffix="final")

    # ------------------------------------------------------------------
    # Escritura del CSV
    # ------------------------------------------------------------------
    def _save_episode_csv(self, suffix: str | None = None) -> None:
        data = self._last_complete_episode
        if not data:
            if self.verbose:
                print(
                    f"[EpisodeDataCallback] Paso {self.num_timesteps}: "
                    "no hay episodio completo todavía, CSV omitido."
                )
            return

        tag = suffix if suffix else f"step_{self.num_timesteps:010d}"
        csv_path = self.run_dir / f"episode_data_{tag}.csv"

        try:
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=_CSV_COLUMNS, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(data)
            print(
                f"[INFO] CSV biomecánico guardado: {csv_path.name}  "
                f"({len(data)} filas, episodio {data[0].get('episode', '?')})"
            )
        except Exception as exc:
            print(f"[WARN] No se pudo guardar el CSV biomecánico: {exc}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    run_tag = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    script_dir = Path(__file__).resolve().parent

    parser.add_argument("--total-timesteps", type=int, default=1_000_000)
    parser.add_argument("--log-dir", type=Path, default=script_dir.parent / "outputs" / "entrenamiento")
    parser.add_argument(
        "--scone-results-dir",
        type=str,
        default="",
        help="Base directory where SCONE result folders will be written (optional)",
    )
    parser.add_argument("--seed", type=int, default=0, help="Random seed for reproducibility")
    parser.add_argument("--checkpoint-freq", type=int, default=25_000, help="Checkpoint frequency in env steps")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--buffer-size", type=int, default=300_000)
    parser.add_argument("--learning-starts", type=int, default=10_000)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--tau", type=float, default=0.005)
    parser.add_argument(
        "--save-scone-episodes",
        action="store_true",
        help="Write SCONE .sto results at episode end. Off by default because it is slow and disk-heavy.",
    )
    parser.add_argument(
        "--save-replay-buffer",
        action="store_true",
        help="Save SAC replay buffers in checkpoints and at the end of training.",
    )
    parser.add_argument(
        "--progress-bar",
        action="store_true",
        help="Enable SB3 progress bar if tqdm/rich are installed.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="auto",
        choices=["auto", "cuda", "cpu"],
        help="Training device for PyTorch/SB3",
    )
    return parser.parse_known_args()[0]


def main():
    args = parse_args()

    args.log_dir.mkdir(parents=True, exist_ok=True)

    try:
        original_argv = sys.argv[:]
        sys.argv = [sys.argv[0]]

        env = gym.make(ENV_ID)

        launch_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        class SavePeriodicScone(gym.Wrapper):
            def __init__(self, env, save_freq: int):
                super().__init__(env)
                self.save_freq = save_freq
                self.total_steps = 0
                self.last_saved_step = 0
                self.should_store_this_episode = False

            def reset(self, **kwargs):
                # 1. Ejecutar el reset nativo del entorno
                ret = self.env.reset(**kwargs)

                # 2. Comprobar si ya toca preparar la grabación del PRÓXIMO episodio
                if self.total_steps == 0 or (self.total_steps - self.last_saved_step) >= self.save_freq:
                    try:
                        self.env.unwrapped.store_next_episode()
                        self.should_store_this_episode = True
                    except Exception:
                        self.should_store_this_episode = False
                else:
                    self.should_store_this_episode = False

                return ret

            def step(self, action):
                result = self.env.step(action)
                self.total_steps += 1

                if len(result) == 5:
                    obs, rew, terminated, truncated, info = result
                    done = bool(terminated or truncated)
                    ret = (obs, rew, terminated, truncated, info)
                else:
                    obs, rew, done, info = result
                    ret = (obs, rew, done, info)

                if done:
                    if self.should_store_this_episode:
                        try:
                            self.env.unwrapped.write_now()
                            print(f"[INFO] Archivo .sto de SCONE guardado con éxito (animación completa) en el paso: {self.total_steps}")
                            self.last_saved_step = self.total_steps
                        except Exception as e:
                            print(f"[WARN] No se pudo guardar el .sto con write_now(): {e}")

                    try:
                        self.env.unwrapped.episode += 1
                    except Exception:
                        pass

                return ret

        if args.save_scone_episodes:
            env = SavePeriodicScone(env, save_freq=args.checkpoint_freq)

        try:
            if args.scone_results_dir:
                env.unwrapped.results_dir = str(Path(args.scone_results_dir).expanduser().resolve())
            env.unwrapped.set_output_dir(f"{launch_stamp}.{env.unwrapped.model.name()}")
        except Exception:
            pass

        run_dir = args.log_dir / launch_stamp
        run_dir.mkdir(parents=True, exist_ok=True)

        # Config file
        config_file = run_dir / "config.txt"
        with open(config_file, "w") as f:
            f.write("Reward weights\n")
            f.write("====================\n\n")
            try:
                for key, value in env.unwrapped.rwd_keys.items():
                    f.write(f"{key}: {value}\n")
            except Exception as e:
                f.write(f"Could not read reward weights: {e}\n")

        env = Monitor(env, filename=str(run_dir / "monitor.csv"))

        env.reset(seed=args.seed)

        try:
            env.action_space.seed(args.seed)
        except Exception:
            pass

        set_random_seed(args.seed)
        np.random.seed(args.seed)
        random.seed(args.seed)
        try:
            torch.manual_seed(args.seed)
            if torch.cuda.is_available():
                torch.cuda.manual_seed_all(args.seed)
        except Exception:
            pass

        device = args.device
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda" and not torch.cuda.is_available():
            print("[WARN] CUDA requested but not available. Falling back to CPU.")
            device = "cpu"
        print(f"[INFO] SAC device: {device}")

        print("Reward keys in the environment:")
        print(env.unwrapped.rwd_keys)

        model = SAC(
            policy="MlpPolicy",
            env=env,
            verbose=0,
            device=device,
            learning_rate=args.learning_rate,
            buffer_size=args.buffer_size,
            learning_starts=args.learning_starts,
            batch_size=args.batch_size,
            tau=args.tau,
            gamma=args.gamma,
            train_freq=1,
            gradient_steps=1,
            ent_coef="auto",
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=str(run_dir / "tensorboard"),
            seed=args.seed,
        )

        checkpoint_cb = CheckpointCallback(
            save_freq=args.checkpoint_freq,
            save_path=run_dir,
            name_prefix="sac_mimo",
            save_replay_buffer=args.save_replay_buffer,
        )

        # Callback que guarda el CSV biomecánico en cada checkpoint
        episode_data_cb = EpisodeDataCallback(
            run_dir=run_dir,
            checkpoint_freq=args.checkpoint_freq,
            verbose=1,
        )

        model.learn(
            total_timesteps=args.total_timesteps,
            log_interval=4,
            progress_bar=args.progress_bar,
            callback=[checkpoint_cb, episode_data_cb],
        )

        model.save(str(run_dir / "sac_mimo_final"))
        if args.save_replay_buffer:
            model.save_replay_buffer(str(run_dir / "sac_mimo_final_replay_buffer"))

    finally:
        sys.argv = original_argv



if __name__ == "__main__":
    main()