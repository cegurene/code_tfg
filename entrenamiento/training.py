#!/usr/bin/env python3
"""Train SAC with Stable-Baselines3 on the MIMO hip-following environment."""

from __future__ import annotations

import argparse
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
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed


ENV_ID = "nair_gait_h0404MimoExo-v0"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    run_tag = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    parser.add_argument("--total-timesteps", type=int, default=1_000_000)
    parser.add_argument("--log-dir", type=Path, default=Path("outputs/sb3/mimo_sac"))
    parser.add_argument(
        "--scone-results-dir",
        type=str,
        default=Path(str(run_tag)),
        help="Base directory where SCONE result folders will be written",
    )
    parser.add_argument("--seed", type=int, default=0, help="Random seed for reproducibility")
    parser.add_argument("--checkpoint-freq", type=int, default=25_000, help="Checkpoint frequency in env steps")
    parser.add_argument(
        "--write-freq",
        type=int,
        default=None,
        help="Deprecated alias for --checkpoint-freq",
    )
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

    if args.write_freq is not None:
        args.checkpoint_freq = args.write_freq

    args.log_dir.mkdir(parents=True, exist_ok=True)

    try:
        original_argv = sys.argv[:]
        sys.argv = [sys.argv[0]]

        env = gym.make(ENV_ID)

        scone_results_root = args.scone_results_dir.expanduser().resolve()
        scone_results_root.mkdir(parents=True, exist_ok=True)
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
                        # LE DECIMOS A SCONE QUE EMPIECE A GRABAR ESTE EPISODIO (Igual que en tu prueba)
                        self.env.unwrapped.store_next_episode()
                        self.should_store_this_episode = True
                    except Exception:
                        self.should_store_this_episode = False
                else:
                    self.should_store_this_episode = False
                    
                return ret

            def step(self, action):
                result = self.env.step(action)
                self.total_steps += 1  # Contador global de pasos

                # Manejo de compatibilidad gymnasium / gym antiguo
                if len(result) == 5:
                    obs, rew, terminated, truncated, info = result
                    done = bool(terminated or truncated)
                    ret = (obs, rew, terminated, truncated, info)
                else:
                    obs, rew, done, info = result
                    ret = (obs, rew, done, info)

                if done:
                    # 3. Si al inicio del episodio se activó la grabación, ahora lo guardamos en disco
                    if self.should_store_this_episode:
                        try:
                            # Usamos el método nativo write_now() que ya comprobaste que funciona bien
                            self.env.unwrapped.write_now()
                            print(f"[INFO] Archivo .sto de SCONE guardado con éxito (animación completa) en el paso: {self.total_steps}")
                            self.last_saved_step = self.total_steps  # Actualizar marcador
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
            env.unwrapped.results_dir = str(scone_results_root)
            env.unwrapped.set_output_dir(str(scone_results_root / f"{launch_stamp}.{env.unwrapped.model.name()}"))
        except Exception:
            pass

        run_dir = args.log_dir / "checkpoints" / launch_stamp
        run_dir.mkdir(parents=True, exist_ok=True)

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

        model.learn(
            total_timesteps=args.total_timesteps,
            log_interval=4,
            progress_bar=args.progress_bar,
            callback=[checkpoint_cb],
        )

        model.save(str(run_dir / "sac_mimo_final"))
        if args.save_replay_buffer:
            model.save_replay_buffer(str(run_dir / "sac_mimo_final_replay_buffer"))

    finally:
        sys.argv = original_argv



if __name__ == "__main__":
    main()
