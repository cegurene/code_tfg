#!/usr/bin/env python3
"""
plot_monitor.py
Visualiza los resultados de entrenamiento almacenados en monitor.csv
de Stable-Baselines3.

Uso:
    python plot_monitor.py monitor.csv
    python plot_monitor.py monitor.csv --window 50
    python plot_monitor.py monitor.csv --save
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_monitor(path: Path) -> pd.DataFrame:
    return pd.read_csv(path, skiprows=1)


def moving_average(x, window):
    return pd.Series(x).rolling(window=window, min_periods=1).mean()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("monitor_csv", type=Path)
    parser.add_argument(
        "--window",
        type=int,
        default=20,
        help="Ventana media móvil",
    )
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    df = load_monitor(args.monitor_csv)
    rewards = df["r"].to_numpy()
    lengths = df["l"].to_numpy()
    times = df["t"].to_numpy()
    episodes = np.arange(1, len(df) + 1)

    reward_ma = moving_average(rewards, args.window)
    reward_best = np.maximum.accumulate(rewards)

    print("\n" + "=" * 80)
    print("ESTADÍSTICAS DE ENTRENAMIENTO")
    print("=" * 80)
    print(f"Episodios             : {len(df)}")
    print(f"Reward media          : {rewards.mean():.3f}")
    print(f"Reward máxima         : {rewards.max():.3f}")
    print(f"Reward mínima         : {rewards.min():.3f}")
    print(f"Reward final          : {rewards[-1]:.3f}")
    print(f"Media últimas 100     : {rewards[-100:].mean():.3f}")
    print(f"Mejor episodio        : {np.argmax(rewards) + 1}")
    print(f"Duración entrenamiento: {times[-1]:.1f} s")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # --------------------------------------------------
    # Reward por episodio
    # --------------------------------------------------
    axes[0].plot(episodes, rewards, linewidth=2, label="Reward")
    axes[0].set_title("Reward por episodio")
    axes[0].set_xlabel("Episodio")
    axes[0].set_ylabel("Reward")
    axes[0].grid(True)

    # --------------------------------------------------
    # Mejor reward acumulada
    # --------------------------------------------------
    axes[1].plot(episodes, reward_best, linewidth=2)
    axes[1].set_title("Mejor reward acumulada")
    axes[1].set_xlabel("Episodio")
    axes[1].set_ylabel("Reward")
    axes[1].grid(True)

    fig.suptitle(args.monitor_csv.name, fontsize=14, fontweight="bold")
    fig.tight_layout()

    if args.save:
        out = args.monitor_csv.with_suffix(".png")
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"\n[INFO] Guardado: {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()