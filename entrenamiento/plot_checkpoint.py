#!/usr/bin/env python3
"""
plot_episode.py
---------------
Genera 4 gráficas a partir de un CSV de episodio producido por EpisodeDataCallback:
  1. Posición de cadera  — real (L/R) vs objetivo (L/R)
  2. Velocidad de cadera — real (L/R) vs objetivo (L/R)
  3. Activaciones musculares — iliopsoas y glúteo (L/R)
  4. Torque del exo       — motor torque (L/R)

Uso:
    python plot_episode.py episode_data_step_0000025000.csv
    python plot_episode.py episode_data_step_0000025000.csv --save   # guarda PNGs
    python plot_episode.py outputs/sb3/mimo_sac/checkpoints/*/episode_data_*.csv --overlay
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Paleta y estilo
# ---------------------------------------------------------------------------
LEFT_COLOR = "#2563EB"   # azul
RIGHT_COLOR = "#DC2626"  # rojo
TARGET_ALPHA = 0.45
TARGET_LS = "--"

MUSCLE_COLORS = {
    "act_iliopsoas_l": "#7C3AED",   # violeta
    "act_iliopsoas_r": "#DB2777",   # rosa
    "act_glut_l":      "#059669",   # verde
    "act_glut_r":      "#D97706",   # ámbar
}

TORQUE_COLORS = {
    "motor_torque_l": LEFT_COLOR,
    "motor_torque_r": RIGHT_COLOR,
}

MUSCLE_LABELS = {
    "act_iliopsoas_l": "Iliopsoas L",
    "act_iliopsoas_r": "Iliopsoas R",
    "act_glut_l":      "Glúteo L",
    "act_glut_r":      "Glúteo R",
}


def apply_style():
    plt.rcParams.update({
        "figure.facecolor": "#F8FAFC",
        "axes.facecolor":   "#FFFFFF",
        "axes.edgecolor":   "#CBD5E1",
        "axes.linewidth":   0.8,
        "axes.grid":        True,
        "grid.color":       "#E2E8F0",
        "grid.linewidth":   0.6,
        "grid.linestyle":   "-",
        "xtick.color":      "#64748B",
        "ytick.color":      "#64748B",
        "xtick.labelsize":  9,
        "ytick.labelsize":  9,
        "axes.labelsize":   10,
        "axes.labelcolor":  "#1E293B",
        "axes.titlesize":   11,
        "axes.titleweight": "bold",
        "axes.titlecolor":  "#0F172A",
        "legend.fontsize":  9,
        "legend.framealpha": 0.85,
        "legend.edgecolor": "#CBD5E1",
        "font.family":      "sans-serif",
        "lines.linewidth":  1.6,
    })


def _dt(df: pd.DataFrame) -> float | None:
    """Paso de tiempo medio, si existe columna 'time'."""
    if "time" in df.columns and len(df) > 1:
        t = df["time"].to_numpy(dtype=float)
        dt = np.diff(t)
        dt = dt[dt > 0]
        if len(dt) > 0:
            return float(np.mean(dt))
    return None


def print_metrics(df: pd.DataFrame, name: str):
    print("\n" + "=" * 80)
    print(f"MÉTRICAS: {name}")
    print("=" * 80)

    # --------------------------------------------------
    # Error de posición
    # --------------------------------------------------
    if {"hip_pos_l", "hip_pos_target_l"}.issubset(df.columns):
        pos_err_l = np.mean(np.abs(df["hip_pos_l"] - df["hip_pos_target_l"]))
        print(f"Error medio posición cadera L: {np.degrees(pos_err_l):.3f} °")
    if {"hip_pos_r", "hip_pos_target_r"}.issubset(df.columns):
        pos_err_r = np.mean(np.abs(df["hip_pos_r"] - df["hip_pos_target_r"]))
        print(f"Error medio posición cadera R: {np.degrees(pos_err_r):.3f} °")

    # --------------------------------------------------
    # Error de velocidad
    # --------------------------------------------------
    if {"hip_vel_l", "hip_vel_target_l"}.issubset(df.columns):
        vel_err_l = np.mean(np.abs(df["hip_vel_l"] - df["hip_vel_target_l"]))
        print(f"Error medio velocidad cadera L: {np.degrees(vel_err_l):.3f} °/s")
    if {"hip_vel_r", "hip_vel_target_r"}.issubset(df.columns):
        vel_err_r = np.mean(np.abs(df["hip_vel_r"] - df["hip_vel_target_r"]))
        print(f"Error medio velocidad cadera R: {np.degrees(vel_err_r):.3f} °/s")

    # --------------------------------------------------
    # Activaciones musculares medias
    # --------------------------------------------------
    muscles = [
        ("act_iliopsoas_l", "Iliopsoas L"),
        ("act_iliopsoas_r", "Iliopsoas R"),
        ("act_glut_l", "Glúteo L"),
        ("act_glut_r", "Glúteo R"),
    ]
    print("\nActivación media muscular:")
    for col, label in muscles:
        if col in df.columns:
            mean_act = df[col].mean()
            print(f"  {label:<15}: {100 * mean_act:.1f} %")

    # --------------------------------------------------
    # Torque (magnitud + suavizado)
    # --------------------------------------------------
    dt = _dt(df)
    print("\nTorque exoesqueleto:")
    for col, label in [
        ("motor_torque_l", "L"),
        ("motor_torque_r", "R"),
    ]:
        if col not in df.columns:
            continue

        torque = df[col].to_numpy(dtype=float)
        mean_torque = torque.mean()
        mean_abs_torque = np.abs(torque).mean()
        peak_torque = np.abs(torque).max()
        rms_torque = np.sqrt(np.mean(torque ** 2))

        print(
            f"  Cadera {label}: "
            f"media={mean_torque:.3f} Nm, "
            f"|media|={mean_abs_torque:.3f} Nm, "
            f"RMS={rms_torque:.3f} Nm, "
            f"pico={peak_torque:.3f} Nm"
        )

        # --- Suavizado: variación entre pasos consecutivos -------------
        diff_torque = np.diff(torque)
        if len(diff_torque) > 0:
            mean_abs_jerk = np.mean(np.abs(diff_torque))
            rms_jerk = np.sqrt(np.mean(diff_torque ** 2))
            total_variation = np.sum(np.abs(diff_torque))
            sign_changes = int(np.sum(np.diff(np.sign(torque)) != 0))
            # índice de suavidad normalizado: jerk relativo a la magnitud
            smoothness_idx = rms_jerk / rms_torque if rms_torque > 1e-9 else 0.0

            rate_str = f"{mean_abs_jerk:.4f} Nm/paso"
            if dt is not None and dt > 0:
                rate_str += f" ({mean_abs_jerk / dt:.3f} Nm/s)"

            print(
                f"suavizado: "
                f"\nΔmedio={rate_str}, "
                f"\nΔ_RMS={rms_jerk:.4f} Nm/paso, "
                f"\nvar. total={total_variation:.3f} Nm, "
                f"\níndice suavidad (Δ_RMS/RMS)={smoothness_idx:.4f}, "
                f"\ncambios de signo={sign_changes}\n"
            )


def _col(df: pd.DataFrame, name: str) -> np.ndarray | None:
    """Devuelve la columna como array o None si no existe / es todo NaN."""
    if name not in df.columns:
        return None
    arr = df[name].to_numpy(dtype=float)
    return None if np.all(np.isnan(arr)) else arr


def _time_axis(df: pd.DataFrame) -> np.ndarray:
    """Eje X en pasos (o en segundos si hay columna 'time')."""
    if "time" in df.columns:
        return df["time"].to_numpy(dtype=float)
    return np.arange(len(df))


def _xlabel(df: pd.DataFrame) -> str:
    return "Tiempo (s)" if "time" in df.columns else "Paso"


# ---------------------------------------------------------------------------
# Las 4 figuras
# ---------------------------------------------------------------------------
def plot_hip_position(ax: plt.Axes, df: pd.DataFrame, label_suffix: str = ""):
    t = _time_axis(df)
    xl = _xlabel(df)
    real_l = _col(df, "hip_pos_l")
    real_r = _col(df, "hip_pos_r")
    tgt_l = _col(df, "hip_pos_target_l")
    tgt_r = _col(df, "hip_pos_target_r")

    if real_l is not None:
        ax.plot(t, np.degrees(real_l), color=LEFT_COLOR, label=f"Real L{label_suffix}")
    if real_r is not None:
        ax.plot(t, np.degrees(real_r), color=RIGHT_COLOR, label=f"Real R{label_suffix}")
    if tgt_l is not None:
        ax.plot(t, np.degrees(tgt_l), color=LEFT_COLOR, label=f"Target L{label_suffix}", ls=TARGET_LS, alpha=TARGET_ALPHA)
    if tgt_r is not None:
        ax.plot(t, np.degrees(tgt_r), color=RIGHT_COLOR, label=f"Target R{label_suffix}", ls=TARGET_LS, alpha=TARGET_ALPHA)

    ax.set_title("Posición de cadera")
    ax.set_xlabel(xl)
    ax.set_ylabel("Ángulo (°)")
    ax.legend(ncol=2)


def plot_hip_velocity(ax: plt.Axes, df: pd.DataFrame, label_suffix: str = ""):
    t = _time_axis(df)
    xl = _xlabel(df)
    real_l = _col(df, "hip_vel_l")
    real_r = _col(df, "hip_vel_r")
    tgt_l = _col(df, "hip_vel_target_l")
    tgt_r = _col(df, "hip_vel_target_r")

    if real_l is not None:
        ax.plot(t, np.degrees(real_l), color=LEFT_COLOR, label=f"Real L{label_suffix}")
    if real_r is not None:
        ax.plot(t, np.degrees(real_r), color=RIGHT_COLOR, label=f"Real R{label_suffix}")
    if tgt_l is not None:
        ax.plot(t, np.degrees(tgt_l), color=LEFT_COLOR, label=f"Target L{label_suffix}", ls=TARGET_LS, alpha=TARGET_ALPHA)
    if tgt_r is not None:
        ax.plot(t, np.degrees(tgt_r), color=RIGHT_COLOR, label=f"Target R{label_suffix}", ls=TARGET_LS, alpha=TARGET_ALPHA)

    ax.set_title("Velocidad de cadera")
    ax.set_xlabel(xl)
    ax.set_ylabel("Velocidad angular (°/s)")
    ax.legend(ncol=2)


def plot_muscle_activations(ax: plt.Axes, df: pd.DataFrame, label_suffix: str = ""):
    t = _time_axis(df)
    xl = _xlabel(df)
    for col, color in MUSCLE_COLORS.items():
        arr = _col(df, col)
        if arr is not None:
            ax.plot(t, arr, color=color, label=f"{MUSCLE_LABELS[col]}{label_suffix}")

    ax.set_title("Activaciones musculares")
    ax.set_xlabel(xl)
    ax.set_ylabel("Activación (0–1)")
    ax.set_ylim(-0.02, 1.05)
    ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1, decimals=0))
    ax.legend(ncol=2)


def plot_torque(ax: plt.Axes, df: pd.DataFrame, label_suffix: str = ""):
    t = _time_axis(df)
    xl = _xlabel(df)
    for col, color in TORQUE_COLORS.items():
        arr = _col(df, col)
        side = "L" if col.endswith("_l") else "R"
        if arr is not None:
            ax.plot(t, arr, color=color, label=f"Torque exo {side}{label_suffix}")

    ax.axhline(0, color="#94A3B8", lw=0.8, ls=":")
    ax.set_title("Torque del exoesqueleto")
    ax.set_xlabel(xl)
    ax.set_ylabel("Torque (N·m)")
    ax.legend()


# ---------------------------------------------------------------------------
# Construcción del dashboard de 4 paneles
# ---------------------------------------------------------------------------
def make_dashboard(dfs: list[pd.DataFrame], labels: list[str], title: str) -> plt.Figure:
    apply_style()
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(title, fontsize=13, fontweight="bold", color="#0F172A", y=1.01)

    multi = len(dfs) > 1
    for df, lbl in zip(dfs, labels):
        suffix = f" [{lbl}]" if multi else ""
        plot_hip_position(axes[0, 0], df, suffix)
        plot_hip_velocity(axes[0, 1], df, suffix)
        plot_muscle_activations(axes[1, 0], df, suffix)
        plot_torque(axes[1, 1], df, suffix)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv_files", nargs="+", type=Path, help="Uno o más CSV de episodio")
    p.add_argument("--save", action="store_true",
                   help="Guardar las figuras como PNG junto al CSV (no abre ventana)")
    p.add_argument("--overlay", action="store_true",
                   help="Superponer todos los CSVs en el mismo panel (útil para comparar checkpoints)")
    p.add_argument("--dpi", type=int, default=150, help="Resolución de los PNGs (default: 150)")
    p.add_argument("--no-degrees", action="store_true",
                   help="Mostrar posición/velocidad en radianes en lugar de grados")
    return p.parse_args()


def main():
    args = parse_args()

    # Si --no-degrees, parchear np.degrees para no convertir
    if args.no_degrees:
        np.degrees = lambda x: x  # type: ignore[assignment]

    csv_paths = sorted(args.csv_files)
    if not csv_paths:
        print("No se encontraron archivos CSV.", file=sys.stderr)
        sys.exit(1)

    dfs: list[pd.DataFrame] = []
    labels: list[str] = []
    for p in csv_paths:
        if not p.exists():
            print(f"[WARN] No existe: {p}", file=sys.stderr)
            continue
        try:
            df = pd.read_csv(p)
            print_metrics(df, p.name)
            dfs.append(df)
            labels.append(p.stem)
            ep_list = df["episode"].unique().tolist() if "episode" in df.columns else "?"
            print(f"[OK] {p.name}  →  {len(df)} pasos, episodio(s): {ep_list}")
        except Exception as exc:
            print(f"[WARN] No se pudo leer {p}: {exc}", file=sys.stderr)

    if not dfs:
        print("Ningún CSV válido cargado.", file=sys.stderr)
        sys.exit(1)

    if args.overlay:
        # Un único dashboard con todas las series superpuestas
        title = f"Comparativa de {len(dfs)} checkpoint(s)"
        fig = make_dashboard(dfs, labels, title)
        if args.save:
            out = csv_paths[0].parent / "episode_comparison.png"
            fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
            print(f"[INFO] Guardado: {out}")
        else:
            plt.show()
    else:
        # Un dashboard por CSV
        for df, lbl, path in zip(dfs, labels, csv_paths):
            ep_info = ""
            if "episode" in df.columns:
                eps = df["episode"].unique()
                ep_info = f" — episodio {eps[0]}" if len(eps) == 1 else f" — episodios {eps[0]}–{eps[-1]}"
            title = f"{path.name}{ep_info}"
            fig = make_dashboard([df], [lbl], title)
            if args.save:
                out = path.with_suffix(".png")
                fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
                print(f"[INFO] Guardado: {out}")
                plt.close(fig)
            else:
                plt.show()
                plt.close(fig)


if __name__ == "__main__":
    main()