import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_csv_run(csv_path: str, plot_path: str = None, show_plot: bool = True) -> None:
    """
    Lee el nuevo CSV y genera una matriz de 3x2 gráficos:
    - Fila 1: Posición Cadera Izquierda y Derecha (Real vs Objetivo)
    - Fila 2: Velocidad Cadera Izquierda y Derecha (Real vs Objetivo)
    - Fila 3: Torque PID Izquierdo y Derecho
    """
    # 1. Cargar los datos desde el CSV
    df = pd.read_csv(csv_path)
    
    # Extraer las variables necesarias
    time_s = df['time_s'].to_numpy()
    
    # Posiciones
    hip_pos_target_r = df['hip_pos_target_r'].to_numpy()
    hip_pos_target_l = df['hip_pos_target_l'].to_numpy()
    hip_pos_r = df['hip_pos_r'].to_numpy()
    hip_pos_l = df['hip_pos_l'].to_numpy()
    
    # Velocidades
    hip_vel_target_r = df['hip_vel_target_r'].to_numpy()
    hip_vel_target_l = df['hip_vel_target_l'].to_numpy()
    hip_vel_r = df['hip_vel_r'].to_numpy()
    hip_vel_l = df['hip_vel_l'].to_numpy()
    
    # Torques PID
    pid_output_r = df['pid_output_r'].to_numpy()
    pid_output_l = df['pid_output_l'].to_numpy()

    # 2. Configurar la matriz de gráficos (3 filas, 2 columnas)
    # Aumentamos el figsize vertical a 12 para que no queden las gráficas muy aplastadas
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), sharex='col')

    # Desempaquetamos la matriz de ejes (axes[fila, columna])
    # FILA 0: Posiciones
    ax_pos_l = axes[0, 0] 
    ax_pos_r = axes[0, 1] 
    
    # FILA 1: Velocidades
    ax_vel_l = axes[1, 0]
    ax_vel_r = axes[1, 1]
    
    # FILA 2: Torques
    ax_pid_l = axes[2, 0] 
    ax_pid_r = axes[2, 1] 

    # ================= FILA 0: POSICIONES =================
    # Izquierda
    ax_pos_l.plot(time_s, hip_pos_target_l, color="#c43c39", linewidth=2, label="Izquierda objetivo")
    ax_pos_l.plot(time_s, hip_pos_l, color="#1f5c9a", linewidth=2.0, label="Izquierda real")
    ax_pos_l.set_ylabel("Ángulo [rad]")
    ax_pos_l.set_title("Posición Cadera Izquierda")
    ax_pos_l.grid(True, alpha=0.25)
    ax_pos_l.legend(loc="best", frameon=False)
    ax_pos_l.invert_yaxis()

    # Derecha
    ax_pos_r.plot(time_s, hip_pos_target_r, color="#c43c39", linewidth=2, label="Derecha objetivo")
    ax_pos_r.plot(time_s, hip_pos_r, color="#1f5c9a", linewidth=2.0, label="Derecha real")
    ax_pos_r.set_title("Posición Cadera Derecha")
    ax_pos_r.grid(True, alpha=0.25)
    ax_pos_r.legend(loc="best", frameon=False)

    # ================= FILA 1: VELOCIDADES =================
    # Izquierda
    ax_vel_l.plot(time_s, hip_vel_target_l, color="#c43c39", linewidth=1.2, label="Izquierda vel. objetivo")
    ax_vel_l.plot(time_s, hip_vel_l, color="#1f5c9a", linewidth=1.2, label="Izquierda vel. real")
    ax_vel_l.set_ylabel("Velocidad [rad/s]")
    ax_vel_l.set_title("Velocidad Cadera Izquierda")
    ax_vel_l.grid(True, alpha=0.25)
    ax_vel_l.legend(loc="best", frameon=False)
    ax_vel_l.invert_yaxis()

    # Derecha
    ax_vel_r.plot(time_s, hip_vel_target_r, color="#c43c39", linewidth=1.2, label="Derecha vel. objetivo")
    ax_vel_r.plot(time_s, hip_vel_r, color="#1f5c9a", linewidth=1.2, label="Derecha vel. real")
    ax_vel_r.set_title("Velocidad Cadera Derecha")
    ax_vel_r.grid(True, alpha=0.25)
    ax_vel_r.legend(loc="best", frameon=False)

    # ================= FILA 2: TORQUES PID =================
    # Izquierda
    ax_pid_l.plot(time_s, pid_output_l, color="#2b8a3e", linewidth=2, label="PID izquierdo")
    ax_pid_l.set_xlabel("Tiempo [s]")
    ax_pid_l.set_ylabel("Torque [Nm]")
    ax_pid_l.set_title("Torque PID Cadera Izquierda")
    ax_pid_l.grid(True, alpha=0.25)
    ax_pid_l.legend(loc="best", frameon=False)
    ax_pid_l.invert_yaxis()

    # Derecha
    ax_pid_r.plot(time_s, pid_output_r, color="#2b8a3e", linewidth=2, label="PID derecho")
    ax_pid_r.set_xlabel("Tiempo [s]")
    ax_pid_r.set_title("Torque PID Cadera Derecha")
    ax_pid_r.grid(True, alpha=0.25)
    ax_pid_r.legend(loc="best", frameon=False)

    # 3. Guardado condicional y visualización
    plt.tight_layout()
    
    if plot_path is not None:
        os.makedirs(os.path.dirname(plot_path) or ".", exist_ok=True)
        plt.savefig(plot_path, dpi=220)
    
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

# Ejecución
plot_csv_run("/home/carlos/Escritorio/TFG/code_tfg/outputs/inferencia/2026-06-26_10-07-12-buena_pid/telemetry.csv")