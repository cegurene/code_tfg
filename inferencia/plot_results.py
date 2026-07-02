import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# CONFIGURACIÓN DE RUTAS - MODIFICA ESTAS VARIABLES
# =============================================================================
RUTA_CSV_PID = '/home/carlos/Escritorio/TFG/code_tfg/outputs/inferencia/2026-06-26_10-07-12-buena_pid/telemetry.csv'
RUTA_CSV_AGENTE = '/home/carlos/Escritorio/TFG/code_tfg/outputs/inferencia/2026-06-26_10-08-06-buena_agente/telemetry.csv'
# =============================================================================

# Si tus CSV originales miden la posición en RADIANES y la velocidad en RAD/S, 
# pon esta variable en True para convertirlos automáticamente a GRADOS en el script.
# Si ya están en grados, ponla en False.
DATOS_EN_RADIANES = True 
# =============================================================================

def calcular_metricas(df, nombre_modo):
    print(f"\n=== MÉTRICAS PARA: {nombre_modo} ===")
    
    # Factor de conversión (1 rad = 57.2958 grados)
    to_deg = np.degrees(1.0) if DATOS_EN_RADIANES else 1.0
    u_pos = "°"
    u_vel = "°/s"
    
    # 1. Errores de Posición (convertidos a grados si corresponde)
    error_pos_r = (df['hip_pos_target_r'] - df['hip_pos_r']) * to_deg
    error_pos_l = (df['hip_pos_target_l'] - df['hip_pos_l']) * to_deg
    
    print(f"Error Posición Derecho   -> Medio Absoluto: {error_pos_r.abs().mean():.4f} {u_pos} | RMS: {np.sqrt((error_pos_r**2).mean()):.4f} {u_pos}")
    print(f"Error Posición Izquierdo -> Medio Absoluto: {error_pos_l.abs().mean():.4f} {u_pos} | RMS: {np.sqrt((error_pos_l**2).mean()):.4f} {u_pos}")
    
    # 2. Errores de Velocidad (convertidos a grados/s si corresponde)
    error_vel_r = (df['hip_vel_target_r'] - df['hip_vel_r']) * to_deg
    error_vel_l = (df['hip_vel_target_l'] - df['hip_vel_l']) * to_deg
    
    print(f"Error Velocidad Derecho   -> Medio Absoluto: {error_vel_r.abs().mean():.4f} {u_vel} | RMS: {np.sqrt((error_vel_r**2).mean()):.4f} {u_vel}")
    print(f"Error Velocidad Izquierdo -> Medio Absoluto: {error_vel_l.abs().mean():.4f} {u_vel} | RMS: {np.sqrt((error_vel_l**2).mean()):.4f} {u_vel}")
    
    # 3. Métricas de Torque (El torque siempre se mantiene en Nm)
    print(f"Torque Máximo (Pico) Derecho: {df['combined_torque_r'].abs().max():.4f} Nm")
    print(f"Torque Máximo (Pico) Izquierdo: {df['combined_torque_l'].abs().max():.4f} Nm")
    print(f"Torque RMS Derecho: {np.sqrt((df['combined_torque_r']**2).mean()):.4f} Nm")
    print(f"Torque RMS Izquierdo: {np.sqrt((df['combined_torque_l']**2).mean()):.4f} Nm")
    
    # Índice de suavidad (Variación del torque por paso / RMS del torque)
    diff_torque_r = df['combined_torque_r'].diff().dropna()
    diff_torque_l = df['combined_torque_l'].diff().dropna()
    
    rms_diff_r = np.sqrt((diff_torque_r**2).mean())
    rms_diff_l = np.sqrt((diff_torque_l**2).mean())
    
    rms_torque_r = np.sqrt((df['combined_torque_r']**2).mean())
    rms_torque_l = np.sqrt((df['combined_torque_l']**2).mean())
    
    print(f"Índice Suavidad Derecho (Δ_RMS/RMS): {rms_diff_r / (rms_torque_r + 1e-6):.4f}")
    print(f"Índice Suavidad Izquierdo (Δ_RMS/RMS): {rms_diff_l / (rms_torque_l + 1e-6):.4f}")
    
    return {
        'error_pos_r': error_pos_r, 'error_pos_l': error_pos_l,
        'torque_r': df['combined_torque_r'], 'torque_l': df['combined_torque_l'],
        'time': df['time_s']
    }

def generar_plots(data_pid, data_combined):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Comparativa Control Real: Solo PID vs PID + Agente RL', fontsize=14, fontweight='bold')
    
    # --- COLUMNA IZQUIERDA: CADERA DERECHA ---
    # Fila 1: Errores de Posición (Ya transformados a grados)
    axes[0, 0].plot(data_pid['time'], data_pid['error_pos_r'], label='Solo PID', alpha=0.7)
    axes[0, 0].plot(data_combined['time'], data_combined['error_pos_r'], label='PID + Agente', alpha=0.7)
    axes[0, 0].set_title('Error de Posición - Cadera Derecha')
    axes[0, 0].set_ylabel('Error (Grados °)')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    
    # Fila 2: Torque Aplicado al Motor
    axes[1, 0].plot(data_pid['time'], data_pid['torque_r'], label='Solo PID', alpha=0.7)
    axes[1, 0].plot(data_combined['time'], data_combined['torque_r'], label='PID + Agente', alpha=0.7)
    axes[1, 0].set_title('Torque Total al Motor - Cadera Derecha')
    axes[1, 0].set_ylabel('Torque (Nm)')
    axes[1, 0].grid(True)
    
    # Fila 3: Diferencia de Torque Neto
    diff_r = data_combined['torque_r'] - data_pid['torque_r'].reindex_like(data_combined['torque_r']).fillna(0)
    axes[2, 0].plot(data_combined['time'], diff_r, color='purple', alpha=0.7)
    axes[2, 0].set_title('Aporte Neto del Agente (Δ Torque Derecho)')
    axes[2, 0].set_xlabel('Tiempo (s)')
    axes[2, 0].set_ylabel('Torque Agente (Nm)')
    axes[2, 0].grid(True)
    
    # --- COLUMNA DERECHA: CADERA IZQUIERDA ---
    # Fila 1: Errores de Posición (Ya transformados a grados)
    axes[0, 1].plot(data_pid['time'], data_pid['error_pos_l'], label='Solo PID', alpha=0.7)
    axes[0, 1].plot(data_combined['time'], data_combined['error_pos_l'], label='PID + Agente', alpha=0.7)
    axes[0, 1].set_title('Error de Posición - Cadera Izquierda')
    axes[0, 1].set_ylabel('Error (Grados °)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    
    # Fila 2: Torque Aplicado al Motor
    axes[1, 1].plot(data_pid['time'], data_pid['torque_l'], label='Solo PID', alpha=0.7)
    axes[1, 1].plot(data_combined['time'], data_combined['torque_l'], label='PID + Agente', alpha=0.7)
    axes[1, 1].set_title('Torque Total al Motor - Cadera Izquierda')
    axes[1, 1].grid(True)
    
    # Fila 3: Diferencia de Torque Neto Izquierdo
    diff_l = data_combined['torque_l'] - data_pid['torque_l'].reindex_like(data_combined['torque_l']).fillna(0)
    axes[2, 1].plot(data_combined['time'], diff_l, color='purple', alpha=0.7)
    axes[2, 1].set_title('Aporte Neto del Agente (Δ Torque Izquierdo)')
    axes[2, 1].set_xlabel('Tiempo (s)')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    print("Iniciando análisis comparativo...")
    try:
        # Cargar los CSVs
        df_pid = pd.read_csv(RUTA_CSV_PID)
        df_combined = pd.read_csv(RUTA_CSV_AGENTE)
        
        # Procesar estadísticas por consola
        metrics_pid = calcular_metricas(df_pid, "SOLO PID")
        metrics_combined = calcular_metricas(df_combined, "PID + AGENTE")
        
        # Dibujar gráficas
        print("\nGenerando gráficos comparativos...")
        generar_plots(metrics_pid, metrics_combined)
        
    except FileNotFoundError as e:
        print(f"\n[ERROR] Archivo no encontrado. Verifica las rutas en el código.\n{e}")