import numpy as np
import pandas as pd


def analizar_metricas_robotica(archivo_csv):
    # 1. Cargar los datos
    try:
        df = pd.read_csv(archivo_csv)
    except FileNotFoundError:
        return "Error: No se encontró el archivo CSV."

    df.columns = df.columns.str.strip()

    columnas_esperadas = ["time_s", "q_rad", "qd_rad_s", "q_des_rad", "applied_torque_nm"]
    faltantes = [c for c in columnas_esperadas if c not in df.columns]
    if faltantes:
        return f"Error: faltan columnas en el CSV: {faltantes}"

    # 2. Convertir posiciones de Radianes a Grados
    df["q_deg"] = np.degrees(df["q_rad"])
    df["q_des_deg"] = np.degrees(df["q_des_rad"])

    metrics = {}

    # 3. Calcular Errores de Seguimiento (en GRADOS)
    error_deg = df["q_des_deg"] - df["q_deg"]
    metrics["MAE_deg"] = np.mean(np.abs(error_deg))
    metrics["RMSE_deg"] = np.sqrt(np.mean(error_deg**2))

    # 4. Métricas de Torque Estándar (en Nm)
    metrics["Torque_Max_Nm"] = df["applied_torque_nm"].abs().max()
    metrics["Torque_Mean_Nm"] = df["applied_torque_nm"].mean()

    # 5. Métricas de Torque Avanzadas (Absoluto y RMS en Nm)
    metrics["Torque_MAD_Nm"] = df["applied_torque_nm"].abs().mean()
    metrics["Torque_RMS_Nm"] = np.sqrt(np.mean(df["applied_torque_nm"] ** 2))

    # 6. Métricas extra de velocidad (ya que tenemos qd_rad_s disponible)
    metrics["Vel_Max_rad_s"] = df["qd_rad_s"].abs().max()
    metrics["Vel_Mean_rad_s"] = df["qd_rad_s"].mean()
    metrics["Vel_RMS_rad_s"] = np.sqrt(np.mean(df["qd_rad_s"] ** 2))

    # ==========================================
    # REPORTE EN TERMINAL
    # ==========================================
    print("=" * 50)
    print("    REPORTE DE MÉTRICAS GENERADAS (NUEVO CSV)   ")
    print("=" * 50)

    print("\n--- SEGUIMIENTO DE POSICIÓN ---")
    print(f"Error Absoluto Medio (MAE):    {metrics['MAE_deg']:.2f}°")
    print(f"Error Cuadrático Medio (RMSE): {metrics['RMSE_deg']:.2f}°")

    print("\n--- CINÉTICA (TORQUE) ---")
    print(f"Torque Máximo:                 {metrics['Torque_Max_Nm']:.2f} Nm")
    print(f"Torque Promedio Neto:          {metrics['Torque_Mean_Nm']:.2f} Nm")
    print(f"Torque Absoluto Medio:         {metrics['Torque_MAD_Nm']:.2f} Nm")
    print(f"Torque RMS:                    {metrics['Torque_RMS_Nm']:.2f} Nm")

    print("\n--- VELOCIDAD ARTICULAR ---")
    print(f"Velocidad Máxima (abs):        {metrics['Vel_Max_rad_s']:.2f} rad/s")
    print(f"Velocidad Promedio:            {metrics['Vel_Mean_rad_s']:.2f} rad/s")
    print(f"Velocidad RMS:                 {metrics['Vel_RMS_rad_s']:.2f} rad/s")

    print("=" * 50)

    return metrics


# Para ejecutarlo, descomenta la línea de abajo y pon el nombre de tu archivo:
if __name__ == "__main__":
    analizar_metricas_robotica(
        "/home/carlos/Escritorio/MIMo/results/new_workflow/runner/2026-05-21_12-37-27_id=11/telemetry.csv"
    )