import numpy as np
import pandas as pd


def analizar_metricas_robotica(archivo_csv):
    # 1. Cargar los datos
    try:
        df = pd.read_csv(archivo_csv)
    except FileNotFoundError:
        return "Error: No se encontró el archivo CSV."

    df.columns = df.columns.str.strip()

    # 2. Convertir posiciones de Radianes a Grados
    # q = actual, q_des = target
    df["q_l_deg"] = np.degrees(df["q_l_rad"])
    df["q_r_deg"] = np.degrees(df["q_r_rad"])
    df["q_des_l_deg"] = np.degrees(df["q_des_l_rad"])
    df["q_des_r_deg"] = np.degrees(df["q_des_r_rad"])

    metrics = {}

    # 3. Calcular Errores de Seguimiento (en GRADOS)
    # Lado Derecho (Right)
    error_right_deg = df["q_des_r_deg"] - df["q_r_deg"]
    metrics["MAE_Right_deg"] = np.mean(np.abs(error_right_deg))
    metrics["RMSE_Right_deg"] = np.sqrt(np.mean(error_right_deg**2))

    # Lado Izquierdo (Left)
    error_left_deg = df["q_des_l_deg"] - df["q_l_deg"]
    metrics["MAE_Left_deg"] = np.mean(np.abs(error_left_deg))
    metrics["RMSE_Left_deg"] = np.sqrt(np.mean(error_left_deg**2))

    # 4. Métricas de Torque Estándar (en Nm)
    metrics["Torque_Right_Max_Nm"] = df["applied_torque_r_nm"].abs().max()
    metrics["Torque_Right_Mean_Nm"] = df["applied_torque_r_nm"].mean()
    metrics["Torque_Left_Max_Nm"] = df["applied_torque_l_nm"].abs().max()
    metrics["Torque_Left_Mean_Nm"] = df["applied_torque_l_nm"].mean()

    # 5. Métricas de Torque Avanzadas (Absoluto y RMS en Nm)
    metrics["Torque_Right_MAD_Nm"] = df["applied_torque_r_nm"].abs().mean()
    metrics["Torque_Right_RMS_Nm"] = np.sqrt(
        np.mean(df["applied_torque_r_nm"] ** 2)
    )

    metrics["Torque_Left_MAD_Nm"] = df["applied_torque_l_nm"].abs().mean()
    metrics["Torque_Left_RMS_Nm"] = np.sqrt(
        np.mean(df["applied_torque_l_nm"] ** 2)
    )

    # ==========================================
    # REPORTE EN TERMINAL
    # ==========================================
    print("=" * 50)
    print("    REPORTE DE MÉTRICAS GENERADAS (NUEVO CSV)   ")
    print("=" * 50)

    print("\n--- SEGUIMIENTO DE POSICIÓN (LADO DERECHO) ---")
    print(f"Error Absoluto Medio (MAE):    {metrics['MAE_Right_deg']:.2f}°")
    print(f"Error Cuadrático Medio (RMSE):  {metrics['RMSE_Right_deg']:.2f}°")

    print("\n--- SEGUIMIENTO DE POSICIÓN (LADO IZQUIERDO) --")
    print(f"Error Absoluto Medio (MAE):    {metrics['MAE_Left_deg']:.2f}°")
    print(f"Error Cuadrático Medio (RMSE):  {metrics['RMSE_Left_deg']:.2f}°")

    print("\n--- CINÉTICA DE CADERA DERECHA ----------------")
    print(f"Torque Máximo:                 {metrics['Torque_Right_Max_Nm']:.2f} Nm")
    print(f"Torque Promedio Neto:          {metrics['Torque_Right_Mean_Nm']:.2f} Nm")
    print(f"Torque Absoluto Medio:         {metrics['Torque_Right_MAD_Nm']:.2f} Nm")
    print(f"Torque RMS:                    {metrics['Torque_Right_RMS_Nm']:.2f} Nm")

    print("\n--- CINÉTICA DE CADERA IZQUIERDA --------------")
    print(f"Torque Máximo:                 {metrics['Torque_Left_Max_Nm']:.2f} Nm")
    print(f"Torque Promedio Neto:          {metrics['Torque_Left_Mean_Nm']:.2f} Nm")
    print(f"Torque Absoluto Medio:         {metrics['Torque_Left_MAD_Nm']:.2f} Nm")
    print(f"Torque RMS:                    {metrics['Torque_Left_RMS_Nm']:.2f} Nm")
    print("=" * 50)

    return metrics


# Para ejecutarlo, descomenta la línea de abajo y pon el nombre de tu archivo:
analizar_metricas_robotica('/home/carlos/Escritorio/MIMo/results/new_workflow/runner/2026-05-21_12-37-27_id=11/telemetry.csv')