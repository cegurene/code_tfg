import pandas as pd
import numpy as np

def analizar_metricas_grados(archivo_csv):
    # 1. Cargar los datos
    try:
        df = pd.read_csv(archivo_csv)
    except FileNotFoundError:
        return "Error: No se encontró el archivo CSV."

    df.columns = df.columns.str.strip()

    # 2. Convertir las columnas de posición de Radianes a Grados
    columnas_rad = [
        "target_right_rad",
        "actual_right_rad",
        "target_left_rad",
        "actual_left_rad",
    ]
    for col in columnas_rad:
        nombre_grado = col.replace("_rad", "_deg")
        df[nombre_grado] = np.degrees(df[col])

    metrics = {}

    # 3. Calcular Errores de Seguimiento en GRADOS
    # Lado Derecho
    error_right_deg = df["target_right_deg"] - df["actual_right_deg"]
    metrics["MAE_Right_deg"] = np.mean(np.abs(error_right_deg))
    metrics["RMSE_Right_deg"] = np.sqrt(np.mean(error_right_deg**2))

    # Lado Izquierdo
    error_left_deg = df["target_left_deg"] - df["actual_left_deg"]
    metrics["MAE_Left_deg"] = np.mean(np.abs(error_left_deg))
    metrics["RMSE_Left_deg"] = np.sqrt(np.mean(error_left_deg**2))

    # 4. Métricas de Torque (Se quedan igual, en Nm)
    metrics["Torque_Right_Max_Nm"] = df["torque_right_nm"].min()
    metrics["Torque_Right_Mean_Nm"] = df["torque_right_nm"].abs().mean()
    metrics["Torque_Left_Max_Nm"] = df["torque_left_nm"].min()
    metrics["Torque_Left_Mean_Nm"] = df["torque_left_nm"].abs().mean()

    # 5. Porcentaje de tiempo en Flexión y Extensión
    total_filas = len(df)
    metrics["Pct_Time_Right_Flex"] = (df["right_flex"].sum() / total_filas) * 100
    metrics["Pct_Time_Right_Ext"] = (df["right_ext"].sum() / total_filas) * 100
    metrics["Pct_Time_Left_Flex"] = (df["left_flex"].sum() / total_filas) * 100
    metrics["Pct_Time_Left_Ext"] = (df["left_ext"].sum() / total_filas) * 100

    #
    metrics["Torque_Right_MAD_Nm"] = df["torque_right_nm"].abs().mean()
    metrics["Torque_Left_MAD_Nm"] = df["torque_left_nm"].abs().mean()

    # 2. RMS del Torque (Root Mean Square)
    metrics["Torque_Right_RMS_Nm"] = np.sqrt(np.mean(df["torque_right_nm"] ** 2))
    metrics["Torque_Left_RMS_Nm"] = np.sqrt(np.mean(df["torque_left_nm"] ** 2))

    # 3. Activación Media de cada Actuador (Media de las señales binarias/analógicas)
    metrics["Mean_Act_Right_Flex"] = df["right_flex"].mean()
    metrics["Mean_Act_Right_Ext"] = df["right_ext"].mean()
    metrics["Mean_Act_Left_Flex"] = df["left_flex"].mean()
    metrics["Mean_Act_Left_Ext"] = df["left_ext"].mean()

    # 6. Mostrar resultados
    print("=" * 45)
    print("    REPORTE DE MÉTRICAS GENERADAS (EN GRADOS)    ")
    print("=" * 45)

    print("\n--- SEGUIMIENTO DE POSICIÓN (LADO DERECHO) ---")
    print(f"Error Absoluto Medio (MAE):    {metrics['MAE_Right_deg']:.2f}°")
    print(f"Error Cuadrático Medio (RMSE):  {metrics['RMSE_Right_deg']:.2f}°")

    print("\n--- SEGUIMIENTO DE POSICIÓN (LADO IZQUIERDO) --")
    print(f"Error Absoluto Medio (MAE):    {metrics['MAE_Left_deg']:.2f}°")
    print(f"Error Cuadrático Medio (RMSE):  {metrics['RMSE_Left_deg']:.2f}°")

    print("\n--- CINÉTICA Y TORQUE ------------------------")
    print(f"Torque Máx Derecho:            {metrics['Torque_Right_Max_Nm']:.2f} Nm")
    print(f"Torque Absoluto Promedio Derecho:       {metrics['Torque_Right_Mean_Nm']:.2f} Nm")
    print(f"Torque Máx Izquierdo:          {metrics['Torque_Left_Max_Nm']:.2f} Nm")
    print(f"Torque Absoluto Promedio Izquierdo:     {metrics['Torque_Left_Mean_Nm']:.2f} Nm")

    print("\n--- ACTIVACIÓN DE ESTADOS (% DEL TIEMPO) -----")
    print(f"Derecha - Flexión:             {metrics['Pct_Time_Right_Flex']:.1f}%")
    print(f"Derecha - Extensión:           {metrics['Pct_Time_Right_Ext']:.1f}%")
    print(f"Izquierda - Flexión:           {metrics['Pct_Time_Left_Flex']:.1f}%")
    print(f"Izquierda - Extensión:         {metrics['Pct_Time_Left_Ext']:.1f}%")
    print("=" * 45)
    print("=" * 50)
    print("         REPORTE DE MÉTRICAS AVANZADAS        ")
    print("=" * 50)

    print("\n--- CINÉTICA DE CADERA DERECHA ----------------")
    print(f"Torque Absoluto Medio:         {metrics['Torque_Right_MAD_Nm']:.2f} Nm")
    print(f"Torque RMS:                    {metrics['Torque_Right_RMS_Nm']:.2f} Nm")

    print("\n--- CINÉTICA DE CADERA IZQUIERDA --------------")
    print(f"Torque Absoluto Medio:         {metrics['Torque_Left_MAD_Nm']:.2f} Nm")
    print(f"Torque RMS:                    {metrics['Torque_Left_RMS_Nm']:.2f} Nm")

    print("\n--- ACTIVACIÓN MEDIA DE ACTUADORES ------------")
    print(f"Actuador Flexor Derecho:       {metrics['Mean_Act_Right_Flex']:.4f}")
    print(f"Actuador Extensor Derecho:     {metrics['Mean_Act_Right_Ext']:.4f}")
    print(f"Actuador Flexor Izquierdo:     {metrics['Mean_Act_Left_Flex']:.4f}")
    print(f"Actuador Extensor Izquierdo:   {metrics['Mean_Act_Left_Ext']:.4f}")
    print("=" * 50)

    return metrics


# --- CÓMO EJECUTAR EL SCRIPT ---
# Reemplaza 'tus_datos.csv' por el nombre real de tu archivo
analizar_metricas_grados('/home/carlos/Escritorio/MIMo/results/new_workflow/simulator/2026-05-21_12-24-16_trajectory_tracking/hip_tracking_telemetry.csv')