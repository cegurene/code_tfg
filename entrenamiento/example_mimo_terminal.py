import gymnasium as gym
import sconegym
import numpy as np

# create the sconegym env
env = gym.make("nair_gait_h0404MimoExo-v0", use_delayed_sensors=True)
env.reset()
action = np.array([0, 0])
ep_steps = 0
reference_csv = None
target_pos_history = []
joint_positions_history = []
joint_velocities_history = []
muscle_activations_history = []
pid_r_history = []
pid_l_history = []
err_right_history = []
err_left_history = []
env.unwrapped.store_next_episode()  

while True:
    # applies action and advances environment by one step
    next_state, reward, done, truncated, info = env.step(action)
    ep_steps += 1

    # Save data from the info dict returned by the environment.
    ref = np.asarray(info.get("reference_csv", []), dtype=float)
    if reference_csv is None and ref.size > 0:
        reference_csv = ref

    target_pos = np.asarray(info.get("target_pos", []), dtype=float)
    if target_pos.size == 2:
        target_pos_history.append(target_pos)

    pid_r = info.get("pid_r", None)
    pid_l = info.get("pid_l", None)
    err_right = info.get("err_right", None)
    err_left = info.get("err_left", None)
    if pid_r is not None:
        pid_r_history.append(float(pid_r))
    if pid_l is not None:
        pid_l_history.append(float(pid_l))
    if err_right is not None:
        err_right_history.append(float(err_right))
    if err_left is not None:
        err_left_history.append(float(err_left))

    dof_names = info.get("dofs_names", [])
    actuator_names = info.get("actuators_names", info.get("muscles_names", []))
    positions = np.asarray(info.get("joint_positions", []), dtype=float)
    velocities = np.asarray(info.get("joint_velocities", []), dtype=float)
    activations = np.asarray(info.get("muscle_activations", []), dtype=float)

    if positions.size:
        joint_positions_history.append(positions)
    if velocities.size:
        joint_velocities_history.append(velocities)
    if activations.size:
        muscle_activations_history.append(activations)

    # check if done
    if done or (ep_steps >= 500):
        env.unwrapped.write_now()
        break

# =====================================================================
# IMPRESIÓN DE MÉTRICAS EN TERMINAL (EN GRADOS)
# =====================================================================
print("\n" + "="*70)
print("             MÉTRICAS RESUMEN DEL EPISODIO (EN GRADOS)")
print("="*70)
print(f"Pasos totales del episodio: {ep_steps}")
print(f"Recompensa del último paso: {reward:0.3f}")

# 1) Activaciones medias en porcentaje de los actuadores (no cambia, ya es %)
if muscle_activations_history:
    muscle_activations_array = np.vstack(muscle_activations_history)
    print("\n[1] Activaciones Medias de los Actuadores (%):")
    for idx in range(muscle_activations_array.shape[1]):
        label = actuator_names[idx] if idx < len(actuator_names) else f"act_{idx}"
        mean_act_pct = np.mean(muscle_activations_array[:, idx]) * 100
        print(f"    - {label}: {mean_act_pct:.2f}%")
else:
    print("\n[1] No se encontraron datos de activaciones de actuadores.")

# 2) Errores de posición de cada cadera con respecto a la trayectoria objetivo (Convertido a Grados)
print("\n[2] Errores de Posición de las Caderas (en grados °):")

# Para el NRMSE necesitamos el rango real de movimiento (Max - Min de la posición de la cadera)
# Busquemos los índices de las caderas en dof_names
idx_hip_r = next((i for i, name in enumerate(dof_names) if "hip_r" in name.lower() or "cadera_der" in name.lower()), None)
idx_hip_l = next((i for i, name in enumerate(dof_names) if "hip_l" in name.lower() or "cadera_izq" in name.lower()), None)

joint_positions_array = np.vstack(joint_positions_history) if joint_positions_history else None

# --- CADERA DERECHA ---
if err_right_history:
    # Convertimos el histórico de errores a grados (este es el error absoluto por paso)
    err_r_deg = np.degrees(np.abs(err_right_history))
    
    # 1. MAE y Máximo (Ya los tenías)
    mae_r = np.mean(err_r_deg)
    max_r = np.max(err_r_deg)
    
    # 2. MSE (Error Cuadrático Medio) -> El absoluto al cuadrado es igual al error al cuadrado
    mse_r = np.mean(err_r_deg ** 2)
    
    # 3. RMSE (Raíz del Error Cuadrático Medio)
    rmse_r = np.sqrt(mse_r)
    
    # 4. NRMSE (RMSE Normalizado)
    if joint_positions_array is not None and idx_hip_r is not None:
        # Pasamos la posición real de la cadera derecha a grados
        hip_r_pos_deg = np.degrees(joint_positions_array[:, idx_hip_r])
        rango_r = np.max(hip_r_pos_deg) - np.min(hip_r_pos_deg)
        # Evitamos división por cero si la articulación no se movió nada
        nrmse_r = (rmse_r / rango_r) * 100 if rango_r > 0 else 0.0
        nrmse_str_r = f"{nrmse_r:.2f}%"
    else:
        nrmse_str_r = "N/A (Faltan posiciones de referencia)"

    print(f"    - Cadera Derecha:")
    print(f"        MAE:  {mae_r:.4f}°  | Máximo Absoluto: {max_r:.4f}°")
    print(f"        MSE:  {mse_r:.4f}°² | RMSE:            {rmse_r:.4f}°")
    print(f"        NRMSE: {nrmse_str_r}")
else:
    print("    - Cadera Derecha -> No hay datos de error registrados.")

# --- CADERA IZQUIERDA ---
if err_left_history:
    # Convertimos el histórico de errores a grados
    err_l_deg = np.degrees(np.abs(err_left_history))
    
    # 1. MAE y Máximo
    mae_l = np.mean(err_l_deg)
    max_l = np.max(err_l_deg)
    
    # 2. MSE
    mse_l = np.mean(err_l_deg ** 2)
    
    # 3. RMSE
    rmse_l = np.sqrt(mse_l)
    
    # 4. NRMSE
    if joint_positions_array is not None and idx_hip_l is not None:
        # Pasamos la posición real de la cadera izquierda a grados
        hip_l_pos_deg = np.degrees(joint_positions_array[:, idx_hip_l])
        rango_l = np.max(hip_l_pos_deg) - np.min(hip_l_pos_deg)
        nrmse_l = (rmse_l / rango_l) * 100 if rango_l > 0 else 0.0
        nrmse_str_l = f"{nrmse_l:.2f}%"
    else:
        nrmse_str_l = "N/A (Faltan posiciones de referencia)"

    print(f"    - Cadera Izquierda:")
    print(f"        MAE:  {mae_l:.4f}°  | Máximo Absoluto: {max_l:.4f}°")
    print(f"        MSE:  {mse_l:.4f}°² | RMSE:            {rmse_l:.4f}°")
    print(f"        NRMSE: {nrmse_str_l}")
else:
    print("    - Cadera Izquierda -> No hay datos de error registrados.")
    
# 3) Velocidad angular de cada cadera y articulación (Convertido a Grados/s)
if joint_velocities_history:
    joint_velocities_array = np.vstack(joint_velocities_history)
    print("\n[3] Velocidades Angulares de las Articulaciones (grados/s):")
    for idx in range(joint_velocities_array.shape[1]):
        label = dof_names[idx] if idx < len(dof_names) else f"dof_vel_{idx}"
        
        # Convertimos todo el vector de velocidad de la articulación actual a grados/s
        vel_data_deg = np.degrees(joint_velocities_array[:, idx])
        
        # Métricas promedio tradicionales
        mean_vel = np.mean(vel_data_deg)
        mean_abs_vel = np.mean(np.abs(vel_data_deg)) # (Este es el MAE si tu objetivo de velocidad fuera 0)
        max_abs_vel = np.max(np.abs(vel_data_deg))
        
        # --- NUEVAS MÉTRICAS: MSE y RMSE ---
        # Nota: Como el objetivo implícito suele ser medir la magnitud de la energía/movimiento,
        # calculamos el MSE y RMSE respecto a cero (frenado total) o simplemente como métrica cuadrática de la señal.
        mse_vel = np.mean(vel_data_deg ** 2)
        rmse_vel = np.sqrt(mse_vel)
        
        # Identificar si la articulación pertenece a la cadera para resaltarla
        es_cadera = "hip" in label.lower() or "cadera" in label.lower()
        marca = " <-- CADERA (Hip)" if es_cadera else ""
        
        print(f"    - {label}{marca}:")
        print(f"        Media: {mean_vel:.4f}°/s | Media Absoluta (MAE): {mean_abs_vel:.4f}°/s | Máxima Absoluta: {max_abs_vel:.4f}°/s")
        print(f"        MSE:   {mse_vel:.4f}(°/s)² | RMSE:                  {rmse_vel:.4f}°/s")
else:
    print("\n[3] No se encontraron datos de velocidades articulares.")

print("\n" + "="*70 + "\n")