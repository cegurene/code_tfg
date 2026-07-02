import gymnasium as gym
import sconegym
import numpy as np
import matplotlib.pyplot as plt

# create the sconegym env
env = gym.make("nair_gait_h0404MimoExo-v2", use_delayed_sensors=True)
env.reset()
action = np.array([0, 0])
print("sample action", action)
ep_steps = 0
reference_csv = None
target_pos_history = []
joint_positions_history = []
joint_velocities_history = []
muscle_activations_history = []
err_right_history = []
err_left_history = []
env.unwrapped.store_next_episode()  

while True:
    # applies action and advances environment by one step
    next_state, reward, done, truncated, info = env.step(action)
    ep_steps += 1

    # Save plotting data from the info dict returned by the environment.
    ref = np.asarray(info.get("reference_csv", []), dtype=float)
    if reference_csv is None and ref.size > 0:
        reference_csv = ref

    target_pos = np.asarray(info.get("target_pos", []), dtype=float)
    if target_pos.size == 2:
        target_pos_history.append(target_pos)

    err_right = info.get("err_right", None)
    err_left = info.get("err_left", None)
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

# Plot all signals in one figure (Reducido a 3 filas)
dof_names = info.get("dofs_names", [])
actuator_names = info.get("actuators_names", info.get("muscles_names", []))
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# =====================================================================
# 1) Joint positions + target (CONVERTIDO A GRADOS)
# =====================================================================
if joint_positions_history:
    joint_positions_array = np.degrees(np.vstack(joint_positions_history))  
    for idx in range(joint_positions_array.shape[1]):
        label = dof_names[idx] if idx < len(dof_names) else f"dof_pos_{idx}"
        if "exo" in label.lower():
            continue
        axes[0].plot(joint_positions_array[:, idx], label=label)

if target_pos_history:
    target_pos_array = np.degrees(np.vstack(target_pos_history))
    axes[0].plot(target_pos_array[:, 0], label='Objetivo Derecha', linestyle='--', linewidth=1.5)
    axes[0].plot(target_pos_array[:, 1], label='Objetivo Izquierda', linestyle='--', linewidth=1.5)

axes[0].set_ylabel('Posición (°)')
axes[0].set_title('Posición articular + Objetivo')
axes[0].grid(True)
axes[0].legend(loc='upper right', ncol=2, fontsize=8)

# =====================================================================
# 2) Joint velocities (CONVERTIDO A GRADOS/S - SIN HIP REALS + HIP TARGETS)
# =====================================================================
if joint_velocities_history:
    joint_velocities_array = np.degrees(np.vstack(joint_velocities_history))  
    for idx in range(joint_velocities_array.shape[1]):
        label = dof_names[idx] if idx < len(dof_names) else f"dof_vel_{idx}"
        
        # Excluir las caderas reales
        if "hip" in label.lower() or "cadera" in label.lower():
            continue
            
        axes[1].plot(joint_velocities_array[:, idx], label=label)

# Cálculo y ploteo de las velocidades objetivo de cadera
if target_pos_history:
    target_pos_array = np.degrees(np.vstack(target_pos_history))
    dt = 0.01 
    
    target_vel_r = np.diff(target_pos_array[:, 0]) / dt
    target_vel_l = np.diff(target_pos_array[:, 1]) / dt
    
    axes[1].plot(target_vel_r, label='Vel. Objetivo Cadera Der', linestyle='--', color='red', linewidth=1.5)
    axes[1].plot(target_vel_l, label='Vel. Objetivo Cadera Izq', linestyle='--', color='blue', linewidth=1.5)

axes[1].set_ylabel('Velocidad (°/s)')
axes[1].set_title('Velocidades articulares (Sin Hip Reales + Hip Objetivos)')
axes[1].grid(True)
axes[1].legend(loc='upper right', ncol=2, fontsize=8)

# =====================================================================
# 3) Actuator activations
# =====================================================================
if muscle_activations_history:
    muscle_activations_array = np.vstack(muscle_activations_history)
    for idx in range(muscle_activations_array.shape[1]):
        label = actuator_names[idx] if idx < len(actuator_names) else f"act_{idx}"
        axes[2].plot(muscle_activations_array[:, idx], label=label)
axes[2].set_xlabel('Time Steps')  # Ahora este es el eje X final
axes[2].set_ylabel('Activación')
axes[2].set_title('Activación de Actuadores')
axes[2].grid(True)
axes[2].legend(loc='upper right', ncol=2, fontsize=8)

plt.tight_layout()
plt.show()