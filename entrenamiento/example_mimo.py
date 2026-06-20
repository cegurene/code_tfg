import gymnasium as gym
import sconegym
import numpy as np

# create the sconegym env
env = gym.make("nair_gait_h0404MimoExo-v0", use_delayed_sensors=True)
env.reset()
action = np.array([0, 0])
print("sample action", action)
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
    print("obs", next_state)
    #print("reward", reward)
    #print("done", done)
    #print("truncated", truncated)
    print("info", info)

    # Save plotting data from the info dict returned by the environment.
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
    if done or (ep_steps >= 50):
        env.unwrapped.write_now()
        print(
            f"Episode {0} ending; steps={ep_steps}; reward={reward:0.3f}; \
            com={env.unwrapped.model.com_pos()}"
        )

        break

# Plot all signals in one figure.
import matplotlib.pyplot as plt

dof_names = info.get("dofs_names", [])
actuator_names = info.get("actuators_names", info.get("muscles_names", []))
fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)

# 1) Reference vs target trajectories.
if reference_csv is not None:
    if reference_csv.ndim == 1:
        axes[0].plot(reference_csv, label='Reference Hip Trajectory')
    elif reference_csv.ndim == 2:
        axes[0].plot(reference_csv[:, 0], label='Reference Right Hip')
        if reference_csv.shape[1] > 1:
            axes[0].plot(reference_csv[:, 1], label='Reference Left Hip')

if target_pos_history:
    target_pos_array = np.vstack(target_pos_history)
    axes[0].plot(target_pos_array[:, 0], label='Target Right Hip', linestyle='--')
    axes[0].plot(target_pos_array[:, 1], label='Target Left Hip', linestyle='--')

if err_right_history:
    axes[0].plot(err_right_history, label='err_right', linestyle=':', linewidth=1.5)
if err_left_history:
    axes[0].plot(err_left_history, label='err_left', linestyle=':', linewidth=1.5)

axes[0].set_ylabel('Hip Angle (rad)')
axes[0].set_title('Reference vs Target Hip Angles + Error')
axes[0].grid(True)
axes[0].legend(loc='upper right', ncol=2, fontsize=8)

# 2) Joint positions + target.
if joint_positions_history:
    joint_positions_array = np.vstack(joint_positions_history)
    for idx in range(joint_positions_array.shape[1]): # joint_positions_array.shape[1]
        label = dof_names[idx] if idx < len(dof_names) else f"dof_pos_{idx}"
        if "exo" in label.lower():
            continue
        axes[1].plot(joint_positions_array[:, idx], label=label)

if target_pos_history:
    target_pos_array = np.vstack(target_pos_history)
    axes[1].plot(target_pos_array[:, 0], label='target_right', linestyle='--', linewidth=1.5)
    axes[1].plot(target_pos_array[:, 1], label='target_left', linestyle='--', linewidth=1.5)

axes[1].set_ylabel('Position (rad)')
axes[1].set_title('Joint Positions + Target')
axes[1].grid(True)
axes[1].legend(loc='upper right', ncol=2, fontsize=8)

# 3) Joint velocities.
if joint_velocities_history:
    joint_velocities_array = np.vstack(joint_velocities_history)
    for idx in range(joint_velocities_array.shape[1]): # joint_velocities_array.shape[1]
        label = dof_names[idx] if idx < len(dof_names) else f"dof_vel_{idx}"
        axes[2].plot(joint_velocities_array[:, idx], label=label)
axes[2].set_ylabel('Velocity (rad/s)')
axes[2].set_title('Joint Velocities')
axes[2].grid(True)
axes[2].legend(loc='upper right', ncol=2, fontsize=8)

# 4) Actuator activations.
if muscle_activations_history:
    muscle_activations_array = np.vstack(muscle_activations_history)
    for idx in range(muscle_activations_array.shape[1]):
        label = actuator_names[idx] if idx < len(actuator_names) else f"act_{idx}"
        axes[3].plot(muscle_activations_array[:, idx], label=label)
axes[3].set_xlabel('Time Steps')
axes[3].set_ylabel('Activation')
axes[3].set_title('Actuator Activations')
axes[3].grid(True)
axes[3].legend(loc='upper right', ncol=2, fontsize=8)

# 5) PID outputs from info.
if pid_r_history:
    axes[4].plot(pid_r_history, label='pid_r')
if pid_l_history:
    axes[4].plot(pid_l_history, label='pid_l')
axes[4].set_xlabel('Time Steps')
axes[4].set_ylabel('PID Output')
axes[4].set_title('PID Outputs')
axes[4].grid(True)
axes[4].legend(loc='upper right', ncol=2, fontsize=8)

plt.tight_layout()
plt.show()
