# NAIR_standupEnv.py

from pathlib import Path

import numpy as np
import yaml
import os, sys
import gymnasium as gym
import datetime as dt

from sconegym.nair_sconegym import NairSconeGymEnv
from sconegym.sconetools import sconepy
from sconegym.utils import utils_mtf

from typing import Optional
import sconepy # type: ignore
import collections


DEFAULT_RW_DICT = {
    "position_tracking": 0.1,
    "velocity_tracking": 0.01,
    "torque_cost": 0.0,
    "torque_smoothness": 0.02,
    "muscle_activation_cost": 1.5
}


class MimoGym(NairSconeGymEnv):
    def __init__(self, 
                 model_file, 
                 max_episode_steps=1000,
                 rwd_keys=DEFAULT_RW_DICT,
                 *args, **kwargs):
    
        if "rew_keys" in kwargs and rwd_keys is DEFAULT_RW_DICT:
            candidate_rwd_keys = kwargs.pop("rew_keys")
            if set(candidate_rwd_keys).issubset(DEFAULT_RW_DICT):
                rwd_keys = candidate_rwd_keys

        parser = utils_mtf._build_parser()
        mtf_args = parser.parse_args()
        default_reference_path = Path(__file__).resolve().parent / "healthy_hip_reference.csv"
        defaults = utils_mtf._get_default_args(str(default_reference_path))
        reference = utils_mtf._load_reference_csv(
            csv_path='/home/carlos/Escritorio/NAIR_RL/sconegym/sconegym/nair_envs/H0404_MimoExo/trajectory_refs/healthy_hip_reference.csv',
            phase_column=mtf_args.reference_phase_column,
            angle_column=mtf_args.reference_angle_column,
            angle_unit=mtf_args.reference_angle_unit,
            scale=mtf_args.trajectory_scale,
            angle_sign=mtf_args.reference_angle_sign,
        )
        reference.period_s = float(mtf_args.trajectory_period_s)
        run_tag = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        repo_root = Path(__file__).resolve().parents[2]
        run_dir = mtf_args.run_dir.strip() if mtf_args.run_dir else str(repo_root / "outputs" / "mimo" / "new_workflow" / "simulator" / run_tag)
        #os.makedirs(run_dir, exist_ok=True)

        csv_path = mtf_args.csv_path.strip() if mtf_args.csv_path else os.path.join(run_dir, "hip_tracking_telemetry.csv")
        plot_path = mtf_args.plot_path.strip() if mtf_args.plot_path else os.path.join(run_dir, "hip_tracking_summary.png")

        # Values used by get_obs() while BasicSconeGymEnv builds the spaces.
        self.reference_traj = reference
        self.left_phase_offset_pct = float(mtf_args.left_phase_offset_pct)
        self.pid_phase_offset_pct = float(mtf_args.trajectory_phase_offset_pct)
        self.debug_mimo = bool(kwargs.pop("debug_mimo", False))
        self.min_com_height = float(kwargs.pop("min_com_height", 0.5))
        self.min_head_height = float(kwargs.pop("min_head_height", 0.7))
        self.target_velocity = float(kwargs.pop("target_velocity", 1.0))

        # To do : integrate utils_mtf._apply_mtf_args_to_model to NAIR MIMO model
        """
        utils_mtf._apply_mtf_args_to_model(
            model_file,
            stiffness_scale=mtf_args.joint_stiffness_scale,
            damping_scale=mtf_args.joint_damping_scale,
            frictionloss_scale=mtf_args.joint_frictionloss_scale,
            armature_scale=mtf_args.joint_armature_scale)
        """
        super().__init__(model_file, max_episode_steps=max_episode_steps, *args, **kwargs)
        self._max_episode_steps = max_episode_steps
        self.rwd_keys = rwd_keys
        self.rwd_dict = {key: 0.0 for key in rwd_keys}
        self.mass = np.sum([x.mass() for x in self.model.bodies()])
        self._dof_names = [d.name() for d in self.model.dofs()]
        self.pelvis_pos_rw_prev = 0.0
        self.fall_time = -1.0
        self.prev_motor_action = np.zeros(self.exo_dims, dtype=np.float64)
        self.last_motor_action = np.zeros(self.exo_dims, dtype=np.float64)
        self.last_motor_torque = np.zeros(self.exo_dims, dtype=np.float64)
        self.target_right = 0.0
        self.target_left = 0.0
        self.target_vel_right = 0.0
        self.target_vel_left = 0.0
        self.err_right = 0.0
        self.err_left = 0.0
        self.vel_err_right = 0.0
        self.vel_err_left = 0.0
        self.pid_r = 0.0
        self.pid_l = 0.0

        # MTF/PID config 
        self.pid_kp = float(mtf_args.kp)
        self.pid_ki = float(mtf_args.ki)
        self.pid_kd = float(mtf_args.kd)
        self.pid_activation_scale = max(1e-9, float(mtf_args.pid_activation_scale))
        
        #Safe limits for muscles forces/torque
        self.pid_safe_torque_limit = max(1e-9, float(mtf_args.safe_torque_limit*100))
        self.hip_flx_cap_right = self.model.muscles()[0].max_isometric_force()   # Simplified torque cap based on max force 
        self.hip_flx_cap_left = self.model.muscles()[2].max_isometric_force()    # Simplified torque cap based on max force 
        self.hip_ext_cap_right = self.model.muscles()[1].max_isometric_force()   # Simplified torque cap based on max force 
        self.hip_ext_cap_left = self.model.muscles()[3].max_isometric_force()    # Simplified torque cap based on max force 
        self.act_cap_flex_r = min(1.0, self.pid_safe_torque_limit / self.hip_flx_cap_right)
        self.act_cap_flex_l = min(1.0, self.pid_safe_torque_limit / self.hip_flx_cap_left)
        self.act_cap_ext_r = min(1.0, self.pid_safe_torque_limit / self.hip_ext_cap_right)
        self.act_cap_ext_l = min(1.0, self.pid_safe_torque_limit / self.hip_ext_cap_left)
        if self.debug_mimo:
            #print(f"Calculated muscle activation caps based on safe torque limit: flex_r={self.act_cap_flex_r:.4f}, flex_l={self.act_cap_flex_l:.4f}, ext_r={self.act_cap_ext_r:.4f}, ext_l={self.act_cap_ext_l:.4f}")
            pass
        self._pid_int_err_rigth = 0.0
        self._pid_int_err_left = 0.0
        self._pid_last_time = 0.0

        # Use explicit DOF names to avoid swapping hip/exo indices when model ordering changes.
        try:
            self.dof_r_idx = self._dof_names.index("hip_flexion_r")
            self.dof_l_idx = self._dof_names.index("hip_flexion_l")
        except ValueError as exc:
            raise ValueError(f"Required hip DOFs not found in model DOFs: {self._dof_names}") from exc

        # Get current hip positions and calculate initial phases that match them
        if self.debug_mimo:
            #print("dofs positions", self.get_ms_dof_values())
            #print("dofs exo positions and velocities", self.get_obs_exo())
            pass
        all_dof_values = self.model.dof_position_array()
        self.q_right_init = float(all_dof_values[self.dof_r_idx])
        self.q_left_init = float(all_dof_values[self.dof_l_idx])

        self.pid_initial_phase_rigth = utils_mtf._find_initial_phase(reference, self.q_right_init)
        self.pid_initial_phase_left = utils_mtf._find_initial_phase(reference, self.q_left_init)
        left_start_delay_phase = utils_mtf._find_phase_delay_for_position(
        reference,
        self.q_left_init,
        self.pid_initial_phase_rigth + self.left_phase_offset_pct,
        )
        self.left_start_delay_s = left_start_delay_phase * reference.period_s / 100.0

        self.initial_com_height = float(self.model.com_pos().y)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = True,
        options: Optional[dict] = None,
    ):
        self.fall_time = -1.0
        self.prev_motor_action = np.zeros(self.exo_dims, dtype=np.float64)
        self.last_motor_action = np.zeros(self.exo_dims, dtype=np.float64)
        self.last_motor_torque = np.zeros(self.exo_dims, dtype=np.float64)
        obs, _ = super().reset()
        self._pid_int_err_rigth = 0.0
        self._pid_int_err_left = 0.0
        self._pid_last_time = float(self.model.time())
        self.initial_com_height = float(self.model.com_pos().y)
        self.rwd_dict = {key: 0.0 for key in self.rwd_keys}



        info = {
            'obs_type': self.obs_type,
            'muscles_names': [m.name() for m in self.model.muscles()],
            'dofs_names': [d.name() for d in self.model.dofs()],
            'solved': self.is_solved(),
            'obs_dict': ['acts', 'ms_length', 'ms_forces', 'dof_values', 'solved'],

        }
        return obs, info
        
    def PID_muscle_control(self):
        # Use full model DOF arrays because indexed hip DOFs are resolved on full DOF names.
        self._ensure_mimo_indices()
        dof_values = self.model.dof_position_array()
        dof_vels = self.model.dof_velocity_array()

        t_now = float(self.model.time())
        dt = float(t_now - self._pid_last_time)
        if dt <= 1e-9:
            dt = float(self.step_size)
        self._pid_last_time = t_now

        phase_right = (self.pid_initial_phase_rigth + 100.0 * t_now / self.reference_traj.period_s) % 100.0
        phase_left = (phase_right + self.left_phase_offset_pct) % 100.0

        self.target_right = utils_mtf._reference_at_phase(self.reference_traj, phase_right)
        self.target_left = self.q_left_init if t_now < self.left_start_delay_s else utils_mtf._reference_at_phase(self.reference_traj, phase_left)
        self.target_vel_right = self._reference_velocity_at_phase(phase_right)
        self.target_vel_left = 0.0 if t_now < self.left_start_delay_s else self._reference_velocity_at_phase(phase_left)
        if self.debug_mimo:
            #print(f"PID control - time: {t_now:.2f}s, phase_right: {phase_right:.2f}%, phase_left: {phase_left:.2f}%, target_right: {self.target_right:.4f}, target_left: {self.target_left:.4f}")
            pass
        q_right = float(dof_values[self.dof_r_idx])
        q_left = float(dof_values[self.dof_l_idx])
        qd_right = float(dof_vels[self.dof_r_idx])
        qd_left = float(dof_vels[self.dof_l_idx])


        # Position error for both hips
        self.err_right = -(q_right - self.target_right)
        self.err_left = -(q_left - self.target_left)
        self.vel_err_right = self.target_vel_right - qd_right
        self.vel_err_left = self.target_vel_left - qd_left

        # Integrate error
        self._pid_int_err_rigth += self.err_right * dt
        self._pid_int_err_left += self.err_left * dt

        # PID controller
        self.pid_r = self.pid_kp * self.err_right + self.pid_ki * self._pid_int_err_rigth + self.pid_kd * (0.0 - qd_right)
        self.pid_l = self.pid_kp * self.err_left + self.pid_ki * self._pid_int_err_left + self.pid_kd * (0.0 - qd_left)
        if self.debug_mimo:
            #print(f"PID control - err_right: {self.err_right:.4f}, err_left: {self.err_left:.4f}, pid_r: {self.pid_r:.4f}, pid_l: {self.pid_l:.4f}")
            pass
        # Convert PID output into normalized muscle command [-1, 1].
        # This gain is explicit and independent of safe-torque-limit.
        pid_act_scale = max(1e-9, float(self.pid_activation_scale))
        act_r_norm = float(np.clip(self.pid_r / pid_act_scale, -1.0, 1.0))
        act_l_norm = float(np.clip(self.pid_l / pid_act_scale, -1.0, 1.0))
        if self.debug_mimo:
            #print(f"PID control after activation scaling - act_r_norm: {act_r_norm:.4f}, act_l_norm: {act_l_norm:.4f}")
            pass
        # Apply safe torque as a pure clamp in equivalent torque domain.
        #act_r_norm = float(np.clip(act_r_norm, -self.act_cap_flex_r, self.act_cap_ext_r))
        #act_l_norm = float(np.clip(act_l_norm, -self.act_cap_flex_l, self.act_cap_ext_l))
        if self.debug_mimo:
            #print(f"PID control after safe torque clamp - act_r_norm: {act_r_norm:.4f}, act_l_norm: {act_l_norm:.4f}")
            pass
        action = np.zeros(self.num_muscles, dtype=np.float32)

        # Requested sign convention:
        # negative command -> flexors, positive command -> extensors.
        right_flex_cmd = max(0.0, -act_r_norm)
        right_ext_cmd = max(0.0, act_r_norm)
        left_flex_cmd = max(0.0, -act_l_norm)
        left_ext_cmd = max(0.0, act_l_norm)

        # Assumed muscle ordering for the first four muscle actuators:
        # [0]=right flexor, [1]=right extensor, [2]=left flexor, [3]=left extensor.
        idx_right_flex = 0
        idx_right_ext = 1
        idx_left_flex = 2
        idx_left_ext = 3

        # Review if the action has enough dimensions to assign the commands, otherwise skip (for compatibility with different action space sizes)
        if action.shape[0] > idx_right_flex:
            action[idx_right_flex] = right_flex_cmd
        if action.shape[0] > idx_right_ext:
            action[idx_right_ext] = right_ext_cmd
        if action.shape[0] > idx_left_flex:
            action[idx_left_flex] = left_flex_cmd   
        if action.shape[0] > idx_left_ext:
            action[idx_left_ext] = left_ext_cmd

        return action
        

    def adjust_motor_torque_limits(self, motor_action):
        
        max_torque_motor_r = self.model.actuators()[4].max_input()
        max_torque_motor_l = self.model.actuators()[5].max_input()
        torque_r = motor_action[0] * max_torque_motor_r
        torque_l = motor_action[1] * max_torque_motor_l        
        self.last_motor_torque = np.array([torque_r, torque_l], dtype=np.float64)
        return self.last_motor_torque.astype(np.float32)


    def before_step(self, action):
        if not self.has_reset:
            raise Exception("You have to call reset() once before step()")
        if self.clip_actions:
            action = np.clip(action, -1, 1)

        action = np.asarray(action, dtype=np.float64).reshape(-1)
        self.prev_motor_action = self.last_motor_action.copy()
        self.last_motor_action = action.copy()

        complete_action = np.zeros(self.num_actuators)
        complete_action[self.num_muscles:] = self.adjust_motor_torque_limits(action) # ad exo control comands after muscle commands (if any)
        if self.debug_mimo:
            #print("Action before PID control:", action)
            pass
        complete_action[0:self.num_muscles] = self.PID_muscle_control()
        #action[0] = 1
        #action[3] = 1
        if self.debug_mimo:
            #print("action after PID", complete_action)
            pass
        return complete_action  

    def step(self, action):
        action = self.before_step(action)
        if self.use_delayed_actuators:
            self.model.set_delayed_actuator_inputs(action)
        else:
            self.model.set_actuator_inputs(action)
        self.model.advance_simulation_to(self.time + self.step_size)
        self.episode_steps += 1
        reward = self.get_rew()
        obs = self.get_obs()
        done = self.is_terminal_condition()
        truncated = self.is_truncation_condition()
        self.time += self.step_size
        self.total_reward += reward

        #print("\n")
        info = {
            'actuators_names': [m.name() for m in self.model.actuators()],
            'dofs_names': [d.name() for d in self.model.dofs()],
            'solved': self.is_solved(),
            'reference_csv': self.reference_traj.angle_rad,
            'joint_positions': self.model.dof_position_array().copy(),
            'joint_velocities': self.model.dof_velocity_array().copy(),
            'muscle_activations': self.model.muscle_activation_array().copy(),
            'motor_action': self.last_motor_action.copy(),
            'motor_torque': self.last_motor_torque.copy(),
            'target_pos': [self.target_right, self.target_left],
            'target_vel': [self.target_vel_right, self.target_vel_left],
            'pid_r': self.pid_r,
            'pid_l': self.pid_l,
            'err_right': self.err_right,
            'err_left': self.err_left,
            'vel_err_right': self.vel_err_right,
            'vel_err_left': self.vel_err_left,
            'rwd_dict': {key: wt * self.rwd_dict[key] for key, wt in self.rwd_keys.items() if key in self.rwd_dict},
        }
        #print("\n")
        
        return obs, reward, done, truncated, info
    
    def get_obs(self):
        return self._get_obs_mimo()

    def _ensure_mimo_indices(self):
        if hasattr(self, "dof_r_idx") and hasattr(self, "dof_l_idx"):
            return

        self._dof_names = [d.name() for d in self.model.dofs()]
        try:
            # We get the indexes of both hips by their name in the simulator
            self.dof_r_idx = self._dof_names.index("hip_flexion_r")
            self.dof_l_idx = self._dof_names.index("hip_flexion_l")
        except ValueError as exc:
            raise ValueError(f"Required hip DOFs not found in model DOFs: {self._dof_names}") from exc

        dof_values = self.model.dof_position_array()
        self.q_right_init = float(dof_values[self.dof_r_idx])
        self.q_left_init = float(dof_values[self.dof_l_idx])
        self.pid_initial_phase_rigth = utils_mtf._find_initial_phase(self.reference_traj, self.q_right_init)
        self.pid_initial_phase_left = utils_mtf._find_initial_phase(self.reference_traj, self.q_left_init)
        start_phase = self.pid_initial_phase_rigth + self.left_phase_offset_pct
        left_delay_phase = utils_mtf._find_phase_delay_for_position(self.reference_traj, self.q_left_init, start_phase)
        self.left_start_delay_s = left_delay_phase * self.reference_traj.period_s / 100.0

    def _get_obs_mimo(self):
        self._ensure_mimo_indices()
        # Get an array of current dof positions 
        joint_positions = self.model.dof_position_array()
        #print("joint_positions", joint_positions)
        # Get an array of current dof velocities
        joint_velocities = self.model.dof_velocity_array()
        #print("joint_velocities", joint_velocities)
        # Get an array of current muscle activations
        #muscle_activations = self.model.muscle_activation_array()
        #print("muscle_activations", muscle_activations)
        # glut_max_r, iliopsoas_r, glut_max_l, iliopsoas_l
        """print("\n")
        print("\n")
        print("\n")
        print([m.name() for m in self.model.actuators()])
        print([m.input() for m in self.model.actuators()])
        print("\n")
        print("\n")
        print("\n")"""
        for i in range(len(self.model.muscles())):
            #print(f"muscle {i}:", self.model.muscles()[i].name())
            pass
        
        # Get the current simulation time [s]
        t_now = float(self.model.time())
        phase_right = (self.pid_initial_phase_rigth + 100.0 * t_now / self.reference_traj.period_s) % 100.0
        phase_left = (phase_right + self.left_phase_offset_pct) % 100.0
        target_right = utils_mtf._reference_at_phase(self.reference_traj, phase_right)
        target_left = self.q_left_init if t_now < getattr(self, "left_start_delay_s", 0.0) else utils_mtf._reference_at_phase(self.reference_traj, phase_left)
        target_vel_right = self._reference_velocity_at_phase(phase_right)
        target_vel_left = 0.0 if t_now < getattr(self, "left_start_delay_s", 0.0) else self._reference_velocity_at_phase(phase_left)

        # We get this 8 values from the 2 arrays of positions and velocities
        # The indexes are calculated in _ensure_mimo_indices()
        q_right = float(joint_positions[self.dof_r_idx])
        q_left = float(joint_positions[self.dof_l_idx])
        qd_right = float(joint_velocities[self.dof_r_idx])
        qd_left = float(joint_velocities[self.dof_l_idx])
        err_right = target_right - q_right
        err_left = target_left - q_left
        vel_err_right = target_vel_right - qd_right
        vel_err_left = target_vel_left - qd_left
        #print(f"positions hips: right={q_right:.4f}, left={q_left:.4f}, velocities hips: right={qd_right:.4f}, left={qd_left:.4f}")
        #print(f"errors: pos_right={err_right:.4f}, pos_left={err_left:.4f}, vel_err_right={vel_err_right:.4f}, vel_err_left={vel_err_left:.4f}")

        # We get the exo observations
        # Array of 4 values []
        exo_obs = self.get_obs_exo() if self.exo_dofs > 0 else np.array([], dtype=np.float64)
        #print("dofs_names", self.dofs_names) # hip_flexion_r, exo_flexion_r, hip_flexion_l, exo_flexion_l
        #print("ms_dofs", self.ms_dofs) # number of muscle dofs
        #print("exo_dofs", self.exo_dofs) # number of exo dofs
        #print("exo_obs", exo_obs)
        last_motor_action = getattr(self, "last_motor_action", np.zeros(self.exo_dims, dtype=np.float64))
        #print("last_motor_action", last_motor_action)

        return np.concatenate(
            [
                np.array(
                    [
                        np.sin(2.0 * np.pi * phase_right / 100.0),
                        np.cos(2.0 * np.pi * phase_right / 100.0),
                        np.sin(2.0 * np.pi * phase_left / 100.0),
                        np.cos(2.0 * np.pi * phase_left / 100.0),
                        q_right,
                        q_left,
                        qd_right,
                        qd_left,
                        target_right,
                        target_left,
                        target_vel_right,
                        target_vel_left,
                        err_right,
                        err_left,
                        vel_err_right,
                        vel_err_left,
                    ],
                    dtype=np.float64,
                ),
                exo_obs,
                #muscle_activations,
                last_motor_action,
            ],
            dtype=np.float64,
        ).copy()

    def get_rew(self):
        self._update_rwd_dict()
        custom_reward_sum = np.sum([wt * self.rwd_dict[key] for key, wt in self.rwd_keys.items() if key in self.rwd_dict], axis=0)
        return float(custom_reward_sum)

    def get_reward(self):
        return self.get_rew()


    def _update_rwd_dict(self):
        self.rwd_dict = {
            "position_tracking": self._position_tracking_rw(),
            "velocity_tracking": self._velocity_tracking_rw(),
            "torque_cost": self._motor_torque_cost(),
            "torque_smoothness": self._motor_smoothness_cost(),
            "muscle_activation_cost": self._muscle_activation_cost(),
        }
        return self.rwd_dict

    def get_rwd_dict(self):
        if not self.rwd_dict:
            self.rwd_dict = self._update_rwd_dict()
        rwd_dict = {k: v for k, v in self.rwd_dict.items()}
        return rwd_dict
    
    def _reference_velocity_at_phase(self, phase_pct):
        delta_phase = 0.05
        q_next = utils_mtf._reference_at_phase(self.reference_traj, phase_pct + delta_phase)
        q_prev = utils_mtf._reference_at_phase(self.reference_traj, phase_pct - delta_phase)
        dq_dphase = (q_next - q_prev) / (2.0 * delta_phase)
        return float(dq_dphase * 100.0 / self.reference_traj.period_s)

    def _position_tracking_rw(self):
        err_sq = float(self.err_right**2 + self.err_left**2)
        return float(np.exp(-8.0 * err_sq))

    def _velocity_tracking_rw(self):
        vel_err_sq = float(self.vel_err_right**2 + self.vel_err_left**2)
        return float(np.exp(-0.5 * vel_err_sq))

    def _motor_torque_cost(self):
        action = getattr(self, "last_motor_action", np.zeros(self.exo_dims, dtype=np.float64))
        return float(-np.mean(np.square(action))) if action.size else 0.0

    def _motor_smoothness_cost(self):
        action = getattr(self, "last_motor_action", np.zeros(self.exo_dims, dtype=np.float64))
        prev_action = getattr(self, "prev_motor_action", np.zeros_like(action))
        return float(-np.mean(np.square(action - prev_action))) if action.size else 0.0
    
    def _muscle_activation_cost(self):
        activations = self.model.muscle_activation_array()
        normalized = activations / 0.3
        #return float(-np.mean(np.square(activations)))
        return float(-np.mean(np.abs(normalized)))

    def _number_muscle_cost(self):
        """
        Get number of muscle with activations over 0.15 and apply a penalty is com_height is above 0.7
        """
        com_height = self.model.com_pos().y
        if com_height > 0.7:
            return self._get_active_muscles(0.15)
        else:
            return 0.0

    def _get_active_muscles(self, threshold):
        """
        Get the number of muscles whose activations is above the threshold.
        """
        return float(
            np.sum(
                np.where(self.model.muscle_activation_array() > threshold)[0].shape[0]
            )
            / self.action_space.shape[0]
        )
    
    def is_solved(self) -> bool:
        return False    
    
    def is_terminal_condition(self) -> bool:
        com_height = float(self.model.com_pos().y)
        head_height = float(self.head_body.com_pos().y)
        return bool(com_height < self.min_com_height or head_height < self.min_head_height)


from sconegym.nair_motor_disorders import SpasticityEnv

# To do : integrate SpasticityEnv with FlexoGym and Exo Control
class FlexoGymSpasticity(SpasticityEnv, MimoGym):
    """
    Base class for Spasticity Environments
    - Replicate Hyperreflexia by increasing the gain of selected muscles
    """
    def __init__(self, *args, **kwargs):
        self.passive_subject = kwargs.get('passive_subject', True)
        super().__init__(*args, **kwargs)
    
