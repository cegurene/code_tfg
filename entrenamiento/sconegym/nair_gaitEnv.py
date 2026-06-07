# NAIR_standupEnv.py

import numpy as np
import yaml
import os, sys
import gymnasium as gym
from sconegym.nair_sconegym import NairSconeGymEnv
from sconegym.sconetools import sconepy
from typing import Optional
import sconepy # type: ignore
import collections

DEFAULT_RW_DICT = {
    "gaussian_vel": 10.0,
    "grf": -0.07281,
    "smooth": -0.097,
    "number_muscles": -1.57929,
    "constr": -0.1307,
    "self_contact_coeff": 0.0,

   # "solved": 0.0,
    #"terminated": 0.0,  # Not working at the beginning
}

class GaitGym(NairSconeGymEnv):
    def __init__(self,  
                 left_leg_idxs,
                 right_leg_idxs,
                 target_vel = 1.2,
                 leg_switch = True,
                 run = False,
                 init_activations_mean = 0.3,
                 init_activations_std = 0.1,
                 min_com_height = 0.5,
                 min_head_height = 0.9, #0.9,
                 fall_recovery_time = 0.0,
                 rwd_keys = DEFAULT_RW_DICT,
                 *args, **kwargs):
        # Internal settings
        self.init_pos_rd = kwargs.get('init_pos_rd', True)
        self.init_dof_pos_std = 0.05
        self.init_dof_vel_std = 0.1
        self.init_load = 0.5
        self.init_activations_mean = init_activations_mean
        self.init_activations_std = init_activations_std
        self.min_com_height = min_com_height
        self.min_head_height = min_head_height
        self.fall_time = -1.0
        self.target_vel = target_vel
        self.leg_switch = leg_switch
        self.run = run
        self.obs_type = kwargs.get('obs_type', '3D_Gait_Scone')
        self.left_leg_idxs = left_leg_idxs
        self.right_leg_idxs = right_leg_idxs
        self.fall_recovery_time = fall_recovery_time
        # Sconegym normal settings 
        self.rwd_keys = rwd_keys
        # Reward coefficients from kwargs
        for k, v in self.rwd_keys.items():
            setattr(self, k, float(v))
        super().__init__(*args, **kwargs)
        self.mass = np.sum([x.mass() for x in self.model.bodies()])
        self._dof_names = [d.name() for d in self.model.dofs()]
        self.pelvis_pos_rw_prev = 0.0
        self.pelvis_distance_prev = 0.9 - self.model.com_pos().y
        self.solved_time = None      # Clear saved time
        self.solved_flag = True     # Allow new detection
        self.rwd_debug = 0
        

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = True,
        options: Optional[dict] = None,
    ):
        """
        Reset and randomize the initial state.
        """
        self.model.reset()
        self.has_reset = True
        self.total_reward = 0.0

        self.model.set_store_data(self.store_next)
        if self.init_pos_rd:
            # Positions
            self.init_dof_pos = self.init_dof_pos_ref + np.random.normal(
                0, self.init_dof_pos_std, len(self.init_dof_pos)
            )
            # Velocities
            self.init_dof_vel = self.init_dof_vel_ref + np.random.normal(
                0, self.init_dof_vel_std, len(self.init_dof_vel)
            )
        else:
            self.init_dof_pos = self.init_dof_pos_ref.copy()
            self.init_dof_vel = self.init_dof_vel_ref.copy()

        self.model.set_dof_positions(self.init_dof_pos)
        self.model.set_dof_velocities(self.init_dof_vel)
        # Switch legs randomly
        if self.leg_switch:
            if np.random.uniform() < 0.5:
                self._switch_legs()
                
        # Muscles
        if self.init_activations_std!= 0.0:
            muscle_activations =self._randomize_muscle_activations()
        else:
            # Set non-random initial muscle activations
            muscle_activations = np.ones((len(self.model.muscles()),)) * self.init_activations_mean
        
        self.prev_acts = muscle_activations
        self.prev_excs = self.model.muscle_excitation_array()
        self.model.init_muscle_activations(muscle_activations)

        # Initialize state and equilibrate muscles
        self.model.init_state_from_dofs()

        if self.init_load > 0:
            self.model.adjust_state_for_load(self.init_load)
        
        obs = self.get_obs()

        self.episode_steps = 0
        self.time = 0
        self.fall_time = -1.0       # Reset fall time
        self.solved_time = None      # Clear saved time
        self.solved_flag = True     # Allow new detection
        self.pelvis_distance_prev = 0.9 - self.model.com_pos().y
        info = {
            'obs_type': self.obs_type,
            'muscles_names': [m.name() for m in self.model.muscles()],
            'dofs_names': [d.name() for d in self.model.dofs()],
            'foot_position': ['foot_l', 'foot_r'],
            #'solved': self.is_solved(),
            'rwd_dict': self.rwd_keys,
        }
        return obs, info

    def step(self, action):

        obs, reward, terminated, truncated, info = super().step(action)
        
        info = {
            'obs_type': self.obs_type,
            'muscles_names': [m.name() for m in self.model.muscles()],
            'dofs_names': [d.name() for d in self.model.dofs()],
            'foot_position': ['foot_l', 'foot_r'],
            #'solved': self.is_solved(),
            'rwd_dict': self.rwd_dict,
        }
        
        return obs, reward, terminated, truncated, info
        
    def _switch_legs(self):
        """
        Switches leg joint angles. Good for initial
        state randomization.
        """
        pos = self.model.dof_position_array()
        vel = self.model.dof_velocity_array()
        for left, right in zip(self.left_leg_idxs, self.right_leg_idxs):
            pos[left], pos[right] = pos[right], pos[left]
            vel[left], vel[right] = vel[right], vel[left]
        self.model.set_dof_positions(pos)
        self.model.set_dof_velocities(vel)
        
        

    def _randomize_muscle_activations(self):
        """
        Randomizes muscle activations around a mean value.
        """
        activations = np.clip(
            np.random.normal(
                self.init_activations_mean,
                self.init_activations_std,
                self.num_muscles,
            ),
            0.0,
            1.0,
        )
        #print("Initial muscle activations:", activations)
        return activations
        

    def get_rew(self):
        self._update_rwd_dict()
        print("rwd_dict", self.rwd_dict)
        custom_reward_sum = np.sum([wt * self.rwd_dict[key] for key, wt in self.rwd_keys.items()], axis=0)
        #print("custom_reward_sum", custom_reward_sum, "custom_reward_terms", {key: wt * self.rwd_dict[key] for key, wt in self.rwd_keys.items()})
        print("reward", custom_reward_sum, "terms", {key: wt * self.rwd_dict[key] for key, wt in self.rwd_keys.items()})
        return float(custom_reward_sum)


    def _update_rwd_dict(self):
        self.rwd_dict = {
            # Optional rewards
            "gaussian_vel": self._gaussian_plateau_vel(),
            "grf": self._grf(),
            "smooth": self._exc_smooth_cost(),
            "number_muscles": self._number_muscle_cost(),
            "constr": self._joint_limit_torques(),
            "self_contact": self.self_contact_coeff * self._get_self_contact(),
            # Mandatory rewards
            #"solved": 1.0 if self.is_solved() else 0.0,
            #"terminated": 1.0 if self.is_terminal_condition() else 0.0,
        }
        #print("rwd_dict debug:", self.rwd_debug)
        self.rwd_debug = self.rwd_debug + 1
        return self.rwd_dict

    def get_rwd_dict(self):
        if not self.rwd_dict:
            self.rwd_dict = self._update_rwd_dict()
        rwd_dict = {k: v for k, v in self.rwd_dict.items()}
        return rwd_dict
    
    
    def _gaussian_vel(self):
        vel = self.model.com_vel().x
        print("vel", vel)
        return np.exp(-np.square(vel - self.target_vel))

    def _gaussian_plateau_vel(self):
        if self.run:
            return self.model.com_vel().x
        vel = self.model.com_vel().x
        if vel < self.target_vel:
            return self._gaussian_vel()
        else:
            return 1.0

    def _exc_smooth_cost(self):
        excs = self.model.muscle_excitation_array()
        print("excs", excs)
        delta_excs = excs - self.prev_excs
        return np.mean(np.square(delta_excs))

    def _number_muscle_cost(self):
        """
        Get number of muscle with activations over 0.15 and apply a penalty is com_height is above 0.7
        """
        
        com_height = self.model.com_pos().y
        if com_height > 0.7:
            muscle_cost = num_active_muscles = self._get_active_muscles(0.15)
            return -muscle_cost
        else:
            return 0.0
    
    def _get_active_muscles(self, threshold):
        """
        Get the number of muscles whose activations is above the threshold.
        """
        num_active_muscles = np.sum(self.model.muscle_activation_array() > threshold)
        return num_active_muscles
    
    def _joint_limit_torques(self):
        return np.mean(
            [np.mean(np.abs(x.limit_torque().array())) for x in self.model.joints()]
        )

    def _grf(self):
        grf = self.model.contact_load()
        return max(0, grf - 1.2)
    
    def is_solved(self) -> bool:
        """
        The task is considered solved if the center of mass is above 0.9 for 0.5 consecutive seconds.
        
        Estates:
        - solved_flag = True: Waiting for condition to be met
        - solved_flag = False: Condition met, measuring time
        - solved_time = None: No start time saved
        - solved_time = : Time when it was first met  
        """
        com_height = self.model.com_pos().y
        
        threshold = 0.9
        current_time = self.model.time()
        required_duration = 4  # seconds
        
        # Initialize if necessary
        if not hasattr(self, 'solved_time'):
            self.solved_time = None
        if not hasattr(self, 'solved_flag'):
            self.solved_flag = True  # Allow new detection
        
        if com_height > threshold:
            #print(f"[DEBUG] com_height={com_height:.3f} > threshold={threshold}")
            # CASE 1: First time the condition is met
            if self.solved_flag and self.solved_time is None:
                self.solved_time = current_time
                self.solved_flag = False
                #print(f"[INFO] Height condition met at t={self.solved_time:.3f}s. Starting measurement.")

            # CASE 2: Check if it has been maintained long enough
            elif self.solved_time is not None:
                duration = current_time - self.solved_time
                if duration >= required_duration/2:
                    self.last_solved_episode = True  # For force adjustment
                    
                if duration >= required_duration:
                    #print(f"[SUCCESS] Task solved after {duration:.3f}s above threshold!, at t={current_time:.3f}s.")
                    return True
        else:
            # CASE 3: Condition lost, reset for new opportunity
            #if self.solved_time is not None:  # Only if it was being measured
                #print(f"[INFO] Height condition lost at t={current_time:.3f}s (was measuring for {current_time - self.solved_time:.3f}s)")
            self.solved_flag = True     # Allow new detection
            self.solved_time = None      # Clear saved time

        return False
        
    
    def is_terminal_condition(self) -> bool:
        """
        The episode ends if the center of mass is below min_com_height.
        """
        com_height = self.model.com_pos().y
        fall = com_height < self.min_com_height
        fall = fall or self.head_body.com_pos().y < self.min_head_height
        current_time = self.model.time()
        
        if fall:
            if self.fall_time < 0:
                self.fall_time = current_time
            if current_time - self.fall_time >= self.fall_recovery_time:
                return True
        else:
            self.fall_time = -1.0
    

        return False

class NairGaitGym(GaitGym):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class SconeGaitGym(GaitGym):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)       
        self.rwd_dict = None

    def model_velocity(self):
        return self.model.com_vel().x
    
    def get_rew(self):
        """
        Reward function.
        """
        return self.custom_reward()

    def custom_reward(self):
        self._update_rwd_dict()
        
        return np.sum(list(self.rwd_dict.values()))

    def _update_rwd_dict(self):
        self.rwd_dict = {
            "gaussian_vel": self.gaussian_vel * self._gaussian_plateau_vel(),
            "grf": self.grf * self._grf(),
            "smooth": self.smooth * self._exc_smooth_cost(),
            "number_muscles": self.number_muscles * self._number_muscle_cost(),
            "constr": self.constr * self._joint_limit_torques(),
            "self_contact": 0.0 * self._get_self_contact(),
        }
        return self.rwd_dict

    def get_rwd_dict(self):
        if not self.rwd_dict:
            self.rwd_dict = self._update_rwd_dict()
        rwd_dict = {k: v for k, v in self.rwd_dict.items()}
        return rwd_dict

    def _number_muscle_cost(self):
        """
        Get number of muscle with activations over 0.15.
        """
        return self._get_active_muscles(0.15)

    def _get_active_muscles(self, threshold):
        """
        Get the number of muscles whose activations is above the threshold.
        """
        return (
            np.sum(
                np.where(self.model.muscle_activation_array() > threshold)[0].shape[0]
            )
            / self.action_space.shape[0]
        )

    def _gaussian_vel(self):
        vel = self.model_velocity()
        return np.exp(-np.square(vel - self.target_vel))

    def _gaussian_plateau_vel(self):
        if self.run:
            return self.model_velocity()
        vel = self.model_velocity()
        if vel < self.target_vel:
            return self._gaussian_vel()
        else:
            return 1.0

    def _exc_smooth_cost(self):
        excs = self.model.muscle_excitation_array()
        delta_excs = excs - self.prev_excs
        return np.mean(np.square(delta_excs))

    def _get_self_contact(self):
        ignore_bodies = ["calcn_r", "calcn_l"]
        contact_force = np.sum(
            [
                np.abs(x.contact_force().array())
                for x in self.model.bodies()
                if x.name() not in ignore_bodies
            ]
        )
        return np.clip(contact_force, -100, 100) / 100

    def _joint_limit_torques(self):
        return np.mean(
            [np.mean(np.abs(x.limit_torque().array())) for x in self.model.joints()]
        )

    def _grf(self):
        grf = self.model.contact_load()
        return max(0, grf - 1.2)

    def is_terminal_condition(self) -> bool:
        """
        The episode ends if the center of mass is below min_com_height.
        """
        fall = self.model.com_pos().y < self.min_com_height
        fall = fall or self.head_body.com_pos().y < self.min_head_height
        current_time = self.model.time()
        if fall:
            if self.fall_time < 0:
                self.fall_time = current_time
            if current_time - self.fall_time >= self.fall_recovery_time:
                return True
        else:
            self.fall_time = -1.0

        return False




from sconegym.nair_sconegym import MuscleSyn
class GaitGymSyn(MuscleSyn, GaitGym):
    def __init__(self, num_synergies, *args, **kwargs):    
        self.num_synergies = num_synergies
        super().__init__(*args, **kwargs)  



