from sconegym.sconetools import sconepy
import gymnasium as gym
import os
import numpy as np
from typing import Optional
from sconegym.basic_sconegym import BasicSconeGymEnv

class NairSconeGymEnv(BasicSconeGymEnv):
    def __init__(self,
                *args, **kwargs):
        
        self.max_episode_steps = kwargs.get('max_episode_steps', 1000)
        self.obs_type = kwargs.get('obs_type', '3D_Gait_Scone')
        self.exo = kwargs.get('exo', False)
        
        # Action space parameters
        self.passive_subject = kwargs.get('passive_subject', False)
        self.exo_assist = kwargs.get('exo_assist', False) 
        self.exo_type = kwargs.get('exo_type', 'motor')
        self.root_body_name = kwargs.get('root_body_name', 'pelvis')
        self.left_foot_body_name = kwargs.get('left_foot_body_name', 'calcn_l')
        self.right_foot_body_name = kwargs.get('right_foot_body_name', 'calcn_r')

        super().__init__(*args, **kwargs)
        

        

    # ================
    # general functions
    # ================
    def state_names_initialization(self):
        super().state_names_initialization()
        self.num_muscles = len(self.model.muscles())
        self.num_actuators = len(self.model.actuators())
        self.exo_dims = self.num_actuators - self.num_muscles
        self.dofs_names = [m.name() for m in self.model.dofs()]
        self.ms_dofs = len([name for name in self.dofs_names if "exo" not in name]) 
        self.exo_dofs = len(self.dofs_names) - self.ms_dofs
        muscle = self.model.muscles()[0]
        """
        To resolve when Thomas adds more detail about muscles.dofs and muscles.joints problems in the API,
        for now we will just use the dof names to find which muscles are hip, knee or ankle related.
        
        self.hip_muscles = [m for m in self.model.muscles() if "hip" in m.dofs()]
        self.hip_muscles_idx = [i for i, m in enumerate(self.model.muscles()) if "hip" in m.dofs()]
        self.knee_muscles = [m for m in self.model.muscles() if "knee" in m.dofs()]
        self.knee_muscles_idx = [i for i, m in enumerate(self.model.muscles()) if "knee" in m.dofs()]
        self.ankle_muscles = [m for m in self.model.muscles() if "ankle" in m.dofs()]
        self.ankle_muscles_idx = [i for i, m in enumerate(self.model.muscles()) if "ankle" in m.dofs()]
        print(f"[INFO] Total muscles: {self.num_muscles}, Exo muscles: {self.exo_dims}, Muscle-skeleton DOFs: {self.ms_dofs}, Exo DOFs: {self.exo_dofs}")
        print(f"[INFO] Hip muscles: {[m.name() for m in self.hip_muscles]} with indices {self.hip_muscles_idx}, Knee muscles: {[m.name() for m in self.knee_muscles]} with indices {self.knee_muscles_idx}, Ankle muscles: {[m.name() for m in self.ankle_muscles]} with indices {self.ankle_muscles_idx}")
        """
    @property
    def muscle_states(self):
        """
        Computes the DEP input. We assume an input
        muscle_length + force_scale * muscle_force
        where force_scale is chosen by the user and the other
        variables are normalized by the encountered max and min
        values.
        """
        lce = self.muscle_length()
        f = self.muscle_force()
        if not hasattr(self, "max_muscle"):
            self.max_muscle = np.zeros_like(lce)
            self.min_muscle = np.ones_like(lce) * 100.0
            self.max_force = -np.ones_like(f) * 100.0
            self.min_force = np.ones_like(f) * 100.0
        if not np.any(np.isnan(lce)):
            self.max_muscle = np.maximum(lce, self.max_muscle)
            self.min_muscle = np.minimum(lce, self.min_muscle)
        if not np.any(np.isnan(f)):
            self.max_force = np.maximum(f, self.max_force)
            self.min_force = np.minimum(f, self.min_force)
        return (
            1.0
            * (
                (
                    (lce - self.min_muscle)
                    / (self.max_muscle - self.min_muscle + 0.1)
                )
                - 0.5
            )
            * 2.0
            + self.force_scale
            * ((f - self.min_force) / (self.max_force - self.min_force + 0.1))
        ).copy()
    
    @property
    def force_scale(self):
        if not hasattr(self, "_force_scale"):
            self._force_scale = 0
        return self._force_scale
    
    @property
    def results_dir(self):
        """Directory where results should be saved."""
        if not hasattr(self, "_results_dir"):
            self._results_dir = None
        return self._results_dir
    
    @results_dir.setter
    def results_dir(self, value):
        """Set the results directory."""
        self._results_dir = value
    
    # ================================
    # RL functions: general structure
    # ================================
    def before_step(self, action):
        action = super().before_step(action)
        
        complete_action = np.zeros(self.num_actuators)
        #print(f"[DEBUG] Original action: {action}, complete action before filling: {complete_action}")
        # Fill the action from the beginning
        complete_action[:len(action)] = action[:]
        return complete_action  
    
    def get_obs_struct(self):
        # Get information about observation space structure between muscles and exoskeleton
        self.get_obs()
        return self.obs_type, len(self.ms_obs), len(self._get_obs_exo())
    
    def get_obs(self):
        if self.obs_type == 'DOFS': #Get angular and velocity positions of dofs (Imitation learning)
            self.ms_obs = self._get_obs_dofs()
        elif self.obs_type == 'MS': #Get muscle data for synergies
            self.ms_obs = self._get_obs_ms()
        elif self.obs_type == 'Gait_Scone': #Get gait 2D scone observations
            self.ms_obs = self._get_obs_gait_scone()
        elif self.obs_type == '2D_Gait_Scone': #Get gait 2D DEPRL scone observations
            self.ms_obs = self._get_obs_2d_gait_scone()
        elif self.obs_type == '2D_Gait_Scone_MS': #Get gait 2D DEPRL scone observations with only muscle-skeleton info
            self.ms_obs = self._get_obs_2d_gait_scone_ms()
        elif self.obs_type == '2D_Gait_Scone_Exo_Rope': #Get gait 2D DEPRL scone observations with only exoskeleton info for rope model
            self.ms_obs = self._get_obs_2d_gait_scone_exo_rope()
        elif self.obs_type == '3D_Gait_Scone': #Get gait 3D DEPRL scone observations
            self.ms_obs = self._get_obs_3d_gait_scone()
        if self.exo == True:
            return np.concatenate([self.ms_obs, self._get_obs_exo()])
        elif self.exo == False:
            return self.ms_obs
        else:
            print(f'[Error] Observation type {self.obs_type} not implemented')
            raise NotImplementedError
    
    def get_obs_exo(self):
        # self.exo_dofs = number of exo dofs
        # Devuelve los ultimos exo_dofs de los arrays
        pos = self.model.dof_position_array()
        vel = self.model.dof_velocity_array()
        exo_dof_values = [pos[1], pos[3]]  # self.model.dof_position_array()[-self.exo_dofs:]
        exo_dof_vels = [vel[1], vel[3]]  # self.model.dof_velocity_array()[-self.exo_dofs:]
        #print("dof_position_array", self.model.dof_position_array())
        #print("dof_velocity_array", self.model.dof_velocity_array())
        #print("exo_dofs_names1", self.model.dofs()[0].name())
        #print("exo_dofs_names2", self.model.dofs()[1].name())
        #print("exo_dofs_names3", self.model.dofs()[2].name())
        #print("exo_dofs_names4", self.model.dofs()[3].name())
        #print("exo_dof_values", exo_dof_values)
        #print("exo_dof_vels", exo_dof_vels)

        return np.concatenate(
            [
                exo_dof_values,
                exo_dof_vels,
            ],
            dtype=np.float64,
        ).copy()
    
    def _get_obs_ms(self):
        acts = self.model.muscle_activation_array()
        ms_length = self.model.muscle_fiber_length_array()
        ms_forces = self.model.muscle_force_array()  
        dof_values = self.get_ms_dof_values()
        foot_l = self.model.legs()[0].relative_foot_position().array()
        foot_r = self.model.legs()[1].relative_foot_position().array()
        
        return np.concatenate([ acts,
                                ms_length,
                                ms_forces,
                                dof_values,
                                foot_l, 
                                foot_r
                                ],dtype=np.float64,).copy()    
    
    def _get_obs_dofs(self):
        
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        
        return np.concatenate(
            [
                dof_values,
                dof_vels,
            ],
            dtype=np.float64,
        ).copy()
    
    def _get_obs_gait_scone(self):
        acts = self.model.muscle_activation_array()
        #print("muscles_activation:", acts)
        self.prev_acts = self.model.muscle_activation_array().copy()
        self.prev_excs = self.model.muscle_excitation_array().copy()
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        foot_l = self.model.legs()[0].relative_foot_position().array()
        foot_r = self.model.legs()[1].relative_foot_position().array()
        
        return np.concatenate(
            [
                self.model.muscle_fiber_length_array(),
                self.model.muscle_fiber_velocity_array(),
                self.model.muscle_force_array(),
                self.model.muscle_excitation_array(),
                self.head_body.orientation().array(),
                self.head_body.ang_vel().array(),
                foot_l,
                foot_r,
                dof_values,
                dof_vels,
                acts,
            ],
            dtype=np.float64,
        ).copy()
    
    def _get_feet_relative_position(self):
        #print("[DEBUG] Getting feet relative position")
        pelvis = (
            [x for x in self.model.bodies() if self.root_body_name in x.name()][0]
            .com_pos()
            .array()
        )
        #print(f"Pelvis position: {pelvis}")
        foot_l = (
            [x for x in self.model.bodies() if self.left_foot_body_name in x.name()][0]
            .com_pos()
            .array()
        )
       
        #print(f"Left foot position: {foot_l}")
        foot_r = (
            [x for x in self.model.bodies() if self.right_foot_body_name in x.name()][0]
            .com_pos()
            .array()
        )
        
        #print(f"Right foot position: {foot_r}")
        return np.concatenate([foot_l - pelvis, foot_r - pelvis], dtype=np.float32)
    
    def _get_obs_3d_gait_scone(self):
        acts = self.model.muscle_activation_array()
        self.prev_acts = self.model.muscle_activation_array().copy()
        self.prev_excs = self.model.muscle_excitation_array()
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        # No x or y position in the state
        dof_values[3] = 0.0
        dof_values[5] = 0.0
        return np.concatenate(
            [
                self.model.muscle_fiber_length_array(),
                self.model.muscle_fiber_velocity_array(),
                self.model.muscle_force_array(),
                self.model.muscle_excitation_array(),
                self.head_body.orientation().array(),
                self.head_body.ang_vel().array(),
                self._get_feet_relative_position(),
                dof_values,
                dof_vels,
                acts,
            ],
            dtype=np.float64,
        ).copy()
    
    def _get_obs_2d_gait_scone(self):
        acts = self.model.muscle_activation_array()
        self.prev_acts = self.model.muscle_activation_array().copy()
        self.prev_excs = self.model.muscle_excitation_array()
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        dof_values[1] = 0.0
        if not self.use_delayed_sensors:
            #print(f"[Info] NOT Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.muscle_fiber_length_array())}, 2 term: {len(self.model.muscle_fiber_velocity_array())}, 3 term: {len(self.model.muscle_force_array())}, 4 term: {len(self.head_body.orientation().array())}, 5 term: {len(self.head_body.ang_vel().array())}, 6 term: {len(self._get_feet_relative_position())}, 7 term: {len(dof_values)}, 8 term: {len(dof_vels)}, 9 term: {len(acts)}")
            
            return np.concatenate(
                [
                    self.model.muscle_fiber_length_array(),
                    self.model.muscle_fiber_velocity_array(),
                    self.model.muscle_force_array(),
                    self.model.muscle_excitation_array(),
                    self.head_body.orientation().array(),
                    self.head_body.ang_vel().array(),
                    self._get_feet_relative_position(),
                    dof_values,
                    dof_vels,
                    acts,
                ],
                dtype=np.float64,
            ).copy()

        else:
            #print(f"[Info] Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.delayed_muscle_fiber_length_array())}, 2 term: {len(self.model.delayed_muscle_fiber_velocity_array())}, 3 term: {len(self.model.delayed_muscle_force_array())}, 4 term: {len(self.model.delayed_vestibular_array())}, 5 term: {len(self.model.muscle_excitation_array())}, 6 term: {len(self.model.muscle_activation_array())}")
            return np.concatenate(
                [
                    self.model.delayed_muscle_fiber_length_array(),
                    self.model.delayed_muscle_fiber_velocity_array(),
                    self.model.delayed_muscle_force_array(),
                    self.model.delayed_vestibular_array(),
                    self.model.muscle_excitation_array(),
                    self.model.muscle_activation_array(),
                ],
                dtype=np.float64,
            ).copy()
    
    def _get_obs_2d_gait_scone_ms(self):

        acts = self.model.muscle_activation_array()[0:self.num_muscles]
        self.prev_acts = self.model.muscle_activation_array().copy()[0:self.num_muscles]
        self.prev_excs = self.model.muscle_excitation_array()[0:self.num_muscles]
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        dof_values[1] = 0.0
        if not self.use_delayed_sensors:
            #print(f"[Info] NOT Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.muscle_fiber_length_array())}, 2 term: {len(self.model.muscle_fiber_velocity_array())}, 3 term: {len(self.model.muscle_force_array())}, 4 term: {len(self.head_body.orientation().array())}, 5 term: {len(self.head_body.ang_vel().array())}, 6 term: {len(self._get_feet_relative_position())}, 7 term: {len(dof_values)}, 8 term: {len(dof_vels)}, 9 term: {len(acts)}")
            
            return np.concatenate(
                [
                    self.model.muscle_fiber_length_array()[0:self.num_muscles],
                    self.model.muscle_fiber_velocity_array()[0:self.num_muscles],
                    self.model.muscle_force_array()[0:self.num_muscles],
                    self.model.muscle_excitation_array()[0:self.num_muscles],
                    self.head_body.orientation().array(),
                    self.head_body.ang_vel().array(),
                    self._get_feet_relative_position(),
                    dof_values,
                    dof_vels,
                    acts[0:self.num_muscles],
                ],
                dtype=np.float64,
            ).copy()

        else:
            #print(f"[Info] Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.delayed_muscle_fiber_length_array())}, 2 term: {len(self.model.delayed_muscle_fiber_velocity_array())}, 3 term: {len(self.model.delayed_muscle_force_array())}, 4 term: {len(self.model.delayed_vestibular_array())}, 5 term: {len(self.model.muscle_excitation_array())}, 6 term: {len(self.model.muscle_activation_array())}")
            return np.concatenate(
                [
                    self.model.delayed_muscle_fiber_length_array()[0:self.num_muscles],
                    self.model.delayed_muscle_fiber_velocity_array()[0:self.num_muscles],
                    self.model.delayed_muscle_force_array()[0:self.num_muscles],
                    self.model.delayed_vestibular_array(),
                    self.model.muscle_excitation_array()[0:self.num_muscles],
                    self.model.muscle_activation_array()[0:self.num_muscles],
                ],
                dtype=np.float64,
            ).copy()
    
    def _get_obs_2d_gait_scone_exo_rope(self):

        acts = self.model.muscle_activation_array()[self.num_muscles:-1] # Only exo activations
        self.prev_acts = self.model.muscle_activation_array().copy()[self.num_muscles:-1]
        self.prev_excs = self.model.muscle_excitation_array()[self.num_muscles:-1]
        dof_values = self.get_ms_dof_values()
        dof_vels = self.get_ms_dof_vels()
        dof_values[1] = 0.0
        if not self.use_delayed_sensors:
            #print(f"[Info] NOT Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.muscle_fiber_length_array())}, 2 term: {len(self.model.muscle_fiber_velocity_array())}, 3 term: {len(self.model.muscle_force_array())}, 4 term: {len(self.head_body.orientation().array())}, 5 term: {len(self.head_body.ang_vel().array())}, 6 term: {len(self._get_feet_relative_position())}, 7 term: {len(dof_values)}, 8 term: {len(dof_vels)}, 9 term: {len(acts)}")
            
            return np.concatenate(
                [
                    self.model.muscle_fiber_length_array()[self.num_muscles:-1],
                    self.model.muscle_fiber_velocity_array()[self.num_muscles:-1],
                    self.model.muscle_force_array()[self.num_muscles:-1],
                    self.model.muscle_excitation_array()[self.num_muscles:-1],
                    self.head_body.orientation().array(),
                    self.head_body.ang_vel().array(),
                    self._get_feet_relative_position(),
                    dof_values,
                    dof_vels,
                    acts[self.num_muscles:-1],
                ],
                dtype=np.float64,
            ).copy()

        else:
            #print(f"[Info] Using delayed sensors for observation")
            #print(f"[Info] parcial length: 1 term: {len(self.model.delayed_muscle_fiber_length_array())}, 2 term: {len(self.model.delayed_muscle_fiber_velocity_array())}, 3 term: {len(self.model.delayed_muscle_force_array())}, 4 term: {len(self.model.delayed_vestibular_array())}, 5 term: {len(self.model.muscle_excitation_array())}, 6 term: {len(self.model.muscle_activation_array())}")
            return np.concatenate(
                [
                    self.model.delayed_muscle_fiber_length_array()[self.num_muscles:-1],
                    self.model.delayed_muscle_fiber_velocity_array()[self.num_muscles:-1],
                    self.model.delayed_muscle_force_array()[self.num_muscles:-1],
                    self.model.delayed_vestibular_array(),
                    self.model.muscle_excitation_array()[self.num_muscles:-1],
                    self.model.muscle_activation_array()[self.num_muscles:-1],
                ],
                dtype=np.float64,
            ).copy()
        
    def _set_action_space(self):
        motor_action_space = [-1,1]
        if self.exo_type == 'rope':
            # Find "exo" inside muscles names and readjust motor action space
            muscle_names = [m.name() for m in self.model.muscles()]
            self.num_muscles = len([name for name in muscle_names if "exo" not in name])
            self.exo_dims = self.num_actuators - self.num_muscles
            motor_action_space = [0,1]
            
        if self.passive_subject:
        # Passive subject: only exoskeleton, no muscle control
            action_space = gym.spaces.Box(
                low=motor_action_space[0], 
                high=motor_action_space[1], 
                shape=(self.exo_dims,), 
                dtype=np.float64
            )
            #print(f"[INFO] Passive subject action space: {action_space}")
            
        elif not self.exo_assist:
        # Active subject without assist: only muscles control, no exoskeleton
            action_space = gym.spaces.Box(
                low=0,
                high=1,
                shape=(self.num_muscles,), 
                dtype=np.float64
            )
            #print(f"[INFO] Active subject action space: {action_space}")
        
        elif self.exo_assist:
            # Active subject with exoskeleton assist: muscles + exoskeleton motors
            muscles_action_space = gym.spaces.Box(
                low=0, 
                high=1, 
                shape=(self.num_muscles,), 
                dtype=np.float64
            )
            exo_action_space = gym.spaces.Box(
                low=motor_action_space[0], 
                high=motor_action_space[1], 
                shape=(self.exo_dims,), 
                dtype=np.float64
            )
            action_space = gym.spaces.Box(
                low=np.concatenate([muscles_action_space.low, exo_action_space.low]),
                high=np.concatenate([muscles_action_space.high, exo_action_space.high]),
                dtype=np.float64
            )
            #print(f"[INFO] Active subject with exo assist type [{self.exo_type}], Total action space: {action_space}, Muscles: {self.num_muscles}, Exo: {self.exo_dims}")
          
        return action_space
    
    def get_ms_dof_values(self):
        #print("[DEBUG] Getting muscle-skeleton DOF values")
        #print(f"[DEBUG] Total DOFs: {len(self.model.dof_position_array())}, Exo DOFs: {self.exo_dofs}, Muscle-Skeleton DOFs: {len(self.model.dof_position_array())-self.exo_dofs}")
        #Take the DOF values without the exoskeleton DOFs
        if self.exo_dofs == 0:
            return self.model.dof_position_array()
        else:
            return self.model.dof_position_array()[:-self.exo_dofs]
        
    def get_ms_dof_vels(self):
        if self.exo_dofs == 0:
            return self.model.dof_velocity_array()
        else:
            return self.model.dof_velocity_array()[:-self.exo_dofs]
    
from sklearn.decomposition import PCA, FastICA, NMF
from sklearn.preprocessing import MinMaxScaler    
import joblib

class MuscleSyn():
    def __init__(self, *args, **kwargs):   
        self.passive_subject = False  
        super().__init__(*args, **kwargs)       
        
    def load_muscle_synergies(self):
        # load muscle synergies
        model_name = self.model.name()
        model_name = model_name.split('_')[0]
        #print(f'[Info] loading muscle synergy parameters for model: {model_name}')
        postfix = "syn{}".format(self.num_synergies) # load different synergy sets
        self.pca = joblib.load(f"config/DEP/synergies/{model_name}/PCAICA_pca_{postfix}.pkl")
        self.ica = joblib.load(f"config/DEP/synergies/{model_name}/PCAICA_ica_{postfix}.pkl")
        self.normalizer = joblib.load(f"config/DEP/synergies/{model_name}/PCAICA_normalizer_{postfix}.pkl")
        
    def _set_action_space(self):
        self.load_muscle_synergies()
        self.sar_n_actions = self.pca.components_.shape[0]
        synergies_action_space = gym.spaces.Box(low=0, high=1, shape=(self.sar_n_actions,),dtype=np.float64)
        if self.exo_assist:
            # Get parent action space to extract exo dimensions
            parent_action_space = super()._set_action_space()
            
            # Combine synergies + exoskeleton
            action_space = gym.spaces.Box(
                low=np.concatenate([synergies_action_space.low, parent_action_space.low[-self.exo_dims:]]),
                high=np.concatenate([synergies_action_space.high, parent_action_space.high[-self.exo_dims:]]),
                dtype=np.float64
            )
            #print(f'[Info] Combined action space: {self.sar_n_actions} synergies + {self.exo_dims} exo = {action_space.shape[0]} total')
        else:
            # Only synergies, no exoskeleton
            action_space = synergies_action_space
            #print(f'[Info] Synergies-only action space: {self.sar_n_actions} actions')
        #print(f'[Info] Action space set to: {action_space}')
        return action_space
    
    def synergy_to_muscle_commands(self, ctrl_cmd):
        """@info ctrl_cmd considers the primitive control command"""
        # RL commands betwee 0 and 1
        ctrl_cmd = ctrl_cmd.squeeze()
        ctrl_cmd[ctrl_cmd<0] = 0.
        ctrl_cmd[ctrl_cmd>1] = 1.

        # either synergetic or synergy+non-synergetic action representation
        try:
        #Synergectic + non-synergectic action representation (SAR+muscles with phi 0<phi<1)
            syn_ctrl_cmd = ctrl_cmd[:self.pca.components_.shape[0]].copy()
            nonsyn_ctrl_cmd = ctrl_cmd[self.pca.components_.shape[0]:].copy()
        except:
            syn_ctrl_cmd = ctrl_cmd.copy()
            nonsyn_ctrl_cmd = None
        
        # synergectic action representation (SAR publication)
        syn_ctrl_cmd = self.pca.inverse_transform(self.ica.inverse_transform(self.normalizer.inverse_transform([syn_ctrl_cmd])))[0]

        # additional clipping for synergies
        syn_ctrl_cmd[syn_ctrl_cmd<0] = 0.
        syn_ctrl_cmd[syn_ctrl_cmd>1] = 1.
    
        try:
            ctrl_cmd = self.muscle_syn['phi']*syn_ctrl_cmd + (1-self.muscle_syn['phi'])*nonsyn_ctrl_cmd
        except:
            ctrl_cmd = syn_ctrl_cmd.copy()

        return ctrl_cmd
  
    def before_step(self, action):
        if self.passive_subject:
            # Passive: action = [exo_commands] -> [zero_muscles, exo_commands]
            full_action = np.concatenate([np.zeros(self.num_muscles), action*100])
            
        else:
            # Active: action = [synergies, exo_commands] -> [exo_commands, muscle_commands]
            synergy_action = action[:self.sar_n_actions]
            exo_action = action[self.sar_n_actions:]*100
            muscle_action = self.synergy_to_muscle_commands(synergy_action)
                
            if self.exo_dims > 0:
                full_action = np.concatenate([muscle_action, exo_action])
            else:
                full_action = muscle_action
        return super().before_step(full_action)  
