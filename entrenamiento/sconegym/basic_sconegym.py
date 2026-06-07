from sconegym.sconetools import sconepy
import gymnasium as gym
import os
import numpy as np
from typing import Optional


class BasicSconeGymEnv(gym.Env):
    def __init__(self, 
                model_file, 
                set_rl_spaces=True, 
                max_episode_steps=1000,
                step_size=0.025,
                use_delayed_sensors=False,
                use_delayed_actuators=False,
                *args, **kwargs):
        #print(kwargs)
        self.model_file = model_file
        self.max_episode_steps = max_episode_steps
        self.step_size = step_size
        self.use_delayed_sensors = use_delayed_sensors
        self.use_delayed_actuators = use_delayed_actuators
        self.clip_actions = kwargs.get('clip_actions', True)
        
        #Store parameters 
        self.has_reset = False
        self.store_next = False
        self.episode = 0
        self.total_reward = 0.0
        super().__init__()
        sconepy.set_log_level(3)
        #print(f'hyfydy support: {sconepy.is_supported("ModelHyfydy")}')              
        
        # load model
        if hasattr(self, 'par_file'):
            self.model = sconepy.load_model(model_file, par_file=self.par_file)
        else:
            self.model = sconepy.load_model(self.model_file)
            
        # initial positions and velocities
        self.init_dof_pos = self.model.dof_position_array().copy()
        self.init_dof_vel = self.model.dof_velocity_array().copy()
        self.init_dof_pos_ref = self.model.dof_position_array().copy()
        self.init_dof_vel_ref = self.model.dof_velocity_array().copy()
        self._find_head_body()

        # default simulation parameters
        self.step_size = self.step_size
        #if not hasattr(self, 'step_size'): self.step_size = self.model.control_step_size()
        if not hasattr(self, 'store_next'): self.store_next = False
        if not hasattr(self, 'max_episode_steps'): self.max_episode_steps = 1000
        self.results_dir = sconepy.scone_results_dir()
        
            
        self.set_output_dir("DTIME." + self.model.name())   
        # create useful variables and name lists: actuators, joints
        self.state_names_initialization()
        
        # gym requires obs and action space
        if set_rl_spaces:
            # Rl spaces
            obs = self.get_obs()
            #print(f'[Info] Setting RL spaces for env obs length: {len(obs)}')
            self.action_space = self._set_action_space()
            self.observation_space = self._set_observation_space(obs)      
        #print(f'[Info] Environment initialized with model: {model_file}')

    # ================
    # synergy functions
    # ================
    def load_synergies(self):
        pass
    # ================
    # general functions
    # ================
    def state_names_initialization(self):
        self.actuators = self.model.actuators()
        self.bodies = self.model.bodies()
        self.dofs = self.model.dofs()
        self.muscles = self.model.muscles()
        self.joints = self.model.joints()

        self.njoints = len(self.joints)
        self.ndofs = len(self.dofs)
        #self.joint_names = [joint.name() for joint in self.joints]
        self.joint_names = [joint.name() for joint in self.dofs]

    def time_data(self):
        return self.time + self.step_size
    
    def get_body(self, name):
        for body in self.bodies:
            if body.name() == name:
                return body
    
        #print(f'[Info] Failed to find body: {name}')
  
    def _find_head_body(self):
        head_names = ["torso", "head", "lumbar"]
        self.head_body = None
        for b in self.model.bodies():
            if b.name() in head_names:
                self.head_body = b
        if self.head_body is None:
            raise Exception("Could not find head body")
        
    def adjust_path(self, path):
        # Cut path at "nair_envs" and prepend the absolute path to that folder
        if "nair_envs" in path:
            path_parts = path.split("nair_envs")
            base_path = "/home/carlos/Escritorio/NAIR_RL/sconegym/sconegym"
            path = os.path.join(base_path, "nair_envs" + path_parts[1])
            #print(f'[Info] Adjusted model path to: {path}')
        return path
    # ================
    # scone functions
    # ================
    def reset_model(self, init_qpos, init_qvel):
        
        self.model.reset()
        self.model.set_store_data(self.store_next)
        # ref: https://scone.software/doku.php?id=doc:sconepy#model
        # 1. set muscle activations
        # 2. set joint kinematics
        # 3. equilibrate muscles to match the joint kinematics 
        self.model.set_dof_positions(init_qpos)         # step 1
        self.model.set_dof_velocities(init_qvel )        # step 2
        self.model.init_state_from_dofs()                # step 3
        


    def reset_simulation(self, *args, **kwargs):
        self.reset_model(self.init_dof_pos, self.init_dof_vel)
        self.episode_steps = 0
        self.time = 0


    def step_model(self, action):
    
        # scone reuses the 'action' for 'step_size' steps
        # ref: https://scone.software/doku.php?id=doc:sconepy#model
        if self.use_delayed_actuators:
            self.model.set_delayed_actuator_inputs(action)
        else:
            #print("Model actuator inputs before:", self.model.actuator_input_array())
            self.model.set_actuator_inputs(action)
            #print("Model actuator inputs after:", self.model.actuator_input_array())
        #print("Action step_model:", action)
        self.model.advance_simulation_to(self.time + self.step_size)
        

    # ================
    # muscle functions
    # ================
    def muscle_length(self):
        return self.model.muscle_fiber_length_array()
    def muscle_velocity(self):
        return self.model.muscle_fiber_velocity_array()
    def muscle_activation(self):
        return self.model.muscle_activation_array()
    def muscle_force(self):
        return self.model.muscle_force_array()
    def muscle_command(self):
        return self.model.muscle_excitation_array()
          
    # ================================
    # RL functions: general structure
    # ================================
    def reset_start(self, *args, **kwargs):
        return 0
    def reset_end(self, *args, **kwargs):
        return 0
    def reset(self,
              seed: Optional[int] = None,
              options = None, 

              *args, **kwargs
              ):
        
        self.reset_start(*args, **kwargs)
        self.reset_simulation(*args, **kwargs)
        self.reset_end(*args, **kwargs)
        self.has_reset = True
        self.total_reward = 0.0
        return self.get_obs(), {}

    def seed(self, seed=None):
        """Set the seed for the environment's random number generator."""
        np.random.seed(seed)
        return [seed]

    def before_step(self, action):
        if not self.has_reset:
            raise Exception("You have to call reset() once before step()")
        if self.clip_actions:
            action = np.clip(action, 0, 0.5)
        else:
            action = np.clip(action, 0, 1.0)
    
        return action  
    
    def step(self, action):
        action = self.before_step(action)
        #print("Step called with action2:", action)
        self.step_model(action)
        self.episode_steps += 1
        reward  = self.get_rew()
        obs = self.get_obs()
        terminated = self.is_terminal_condition()   # task specific limit condition
        truncated = self.is_truncation_condition() # time limit
        self.time += self.step_size 
        self.total_reward += reward

        info = {}
        """ DEPRL compatibility: call after_step to check for storing and episode incrementing
        if terminated or truncated:
            self.write_now()
            self.episode += 1
        """
        return obs, reward, terminated, truncated, info
    
    def after_step(self, *args, **kwargs):

        if self.is_terminal_condition() or self.is_truncation_condition():
            self.write_now()
            self.episode += 1

    def get_rew(self):
        return 0
    def is_terminal_condition(self):
        return False
    def is_truncation_condition(self):
        return self.episode_steps>=self.max_episode_steps
  
    def get_obs(self):
        return np.array([], dtype=np.float64)
    
    def _set_action_space(self):
        
        action_space = gym.spaces.Box(
            low=0, 
            high=1, 
            shape=(len(self.actuators),), 
            dtype=np.float64
        )
        #print(f"[INFO] Action space: {action_space}")
          
        return action_space
    
    def _set_observation_space(self, obs):
        low = np.full(obs.shape, -10000, dtype=np.float64)
        high = np.full(obs.shape, 10000, dtype=np.float64)
        return gym.spaces.Box(low, high, dtype=np.float64)    

    def render(self, *args, **kwargs):
        # gym requires this function
        pass

    # ================================
    # DEPRL functions: logger
    # ================================
    def write_now(self):
        if self.store_next:
            self.model.write_results(
                self.output_dir, f"{self.episode:05d}_{self.total_reward:.3f}"
            )
            
            self.store_next = False

    def store_next_episode(self):
        """
        Primes the environment to store the next episode.
        This also calls reset() to ensure that the data is
        written correctly.
        """
        self.store_next = True
        self.reset()
        
    def set_output_dir(self, dir_name):
        self.output_dir = sconepy.replace_string_tags(dir_name)
        
