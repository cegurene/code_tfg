
#=================================================
#	This model is generated with tacking the Myosuite conversion of [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close
#reference.
#	Model	  :: MyoLeg 1 Dof 40 Musc Exo (MuJoCoV2.0)
#	Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), Jorge Gomez, David Rodriguez, Pablo Lanillos, Carlos Eguren
#	source	:: https://github.com/AndresChS/NAIR_Code
#	====================================================== -->
import collections
from pathlib import Path
from myosuite.utils import gym
import numpy as np
import mujoco
from myosuite.envs.myo.base_v0 import BaseV0
from myosuite.envs import env_base
import torch.nn
import random
import pickle

import socket

ANGLES_DICT_PATH = Path(__file__).resolve().with_name("angles_dict.pkl")
# The sockets are created in lines 43-51 and 444-475

class Myoleg_env_v0(BaseV0,env_base.MujocoEnv):

    MYO_CREDIT = """\
    This model is generated with tacking the Myosuite conversion of [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close reference.
        Model	:: MyoLeg 1 Dof 40 Musc Exo (MuJoCoV2.0)
        Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), David Rodriguez, Pablo Lanillos 
        Source	:: https://github.com/AndresChS/NAIR_Code
    """
    
    DEFAULT_OBS_KEYS = ['time', 'qpos', 'qvel', 'qacc', 'act', 'pose_err', 'inter_force']
    DEFAULT_RWD_KEYS_AND_WEIGHTS = {
        "pose": 10,  
        "inter_force": 0,
        "smooth_act": 0.03,
        "acceleration": 0,
        "penalty_high": 10000.0,
        "penalty_low": 10000.0
    }

    def __init__(self, model_path, obsd_model_path=None, seed=None, **kwargs):

        """
        # We create the server in the port specified by the socket_server.bind().
        # If we encounter an error, we should change the port (which must be the same as the client socket).
        print("Starting server")
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_server.bind(('localhost', 9997))
        print("Server started on port 9997")
        self.socket_server.listen(1)
        """

        # EzPickle.__init__(**locals()) is capturing the input dictionary of the init method of this class.
        # In order to successfully capture all arguments we need to call gym.utils.EzPickle.__init__(**locals())
        # at the leaf level, when we do inheritance like we do here.
        # kwargs is needed at the top level to account for injection of __class__ keyword.
        # Also see: https://github.com/openai/gym/pull/1497
        gym.utils.EzPickle.__init__(self, model_path, obsd_model_path, seed, **kwargs)

        # This two step construction is required for pickling to work correctly. All arguments to all __init__
        # calls must be pickle friendly. Things like sim / sim_obsd are NOT pickle friendly. Therefore we
        # first construct the inheritance chain, which is just __init__ calls all the way down, with env_base
        # creating the sim / sim_obsd instances. Next we run through "setup"  which relies on sim / sim_obsd
        # created in __init__ to complete the setup.
        super().__init__(model_path=model_path, obsd_model_path=obsd_model_path, seed=seed, env_credits=self.MYO_CREDIT)

        self._setup(**kwargs)

        

    def _setup(self,
            obs_keys:list = DEFAULT_OBS_KEYS,
            weighted_reward_keys:dict = DEFAULT_RWD_KEYS_AND_WEIGHTS,
            viz_site_targets:tuple = None,  # site to use for targets visualization []
            target_jnt_range: dict = {'joint_range' :(0, 90)},   # joint ranges as tuples {name:(min, max)}_nq
            target_jnt_value: list = None,   # desired joint vector [des_qpos]_nq
            reset_type = "init",            # none; init; random
            target_type = "fixed",       # generate; switch; fixed
            target_qpos = 0.5, # Desired knee_angle_r position
            spasticity = "passive",
            spasticity_level = 0,
            sigma = 0.5,
            weight_bodyname = None,
            weight_range = None,
            **kwargs,
        ):
        #Memmory variables
        # Initialize as vectors (1 exo + 40 muscles = 41 total actions)
        self.action_prev1 = np.zeros(41)
        self.action_prev2 = np.zeros(41)
        self.excess_force_prev = 0
        self.cumulative_reward_value = 0
        self.qacc_prev = 0
        self.torque_prev = 0
        
        self.reset_type = reset_type
        self.target_type = target_type
        self.sigma = sigma
        self.spasticity = spasticity
        self.spasticity_level = spasticity_level
        self.spasticity_args = self.randomize_spas(level=self.spasticity_level, randomize=False)
        self.weight_bodyname = weight_bodyname
        self.weight_range = weight_range
        self.target_qpos = []
        self.target_qpos.append(target_qpos)
        print('target_qpos:',target_qpos)
        # resolve joint demands

        if target_jnt_range:
            self.target_jnt_ids = []
            self.target_jnt_range = []
            for jnt_name, jnt_range in target_jnt_range.items():
                self.target_jnt_ids.append(self.sim.model.joint_name2id(jnt_name))
                self.target_jnt_range.append(jnt_range)
            self.target_jnt_range = np.array(self.target_jnt_range)
            self.target_jnt_value = np.mean(self.target_jnt_range, axis=1)  # pseudo targets for init
        else:
            self.target_jnt_value = target_jnt_value
        
        """
        # Initialize DR and metrics BEFORE super()._setup() (which calls step())
        self.dr_enabled = True  # Toggle for sim-to-real
        self.dr_ranges = {
            'friction': (0.7, 1.3),           # Multiplicador sobre fricción base
            'damping': (0.8, 1.2),            # Joint damping variability
            'mass_multiplier': (0.95, 1.05),  # Body mass variation
            'input_noise_vel': (0.03, 0.1),   # Noise std as fraction of max velocity
        }

        self.episode_metrics = {
            'action_smoothness': [],  # L2 norm of action rate
            'torque_peaks': [],       # Max |torque| applied
            'saturation_count': 0,    # Count of saturated actions
            'mean_reward': 0,
        }
        """
        
        super()._setup(obs_keys=obs_keys,
                weighted_reward_keys=weighted_reward_keys,
                sites=viz_site_targets,
                **kwargs,
                )

        """
        # After setting up your muscles/actuators
        self.k_damp_target = 0.1  # Nms/rad, alcanzable en mdtool
        self.compute_and_set_muscle_damping()
        """

    def get_obs_dict(self, sim):
        obs_dict = {}
        obs_dict['time'] = np.array([sim.data.time])
        obs_dict['qpos'] = sim.data.qpos
        obs_dict['qvel'] = sim.data.qvel
        obs_dict['qacc'] = sim.data.qacc
        obs_dict['act'] = np.asarray([sim.data.ctrl[0]])#if sim.model.na>0 else np.zeros_like(obs_dict['qpos'])
        obs_dict['qfrc_actuator'] = sim.data.qfrc_actuator
        obs_dict['qfrc_constraint'] = sim.data.qfrc_constraint
        obs_dict['pose_err'] = np.absolute(self.target_qpos - sim.data.qpos[6])
        obs_dict['inter_force'] = np.asarray([self.get_inter_force(pos=sim.data.qpos[6], torque=sim.data.qfrc_constraint[6])])
        return obs_dict

    def squeeze_obs_dict(self, obs_dict):
        obs_dict_squeeze = {}
        obs_dict_squeeze['time'] = np.squeeze(obs_dict['time'])
        obs_dict_squeeze['qpos'] = np.squeeze(obs_dict['qpos'])
        obs_dict_squeeze['qacc'] = np.squeeze(obs_dict['qacc'])
        obs_dict_squeeze['qvel'] = np.squeeze(obs_dict['qvel'])
        obs_dict_squeeze['act'] = np.squeeze(obs_dict['act'])
        obs_dict_squeeze['qfrc_actuator'] = np.squeeze(obs_dict['qfrc_actuator'])
        obs_dict_squeeze['qfrc_constraint'] = np.squeeze(obs_dict['qfrc_constraint'])
        obs_dict_squeeze['pose_err'] = np.squeeze(obs_dict['pose_err'])
        obs_dict_squeeze['inter_force'] = np.squeeze(obs_dict['inter_force'])
        return obs_dict_squeeze

    def get_inter_force(self, pos, torque):
        m = 68 / 1.57
        torque_pos = m * pos - 64
        torque_ext = torque - torque_pos
        #print("Position",pos, "   Torque", torque, "Torque ext:", torque_ext)
        return torque_ext

    
    def cumulative_rw(self):
        self.cumulative_reward_value += 0.01
        return self.cumulative_reward_value
    def reset_cumulative_reward(self):
        self.cumulative_reward_value = 0
    
    def get_pose_reward(self, pose_err, vel):
        # generates pose reward based on qpos
        pose_reward = np.exp((-(1.*pose_err))/(2.*np.power(self.sigma,2)))
        lineal_penalty = np.linalg.norm(pose_err)
        pose_reward =  pose_reward - lineal_penalty
        
        if (pose_err < 0.05) and (vel< 0.05):
            pose_reward += self.cumulative_rw()
        else:
            self.reset_cumulative_reward()
        
        return pose_reward

    
    def get_inter_force_penalty(self, torque, force_threshold):
        
        """
        inter_force_penalty = np.linalg.norm(torque - self.torque_prev)**2
        # Penalize both positive and negative overshoots
        if abs(torque) > force_threshold:
            threshold_penalty = (abs(torque) - force_threshold)**4
        else:
            threshold_penalty = 0
        inter_force_penalty += threshold_penalty
        self.torque_prev = torque
        """

        inter_force_penalty = np.linalg.norm(torque - self.torque_prev)**2
        if torque > force_threshold and torque < -25:
            threshold_penalty = (np.linalg.norm(torque - force_threshold))**4
        else:
            threshold_penalty = 0
        inter_force_penalty += threshold_penalty
        self.torque_prev = torque
        
        return inter_force_penalty
    
    def get_acc_penalty(self, exo_acc, knee_vel):
        #acc_penalty = np.linalg.norm(exo_acc - self.qacc_prev)**2
        #self.qacc_prev = exo_acc
        vel_penalty = np.linalg.norm(knee_vel)
        acc_penalty = vel_penalty
        return acc_penalty
    
    def get_action_penalty(self):
        action_penalty = np.linalg.norm(self.action_prev1 - self.action_prev2)**2
        return action_penalty
    
    def get_reward_dict(self, obs_dict):

        obs_dict_squeeze = self.squeeze_obs_dict(obs_dict)
        actual_pos = obs_dict_squeeze['qpos'][6]
        pose_err = obs_dict_squeeze['pose_err']
        knee_vel = obs_dict_squeeze['qvel'][6]
        inter_force_leg = obs_dict_squeeze['inter_force']
        knee_constraint = obs_dict_squeeze['qfrc_constraint'][6]
        actuator_force =  obs_dict_squeeze['qfrc_actuator'][6]
        act_exo = obs_dict_squeeze['act']
        exo_acc = obs_dict_squeeze['qacc'][6]
        force_threshold = 75

        #Pose reward
        pose_reward = self.get_pose_reward(pose_err=pose_err, vel=knee_vel)
        #Acc penalty
        acc_penalty = self.get_acc_penalty(exo_acc, knee_vel)
        #Interaction penalty
        inter_force_penalty = self.get_inter_force_penalty(torque=actuator_force, force_threshold=force_threshold)
        #Action penalty 
        action_penalty=self.get_action_penalty()
        #Done conditions
        done_high = ((actual_pos > self.target_jnt_range[0][1]) & (inter_force_leg > 25)).astype(float)
        done_low = ((actual_pos < self.target_jnt_range[0][0]) & (inter_force_leg > 25)).astype(float)
        rwd_dict = collections.OrderedDict((

            # Optional Keys
            ('pose',    1.*pose_reward),
            ('acceleration',    -1.*acc_penalty),
            ('inter_force',  -1.*inter_force_penalty),
            ('penalty_high', -1.*(actual_pos > self.target_jnt_range[0][1]).astype(float)),
            ('penalty_low', -1.*(actual_pos < self.target_jnt_range[0][0]).astype(float)),
            ('smooth_act', -1*(action_penalty)),
            
            # Must keys
            ('sparse',  0),
            ('solved',  pose_err<self.sigma*0.1),
            ('done', (done_high + done_low).astype(float)), 
        ))       
        rwd_dict['dense'] = np.sum([wt*rwd_dict[key] for key, wt in self.rwd_keys_wt.items()], axis=0)
        #print("Reward", rwd_dict['dense'], "Error", pose_err, "Pos_score", pose_reward*5, "Inter_penalty:", -0.001*(inter_force_penalty), "acceleration_penalty", -0.001*acc_penalty)
        return rwd_dict

    def random_init(self, **kwargs):
        with open(ANGLES_DICT_PATH, 'rb') as file:
            angles_dict = pickle.load(file)
        init_pos = random.randint(0, 90)
        random_qpos = angles_dict[init_pos][1:]
        print("random pos:", init_pos)
        obs = super().reset(reset_qpos=random_qpos, **kwargs)
        return obs
    
    def set_target_qpos(self, target_qpos):
        target_qpos = target_qpos*(2*np.pi/360)
        if isinstance(target_qpos, torch.Tensor):
            self.target_qpos = np.array([target_qpos.item()])
        else:
            self.target_qpos = np.array([target_qpos])
        print('New target qpos:', self.target_qpos)

    def set_spasticity_level(self, spasticity_level):
        self.spasticity_level = spasticity_level
        """
        if isinstance(spasticity_level, torch.Tensor):
            self.pasticity_level = np.array([spasticity_level.item()])
        else:
            self.pasticity_level = np.array([spasticity_level])
        """
        #print('New spas_level:', self.spasticity_level)

    # reset_type = none; init; random
    # target_type = generate; fixed
    def reset(self, **kwargs):
        
        # update init state
        if self.reset_type is None or self.reset_type == "none":
            # no reset; use last state
            ## NOTE: fatigue is also not reset in this case!
            obs = self.get_obs()
        elif self.reset_type == "init":
            # reset to init state
            obs = super().reset(**kwargs)
        elif self.reset_type == "random":
            # reset to random state
            #jnt_init = self.np_random.uniform(high=self.sim.model.jnt_range[:,1], low=self.sim.model.jnt_range[:,0])
            obs = self.random_init()
    
        elif self.reset_type == "fixed": 
             # Exo = 3rd, Knee = 7th qpos
             # position with exo angle = -90º, knee angle = 90º
             qpos = [0.005082630840485775, 0.033695272921116896, -1.6419641702281853, -2.978484348923629, 0.006955509476829092, 0.001288023442163489, 1.5439362618219066, 0.012157153984297496, 0.2594406777004281, 1.6269485818730451, -0.6989795193633237, 0.35052974007072063, -0.5246066599479225, -0.0340516687698901, -0.008846460674014444, -1.238730497394359]
             # position for knee angle = 0º
             #qpos = [0.005834685602873783, -0.00653851963837641, -1.610314825968498, -0.6692592235178847, 0.00566594464090885, 0.0009114920866788922, 1.4722420075124696, 0.006470171955450531, 0.056191264122378036, -0.1603820765979743, -0.4011910332346481, 0.3400333691081888, -0.16811998188958785, -0.017757503091114304, 0.02614206520113357, -1.3097044945216785]
             obs = super().reset(reset_qpos=qpos, **kwargs)
        else:
            print("Reset Type not found")
        
        if self.target_type == 'generate':
            self.set_target_qpos(target_qpos=random.randint(0, 90))
        elif self.target_type == 'fixed':
            self.target_qpos = self.target_qpos
        
        if self.spasticity == 'active':
            #self.spasticity_level = random.randint(0,3) 
            self.spasticity_args = self.randomize_spas(level=self.spasticity_level, randomize=True)
            #print("Spasticity level:", self.spasticity_level)
        else:
            self.spasticity_args = self.randomize_spas(level=self.spasticity_level, randomize=False)
        
        """
        # Apply domain randomization if enabled
        if self.dr_enabled:
            self.apply_domain_randomization()
        """
        
        """
        # Log episode metrics before resetting
        if hasattr(self, 'episode_metrics') and self.episode_metrics['action_smoothness']:
            print(f"Episode metrics: "
                  f"avg_smoothness={np.mean(self.episode_metrics['action_smoothness']):.4f}, "
                  f"max_torque={np.max(self.episode_metrics['torque_peaks']):.2f}, "
                  f"saturations={self.episode_metrics['saturation_count']}")
            self.episode_metrics = {k: [] if isinstance(v, list) else 0 
                                   for k, v in self.episode_metrics.items()}
        """
        
        return obs


    def set_env_state(self, qpos, **kwargs):
        """
        Set full state of the environemnt
        Default implemention provided. Override if env has custom state
        """
        obs = super().reset(reset_qpos=qpos, **kwargs)

        return obs
    
    def compute_and_set_muscle_damping(self):
        """
        Compute and apply vmax for muscle actuators to ensure stable damping on real hardware.
        Based on paper 2402.05371, Eq. (11): β = k_damp / (4 * a_k * f_max)
        With β = 1 / vmax, we get: vmax = 4 * a_k * f_max / k_damp
        """
        # Check if using MuscleModel wrapper
        if hasattr(self, 'muscle_model'):
            # MuscleModel wrapper case (like the script from paper authors)
            a_k = 1.0  # Get from muscle_model.moment_1/moment_2
            f_max = 100.0  # Get from muscle_model.fmax
            vmax_optimal = (4.0 * a_k * f_max) / self.k_damp_target
            self.muscle_model.set_vmax(vmax_optimal)
            print(f"[MUSCLE DAMPING] vmax={vmax_optimal:.4f} applied via MuscleModel wrapper")
        
        # MuJoCo native muscle actuators (your current case)
        elif self.sim.model.na > 0:
            muscle_act_ids = np.where(self.sim.model.actuator_dyntype == mujoco.mjtDyn.mjDYN_MUSCLE)[0]
            
            if len(muscle_act_ids) > 0:
                print(f"[MUSCLE DAMPING] Found {len(muscle_act_ids)} muscle actuators, applying vmax...")
                
                for muscle_id in muscle_act_ids:
                    # Extract moment arm (a_k) from gear
                    a_k = abs(self.sim.model.actuator_gear[muscle_id, 0])
                    
                    # Extract max isometric force (f_max) from forcerange
                    f_max = np.max(np.abs(self.sim.model.actuator_forcerange[muscle_id]))
                    
                    if a_k > 1e-6 and f_max > 1e-6:  # Avoid division by zero
                        vmax_optimal = (4.0 * a_k * f_max) / self.k_damp_target
                        
                        # Apply vmax to MuJoCo model (stored in actuator_user[id, 0])
                        if self.sim.model.nuser_actuator >= 1:
                            self.sim.model.actuator_user[muscle_id, 0] = vmax_optimal
                        
                        # Store for debugging (only print first and last to avoid spam)
                        if muscle_id == muscle_act_ids[0] or muscle_id == muscle_act_ids[-1]:
                            print(f"  Muscle[{muscle_id}]: a_k={a_k:.3f}, f_max={f_max:.1f}N, vmax={vmax_optimal:.2f}")
                
                print(f"[MUSCLE DAMPING] ✓ vmax applied to all {len(muscle_act_ids)} muscles")
            else:
                print("[MUSCLE DAMPING] ⚠️ No muscle actuators found in model")
        else:
            print("[MUSCLE DAMPING] ⚠️ No actuators in model, skipping vmax setup")
    
    def apply_domain_randomization(self):
        """Apply domain randomization to reduce sim-to-real gap."""
        if not self.dr_enabled:
            return
        
        # Randomize friction
        friction_mult = np.random.uniform(*self.dr_ranges['friction'])
        self.sim.model.geom_friction[:, 0] *= friction_mult
        
        # Randomize damping
        damp_mult = np.random.uniform(*self.dr_ranges['damping'])
        self.sim.model.dof_damping[6] *= damp_mult  # Knee joint
        
        # Randomize mass (uncomment and adjust body_id when identified)
        # mass_mult = np.random.uniform(*self.dr_ranges['mass_multiplier'])
        # self.sim.model.body_mass[leg_body_id] *= mass_mult
    
    def randomize_spas(self, level, randomize):
        
        # Modify parameters following the spasticity level
        sigma = 0.1 if randomize else 0
        params_by_level = {
            #k_ang / a0_flexion / a0_extension / L_ang / L_vel / vel_limit
            0: [5, -35, 95, 0, 0, 100],
            1: [0.3, -35, 80, 0.2, 0.2, 1],
            2: [0.05, -30, 75, 0.3, 0.2, 0.5],
            3: [0.05, -10, 70, 0.5, 0.3, 0.25]
        }
        spasticity_args = params_by_level.get(level, [0, 0, 0, 0, 0, 0])
        spasticity_args = [param + np.random.normal(0, sigma) for param in spasticity_args]

        return spasticity_args
    def ad_spasticity(self, muscle_a):
        #   in sim.data.ctrl, [0] is for exo_motor and [1:40] for 40 muscles
        #    Spastic Muscles
        #    # Harmstrings
        #    muscle_a[32]  #Semimembranosus
        #    muscle_a[33]  #Semitendinosus
        #    muscle_a[7]   #Biceps Sural Long
        #    muscle_a[8]   #Biceps Sural Short
        #    # Calf
        #    muscle_a[13]  #Gastrocnemius Lateral
        #    muscle_a[14]  #Gastrocnemius Medial
        #    # Rectus 
        #    muscle_a[24]  #Gracillis
        #    # Quadriceps
        #    muscle_a[40]  #Vastus medial
        #    muscle_a[39]  #Vastus lateral
        #    muscle_a[38]  #Vastus intermedius
        #    muscle_a[30]  #Rectus femoral   
        knee_angle = (360/2*np.pi)*self.sim.data.qpos[6]
        knee_vel = self.sim.data.qvel[6]

        # Spasticity parameters
        k_ang=self.spasticity_args[0]
        a0_flexion=self.spasticity_args[1]
        a0_extension=self.spasticity_args[2]
        L_ang=self.spasticity_args[3]
        L_vel=self.spasticity_args[4]
        vel_limit=self.spasticity_args[5]
        
        # Sigmoid application Angle dependent
        # Sigmoid ecuation for flexion and extension (range)
        sigmoid_flexion_ang = 1 / (1 + np.exp(-k_ang * (knee_angle - a0_flexion)))  # Sigmoide izquierda
        sigmoid_extension_ang = 1 / (1 + np.exp(np.clip(k_ang * (knee_angle - a0_extension), -500, 500)))  # Sigmoide derecha
        spas_coef_ang =L_ang + (L_ang * (1 - (sigmoid_flexion_ang + sigmoid_extension_ang)))
        
        # Sigmoid application Velocity dependent
        k = 5
        v0_flexion = -vel_limit
        v0_extension = vel_limit
        slope_increase = 0.05
        # Sigmoid ecuation for flexion and extension (velocity)
        sigmoid_flexion = 1 / (1 + np.exp(-k * (knee_vel - v0_flexion)))  # Sigmoide izquierda
        sigmoid_extension = 1 / (1 + np.exp(k * (knee_vel - v0_extension)))  # Sigmoide derecha
        spas_coef_vel = L_vel + (L_vel * (1 - (sigmoid_flexion + sigmoid_extension)))
        spas_coef_vel = spas_coef_vel + slope_increase * (v0_flexion - knee_vel)
        spas_coef_vel = spas_coef_vel + slope_increase * (knee_vel - v0_extension)
        # Merge coef  
        spas_coef = spas_coef_vel + spas_coef_ang
        
        if knee_vel > 0:    # Flexion
            muscles_index = [30,38,39,40]
            muscle_a[muscles_index] += spas_coef  
        else:               # Extension
            muscles_index = [7,8,13,14,24,32,33]
            muscle_a[muscles_index] += spas_coef  

        return muscle_a
    
    def process_msg(self, msg):
        """
        This method changes the message from a string to a float list
        From "1.0, 2.0, 3.0, 4.0" into [1.0, 2.0, 3.0, 4.0]
        """
        variables = msg.split(", ")  # We separe the message assuming it is a comma-separated string
        i =  0
        for var in variables:
            variables[i] = float(var)
            i += 1
        #print(f"Processed variables: {variables}")
        return variables


    def step(self, a, **kwargs):
        
        muscle_a = a.copy()
        #print(muscle_a, "-----")
        muscle_act_ind = self.sim.model.actuator_dyntype == mujoco.mjtDyn.mjDYN_MUSCLE
        # Explicitely project normalized space (-1,1) to actuator space (0,1) if muscles
        if self.spasticity_level > 0:
            muscle_a = self.ad_spasticity(muscle_a)
        
        if self.sim.model.na and self.normalize_act:
            # find muscle actuators
            muscle_a[muscle_act_ind] = 1.0 / (
                1.0 + np.exp(-5.0 * (muscle_a[muscle_act_ind] - 0.5))
            )
            
            # TODO: actuator space may not always be (0,1) for muscle or (-1, 1) for others
            isNormalized = (
                False  # refuse internal reprojection as we explicitly did it here
            )
        else:
            isNormalized = self.normalize_act  # accept requested reprojection
        #print(muscle_a, type(muscle_a))
        #Action smooth filter
        self.action_prev2 = self.action_prev1
        self.action_prev1 = muscle_a[0]
        # step forward
        self.last_ctrl = self.robot.step(
            ctrl_desired=muscle_a,
            ctrl_normalized=isNormalized,
            step_duration=self.dt,
            realTimeSim=self.mujoco_render_frames,
            render_cbk=self.mj_render if self.mujoco_render_frames else None,
        )
        #print(muscle_a, "-----")
        self.excess_force_prev = self.sim.data.qfrc_constraint[6]


        """
        # OLD SOCKET CODE (commented out)

        #Here we set our motor socket, first we send the action and then we receive the observation
        client_socket, addr = self.socket_server.accept() # This line blocks the programme until a socket client is connected
        #print(f"Connection from {addr} has been established!")

        try:        # We send the action as a string through the socket
            action = str(muscle_a[0])
            client_socket.sendall(action.encode())

        finally:    # We close the socket
            client_socket.close()
            #print("Connection closed.")
    
        client_socket, addr = self.socket_server.accept() # This line blocks the programme until a socket client is connected
        #print(f"Connection from {addr} has been established!")

        try:        # We receive the information from the motor and substitute it in the observations
            data = client_socket.recv(1024) # We receive the message from the socket
            #print(f"Received observation before parsing: {data.decode()}")
            observation = self.process_msg(data.decode()) # We transform the observations into a float list
            #print(f"Received observation after parsing: {observation}")

            # We change the observations using the values from the motor
            self.sim.data.qpos[6] = observation[0]
            self.sim.data.qvel[6] = observation[1]
            self.sim.data.qacc[6] = observation[2]
            self.sim.data.qfrc_constraint[6] = observation[3]

        finally:    # We close the socket
            client_socket.close()
            #print("Connection closed.")
        """

        return self.forward(**kwargs)
