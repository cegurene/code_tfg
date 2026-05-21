#=================================================
#	This model is generated with tacking the Myosuite conversion of [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close
#reference.
#	Model	  :: MyoLeg 1 Dof 40 Musc Exo (MuJoCoV2.0)
#	Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), David Rodriguez, Pablo Lanillos 
#	source	:: https://github.com/AndresChS/NAIR_Code
#	====================================================== -->
import collections
from pathlib import Path
from myosuite.utils import gym
import numpy as np
import mujoco
from myosuite.envs.myo.base_v0 import BaseV0
from myosuite.envs import env_base
import random
import pickle

ANGLES_DICT_PATH = Path(__file__).resolve().with_name("angles_dict.pkl")

class Myoleg_env_v0(BaseV0,env_base.MujocoEnv):

    MYO_CREDIT = """\
    This model is generated with tacking the Myosuite conversion of [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close reference.
        Model	:: MyoLeg 1 Dof 40 Musc Exo (MuJoCoV2.0)
        Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), David Rodriguez, Pablo Lanillos 
        Source	:: https://github.com/AndresChS/NAIR_Code
    """
    
    DEFAULT_OBS_KEYS = ['time', 'qpos', 'qvel', 'act', 'cfrc_ext', 'pose_err']
    DEFAULT_RWD_KEYS_AND_WEIGHTS = {
        "pose": 0.1,
        "inter_force": 0,
        #"act_reg": 0,
        "velocity": 0,
        "penalty_high": 1000.0,
        "penalty_low": 1000.0
    }

    def __init__(self, model_path, obsd_model_path=None, seed=None, **kwargs):

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

            sigma = 0.5,
            weight_bodyname = None,
            weight_range = None,
            **kwargs,
        ):
        self.reset_type = reset_type
        self.target_type = target_type
        self.sigma = sigma
        self.spasticity = spasticity
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
        
        super()._setup(obs_keys=obs_keys,
                weighted_reward_keys=weighted_reward_keys,
                sites=viz_site_targets,
                **kwargs,
                )
    
    def get_obs_dict(self, sim):
        obs_dict = {}
        obs_dict['time'] = np.array([sim.data.time])
        obs_dict['qpos'] = sim.data.qpos
        obs_dict['qvel'] = sim.data.qvel
        obs_dict['qacc'] = sim.data.qacc
        obs_dict['act'] = sim.data.ctrl #if sim.model.na>0 else np.zeros_like(obs_dict['qpos'])
        obs_dict['cfrc_ext'] = sim.data.qfrc_constraint
        obs_dict['pose_err'] = np.absolute(self.target_qpos - sim.data.qpos[6])
        return obs_dict

    def squeeze_obs_dict(self, obs_dict):
        obs_dict_squeeze = {}
        obs_dict_squeeze['time'] = np.squeeze(obs_dict['time'])
        obs_dict_squeeze['qpos'] = np.squeeze(obs_dict['qpos'])
        obs_dict_squeeze['qacc'] = np.squeeze(obs_dict['qacc'])
        obs_dict_squeeze['qvel'] = np.squeeze(obs_dict['qvel'])
        obs_dict_squeeze['act'] = np.squeeze(obs_dict['act'])
        obs_dict_squeeze['cfrc_ext'] = np.squeeze(obs_dict['cfrc_ext'])
        obs_dict_squeeze['pose_err'] = np.squeeze(obs_dict['pose_err'])
        return obs_dict_squeeze


    def get_pose_reward(self, pose_err):
        # generates pose reward based on qpos
        pose_reward = np.exp((-(1.*pose_err))/(2.*np.power(self.sigma,2)))
        return pose_reward
    

    def get_reward_dict(self, obs_dict):

        obs_dict_squeeze = self.squeeze_obs_dict(obs_dict)
        actual_pos = obs_dict_squeeze['qpos'][6]
        pose_err = obs_dict_squeeze['pose_err']
        #Pose reward
        pose_reward = self.get_pose_reward(pose_err=obs_dict_squeeze['pose_err'])
        if (pose_err < 0.05) and (np.abs(obs_dict_squeeze['qvel'][6]) < 0.1):
            pose_reward = pose_reward*2
        #Velocity reward
        if np.abs(obs_dict_squeeze['qvel'][6]) > 4:
            velocity_penalty = np.absolute((obs_dict_squeeze['qvel'][6]))
        else:
            velocity_penalty = 0
        
        inter_force_leg = np.absolute(obs_dict_squeeze['cfrc_ext'][2]/10)
        exo_torque_generator = (np.absolute(obs_dict_squeeze['act']))
        done_high = (actual_pos > self.target_jnt_range[0][1]).astype(float)
        done_low = (actual_pos < self.target_jnt_range[0][0]).astype(float)
        #print("Act_penalty:", -1*(exo_torque_generator), "Pos_score", pose_reward, "Vel_penalty", velocity_penalty, "Interaction_penalty", -1*inter_force_leg )
        #print("Pos_score", pose_reward*1, "pose error", pose_err)
        rwd_dict = collections.OrderedDict((
            # Optional Keys
            ('pose',    1.*pose_reward),
            ('velocity',    -1.*velocity_penalty),
            ('inter_force',  -1.*inter_force_leg),
            ('penalty_high', -1.*(actual_pos > self.target_jnt_range[0][1]).astype(float)),
            ('penalty_low', -1.*(actual_pos < self.target_jnt_range[0][0]).astype(float)),
            ('act_reg', -1*(exo_torque_generator/100)),
            
            # Must keys
            ('sparse',  0),
            ('solved',  pose_err<self.sigma*0.1),
            ('done', (done_high + done_low).astype(float)), 
        ))       
        rwd_dict['dense'] = np.sum([wt*rwd_dict[key] for key, wt in self.rwd_keys_wt.items()], axis=0)
        #print(rwd_dict['dense'])
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
        if isinstance(target_qpos, torch.Tensor):
            self.target_qpos = np.array([target_qpos.item()])
        else:
            self.target_qpos = np.array([target_qpos])
        print('New target qpos:', self.target_qpos)

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
             # position with knee angle = 90º
             qpos = [0.005082630840485775, 0.033695272921116896, -1.6419641702281853, -2.978484348923629, 0.006955509476829092, 0.001288023442163489, 1.5439362618219066, 0.012157153984297496, 0.2594406777004281, 1.6269485818730451, -0.6989795193633237, 0.35052974007072063, -0.5246066599479225, -0.0340516687698901, -0.008846460674014444, -1.238730497394359]
             # position for knee angle = 0º
             #qpos = [0.005834685602873783, -0.00653851963837641, -1.610314825968498, -0.6692592235178847, 0.00566594464090885, 0.0009114920866788922, 1.4722420075124696, 0.006470171955450531, 0.056191264122378036, -0.1603820765979743, -0.4011910332346481, 0.3400333691081888, -0.16811998188958785, -0.017757503091114304, 0.02614206520113357, -1.3097044945216785]
             obs = super().reset(reset_qpos=qpos, **kwargs)
        else:
            print("Reset Type not found")
        
        if self.target_type == 'generate':
                self.set_target_qpos(target_qpos=random.uniform(0, 1.57))
        elif self.target_type == 'fixed':
                self.target_qpos = self.target_qpos
        
        return obs

    def set_env_state(self, qpos, **kwargs):
        """
        Set full state of the environemnt
        Default implemention provided. Override if env has custom state
        """
        obs = super().reset(reset_qpos=qpos, **kwargs)

        return obs
    
    def ad_spasticity(self, muscle_a, knee_vel):
        #   Muscles
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

        if np.absolute(knee_vel) > 2:
            knee_angle = (360/2*np.pi)*self.sim.data.qpos[6]
            spas_coef = 0.1*np.absolute(knee_vel)
            if knee_vel > 0:    # Flexion
                # Harmstrings
                muscle_a[7,8,13,14,24,32,33,38,39,40] = spas_coef  #Semimembranosus
            else:
            
                """
                muscle_a[33] = spas_coef  #Semitendinosus
                muscle_a[7] = spas_coef  #Biceps Sural Long
                muscle_a[8] = spas_coef  #Biceps Sural Short
                # Calf
                muscle_a[13] = spas_coef  #Gastrocnemius Lateral
                muscle_a[14] = spas_coef  #Gastrocnemius Medial
                # Rectus 
                muscle_a[24] = spas_coef  #Gracillis
                # Quadriceps
                muscle_a[40] = spas_coef  #Vastus medial
                muscle_a[39] = spas_coef  #Vastus lateral
                muscle_a[38] = spas_coef  #Vastus intermedius
                muscle_a[30] = spas_coef  #Rectus femoral 
            
            else:                # Extension
                # Calf
                muscle_a[13] = spas_coef  #Gastrocnemius Lateral
                muscle_a[14] = spas_coef  #Gastrocnemius Medial
                # Rectus 
                muscle_a[24] = spas_coef  #Gracillis
                # Quadriceps
                muscle_a[40] = spas_coef  #Vastus medial
                muscle_a[39] = spas_coef  #Vastus lateral
                muscle_a[38] = spas_coef  #Vastus intermedius 
                muscle_a[30] = spas_coef  #Rectus femoral
                # Harmstrings
                muscle_a[32] = spas_coef  #Semimembranosus
                muscle_a[33] = spas_coef  #Semitendinosus
                muscle_a[7] = spas_coef  #Biceps Sural Long
                muscle_a[8] = spas_coef  #Biceps Sural Short
                """
        return muscle_a

                #print(knee_qvel, "linealizada", knee_qvel*self.dt)
    def step(self, a, **kwargs):
        
        muscle_a = a.copy()
        #print(muscle_a, "-----")
        muscle_act_ind = self.sim.model.actuator_dyntype == mujoco.mjtDyn.mjDYN_MUSCLE
        # Explicitely project normalized space (-1,1) to actuator space (0,1) if muscles
        if self.spasticity=="active" and self.sim.model.na:
            knee_vel = self.sim.data.qvel[6]
            muscle_a = self.ad_spasticity(muscle_a, knee_vel)
            
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
        # step forward
        self.last_ctrl = self.robot.step(
            ctrl_desired=muscle_a,
            ctrl_normalized=isNormalized,
            step_duration=self.dt,
            realTimeSim=self.mujoco_render_frames,
            render_cbk=self.mj_render if self.mujoco_render_frames else None,
        )
        #print(muscle_a, "-----")
        
        
        return self.forward(**kwargs)