# =================================================
#	This model is generated with tacking the Myosuite conversion of [Rajagopal's full body gait model](https://github.com/opensim-org/opensim-models/tree/master/Models/RajagopalModel) as close
#reference.
#	Model	  :: MyoLeg 1 Dof Exo (MuJoCoV2.0)
#	Author	:: Andres Chavarrias (andreschavarriassanchez@gmail.com), David Rodriguez, Pablo Lanillos 
#	source	:: https://github.com/AndresChS/NAIR_Code
#	====================================================== -->
from pathlib import Path

from myosuite.utils import gym; register=gym.register
import os
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
MODEL_XML = BASE_DIR / "myoleg_spasticity_muscexo" / "myoleg_spasticity_muscexo_V0.xml"

register(id='ExoLegSpasticityFlexoExtEnv-v0',
    #entry_point='myosuite.envs.nair.MyolegenvV0:Myoleg_env_v0', # where to find the new Environment Class
    entry_point='myosuite.envs.nair.M1640_KneeExoSim2RealV0:Myoleg_env_v0', # where to find the new Environment Class
    max_episode_steps=256, # duration of the episode
    kwargs={
        'model_path': str(MODEL_XML), # where the xml file of the environment is located
        'target_jnt_range': {'knee_angle_r':(0, 1.57)},
        'target_qpos': 0.2,
        'sigma': .2,
        'target_type': 'fixed',
        'normalize_act': True,
        'reset_type': 'fixed',
        'spasticity': 'passive'
    }
)


register(id='M1640KneeExoSim2Real-v0',
    entry_point='myosuite.envs.nair.M1640_KneeExoSim2RealV0:Myoleg_env_v0', # where to find the new Environment Class
    max_episode_steps=256, # duration of the episode
    kwargs={
        'model_path': str(MODEL_XML), # where the xml file of the environment is located
        'target_jnt_range': {'knee_angle_r':(0, 1.57)}, # 0 estirado, 1.57 doblado
        'target_qpos': 0.5,
        'sigma': .2,
        'target_type': 'fixed',
        'normalize_act': True,
        'reset_type': 'fixed',
        'spasticity': 'passive'
    }
)


# ***************************************************************
#                       Future works
# ***************************************************************

#register(id='ExoLeg9MuscFlexoExtEnv-v0',
#    entry_point='myosuite.envs.nair.MyolegenvV0:Myoleg_env_v0', # where to find the new Environment Class
#    max_episode_steps=200, # duration of the episode
#    kwargs={
#        'model_path': '/Users/achs/Documents/PHD/code/myosuite/envs/myoleg_1dof9muscexo/myoleg_1dof9muscexo_V0.xml', # where the xml file of the environment is located
#        'target_jnt_range': {'knee_angle_r':(0, 2.27),},
#        'normalize_act': True,
#        'pose_thd': .175,
#        'reset_type': 'random'
#    }
#)

#register(id='ExoSuspendedLegFlexoExtEnv-v0',
#    entry_point='myosuite.envs.nair.MyolegenvV0:Myoleg_env_v0', # where to find the new Environment Class
#    max_episode_steps=200, # duration of the episode
#    kwargs={
#        'model_path': '/Users/achs/Documents/PHD/code/myosuite/envs/myoleg_suspendedexo/myoleg_suspendedexo_V0.xml', # where the xml file of the environment is located
#        'target_jnt_range': {'knee_angle_r':(0, 2.27),},
#        'normalize_act': True,
#        'pose_thd': .175,
#        'reset_type': 'random'
#    }
#)
