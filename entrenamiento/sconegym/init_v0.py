import os
from gymnasium.envs.registration import register
curr_dir = os.path.dirname(os.path.abspath(__file__))

# ====================================================================
# Sconegym envs Environments
# --------------------------------------------------------------------
# H0918
register(id="sconewalk_h0918-gym-v0",
         entry_point="sconegym.gaitgym:GaitGym",
         kwargs={
             'model_file': curr_dir + '/data-v1/H0918.scone',
             'obs_type': '2D',
             'left_leg_idxs': [3, 4, 5],
             'right_leg_idxs': [6, 7, 8],
             'clip_actions': True,
             'run': False,
             'target_vel': 1.2,
             'leg_switch': True,
             'rew_keys':{
                "vel_coeff": 10,
                "grf_coeff": -0.07281,
                "joint_limit_coeff": -0.1307,
                "smooth_coeff": -0.097,
                "nmuscle_coeff": -1.57929,
                "self_contact_coeff": 0.0,
             }
         }
        )

# ***********************  Nair Envs definition *********************************************************************+
# ***************** H0404-0412 *******************
# MIMO_EXO (H0404)
# Gait
register(id="nair_gait_h0404MimoExo-v0",
        entry_point="sconegym.nair_mimoEnv:MimoGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0404_MimoExo/H0404_MimoExoV0.scone',
            'model_HFD': curr_dir + '/nair_envs/H0404_MimoExo/models/H0404_MimoExoV0.hfd',
            'passive_subject': True, 
            'exo_assist': True,
            'clip_actions': False,
            'exo_type': 'motor',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )

# MIMO_EXO (H0412)
# Gait
register(id="nair_gait_h0412MimoExo-v0",
        entry_point="sconegym.nair_mimoEnv:MimoGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0412_MimoExo/H0412_MimoExoV0.scone',
            'model_HFD': curr_dir + '/nair_envs/H0412_MimoExo/models/H0412_MimoExoV0.hfd',
            'passive_subject': True, 
            'exo_assist': True,
            'exo_type': 'motor',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )

# ***************** H0109-0105 *******************
# KNEE_EXO (H0105)
# Flexion
register(id="nair_flexo_h0105KneeExo-v0",
        entry_point="sconegym.nair_flexoEnv:FlexoGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0109_KneeExo/H0105_KneeExoFlexoV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )

# ***************** H0918 ************************
# Gait
register(id="nair_gait_h0918-v1",
        entry_point="sconegym.nair_gaitEnv:SconeGaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918/H0918_GaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# Hyperreflexia Gait
register(id="nair_gait_h0918_hyperreflexia-v1",
        entry_point="sconegym.nair_motor_disorders:SpasGaitEnv",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918/H0918_GaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# Weakness Gait
register(id="nair_gait_h0918_weakness-v0",
        entry_point="sconegym.nair_motor_disorders:WeakGaitEnv",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918/H0918_GaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'randomize_weakness': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# Stand
register(id="nair_stand_h0918-v0",
        entry_point="sconegym.nair_standupEnv:StandupGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918/H0918_StandV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.005,
            'obs_type': '2D_Gait_Scone',
            'rew_keys':{
                    "pelvis_height": 1.0,
                    "pelvis_displacement": 1.0,
                    "pelvis_ori": 0.2,
                    "pelvis_vel": 0.0,
                    "number_muscles": 0.01,
                    "foots_contact": 0.0,
                    "symmetric_act": -0.0,
                    "solved": 1000.0,
                    "done": 0.0,  # Not working at the beginning
            }
        }
    )
# Balance
register(id="nair_balance_h0918-v0",
        entry_point="sconegym.nair_balanceEnv:BalanceGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918/H0918_BalanceV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.005,
            'obs_type': 'MS',
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )

# KNEE_EXO (H0918)
# Stand
register(id="nair_stand_h0918KneeExo-v0",
        entry_point="sconegym.nair_standupEnv:StandupGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoStandV0.scone',
            'exo' : True,
            'max_episode_steps': 500,
            'step_size': 0.005,
            'obs_type': '2D_Gait_Scone',
            'rew_keys':{
                    "pelvis_height": 1.0,
                    "pelvis_displacement": 1.0,
                    "pelvis_ori": 0.2,
                    "pelvis_vel": 0.0,
                    "number_muscles": 0.01,
                    "foots_contact": 0.0,
                    "symmetric_act": -0.0,
                    "solved": 1000.0,
                    "done": 0.0,  # Not working at the beginning
            }
        }
    )
# Gait
register(id="nair_gait_h0918KneeExo-v0",
        entry_point="sconegym.nair_gaitEnv:GaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoGaitV0.scone',
            'exo' : True,
            'max_episode_steps': 1000,
            'step_size': 0.005,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'clip_actions': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "solved": 0.0,
                "done": 0.0,  # Not working at the beginning
            }
        }
    )

# HIP EXO (H0918)
# Gait
register(id="nair_gait_h0918HipExo-v0",
        entry_point="sconegym.nair_gaitEnv:SconeGaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_HipExo/H0918_HipExoGaitV0.scone',
             #'passive_subject': True, #Synergy envs can not be passive
            'exo_assist': True,
            'exo_type': 'rope',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# Weakness Gait
register(id="nair_gait_h0918_HipExo_Weakness-v0",
        entry_point="sconegym.nair_motor_disorders:WeakGaitEnv",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_HipExo/H0918_HipExoGaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '2D_Gait_Scone',
            'left_leg_idxs': [3, 4, 5],
            'right_leg_idxs': [6, 7, 8],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'randomize_weakness': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# *********************** H1622 ************************
# Balance
register(id="nair_balance_h1622-v0",
        entry_point="sconegym.nair_balanceEnv:BalanceGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H1622/H1622_BalanceV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.005,
            'obs_type': 'MS',
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )
# Gait
register(id="nair_gait_h1622-v0",
        entry_point="sconegym.nair_gaitEnv:SconeGaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H1622/H1622_GaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '3D_Gait_Scone',
            'left_leg_idxs': [6, 7, 8, 9, 10],
            'right_leg_idxs': [11, 12, 13, 14, 15],
            'run': False,
            'clip_actions': True,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                #"solved": 1000.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )

# weakness Gait
register(id="nair_gait_h1622_Weakness-v0",
        entry_point="sconegym.nair_motor_disorders:WeakGaitEnv",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H1622/H1622_GaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '3D_Gait_Scone',
            'left_leg_idxs': [6, 7, 8, 9, 10],
            'right_leg_idxs': [11, 12, 13, 14, 15],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'randomize_weakness': False,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# HIP EXO (H1622)
# Gait
register(id="nair_gait_h1622HipExo-v0",
        entry_point="sconegym.nair_gaitEnv:SconeGaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H1622/H1622_HipExoGaitV0.scone',
             #'passive_subject': True, #Synergy envs can not be passive
            'exo_assist': True,
            'exo_type': 'rope',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '3D_Gait_Scone',
            'left_leg_idxs': [6, 7, 8, 9, 10],
            'right_leg_idxs': [11, 12, 13, 14, 15],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# Weakness Gait
register(id="nair_gait_h1622_HipExo_Weakness-v0",
        entry_point="sconegym.nair_motor_disorders:WeakGaitEnv",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H1622/H1622_HipExoGaitV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.025,
            'obs_type': '3D_Gait_Scone',
            'left_leg_idxs': [6, 7, 8, 9, 10],
            'right_leg_idxs': [11, 12, 13, 14, 15],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'randomize_weakness': False,
            'weakness_level': 0,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                "self_contact_coeff": 0.0,
                #"solved": 0.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )
# *********************** H2190 ************************
# Gait
register(id="nair_gait_h2190-v0",
        entry_point="sconegym.nair_gaitEnv:GaitGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H2190/H2190.scone',
            'max_episode_steps': 1000,
            'step_size': 0.005,
            'obs_type': '3D_Gait_Scone',
            'left_leg_idxs': [6, 7, 8, 9, 10, 11],
            'right_leg_idxs': [12, 13, 14, 15, 16, 17],
            'clip_actions': True,
            'run': False,
            'target_vel': 1.2,
            'leg_switch': True,
            'rew_keys':{
                "gaussian_vel": 10.0,
                "grf": -0.07281,
                "smooth": -0.097,
                "number_muscles": -1.57929,
                "constr": -0.1307,
                #"solved": 1000.0,
                #"done": 0.0,  # Not working at the beginning
            }
        }
    )

# ====================================================================
# DEP Synergies Environments (PCA-ICA)
# --------------------------------------------------------------------
# ***************** H0109-0105 *******************
# KNEE_EXO 
# Flexion with synergies
register(id="nair_flexo_h0105KneeExoSyn-v0",
        entry_point="sconegym.nair_flexoEnv:FlexoGymSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0109_KneeExo/H0105_KneeExoFlexoV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 2,
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )

register(id="nair_flexo_h0105KneeExoSynSpasticity-v0",
        entry_point="sconegym.nair_flexoEnv:FlexoGymSynSpasticity",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0109_KneeExo/H0105_KneeExoFlexoV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 2,
            'spasticity_level': 3,
            'randomize_spasticity': False,
            'passive_subject': True,
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )  

# ***************** H0918 ************************
# KNEE_EXO (H0918)
# Stand with synergies
register(id="nair_stand_h0918KneeExoSyn-v0",
        entry_point="sconegym.nair_standupEnv:StandupGymSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoStandV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'exo_assist': True,
            'obs_type': 'MS',
            'num_synergies': 9,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )
# Stand with synergies and force
register(id="nair_stand_h0918KneeExoForceSyn-v0",
        entry_point="sconegym.nair_standupEnv:StandupGymForceSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoStandV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 9,
            'applied_force': 298.48,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )

# Balance with synergies and force
register(id="nair_balance_h0918KneeExoForceSyn-v0",
        entry_point="sconegym.nair_balanceEnv:BalanceGymForceSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoBalanceV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 9,
            'applied_force': 96.48,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )

# Gait with synergies
register(id="Nair_Gait_H0918_KneeExoSyn-v0",
        entry_point="sconegym.nair_gaitEnv:GaitGymSyn",
        kwargs={
        'model_file': curr_dir + '/nair_envs/H0918_KneeExo/H0918_KneeExoGaitV0.scone',
        'passive_subject': False, #Synergy envs can not be passive
        'exo_assist': True,
        'exo_type': 'motor',
        'obs_type': 'Gait_Scone',
        'num_synergies': 9,
        'left_leg_idxs': [3, 4, 5],
        'right_leg_idxs': [0, 1, 2],
        'run': False,
        'target_vel': 1.0,
        'leg_switch': False,
        'rew_keys':{
            "gaussian_vel": 10.0,
            "grf": -0.07281,
            "smooth": -0.097,
            "number_muscles": -1.57929,
            "constr": -0.1307,
            "solved": 1000.0,
            "done": 0.0,  # Not working at the beginning
            }
        }
    )

# HIP EXO (H0918)
# Stand with synergies
register(id="nair_stand_h0918HipExoSyn-v0",
        entry_point="sconegym.nair_standupEnv:StandupGymSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_HipExo/H0918_HipExoStandV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'exo_assist': True,
            'exo_type': 'rope',
            'obs_type': 'MS',
            'num_synergies': 9,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )
# Balance with synergies and force
register(id="nair_balance_h0918HipExoForceSyn-v0",
        entry_point="sconegym.nair_balanceEnv:BalanceGymForceSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0918_HipExo/H0918_HipExoBalanceV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 9,
            'applied_force': 96.48,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )

# *********************** H2190 ************************
# Gait with synergies
register(id="nair_gait_h2190Syn-v0",
        entry_point="sconegym.nair_standupEnv:StandupGym",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H2190/H2190.scone',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'obs_type': 'MS',
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.1,
            }
        }
    )

# Balance with synergies and force
register(id="nair_balance_h2190ForceSyn-v0",
        entry_point="sconegym.nair_balanceEnv:BalanceGymForceSyn",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H2190/H2190_BalanceV0.scone',
            'max_episode_steps': 500,
            'step_size': 0.01,
            'obs_type': 'MS',
            'num_synergies': 21,
            #'applied_force': 96.48,
            'rew_keys':{
                "pelvis_pos": 1.0,
                "number_muscles": 0.0,
            }
        }
    )

# ====================================================================
# Motor Disorders Environments
# --------------------------------------------------------------------
# KNEE_EXO H0109-0105
register(id="nair_flexo_h0105KneeExoSpasticity-v0",
        entry_point="sconegym.nair_flexoEnv:FlexoGymSpasticity",
        kwargs={
            'model_file': curr_dir + '/nair_envs/H0109_KneeExo/H0105_KneeExoFlexoV0.scone',
            'max_episode_steps': 1000,
            'step_size': 0.01,
            'exo_assist': True,
            'obs_type': 'MS',
            'rew_keys':{
                "knee_angle": 1.0,
                "number_muscles": 0.1,
            }
        }
    )


