# NAIR_standupEnv.py

import numpy as np
import yaml
import os, sys
import gymnasium as gym
from sconegym.nair_sconegym import BasicSconeGymEnv, NairSconeGymEnv
from sconegym.nair_gaitEnv import SconeGaitGym
from sconegym.sconetools import sconepy
from typing import Optional
import sconepy # type: ignore
import collections

class SpasticityEnv(NairSconeGymEnv):
    """
    Base class for Spasticity Environments
    - Replicate Hyperreflexia by increasing the gain of selected muscles
    - These activation are based on Lassman et al. 2023 Frontiers in Neurorobotics and the Ashworth scale  
    - Depending on the spasticity level, increase the gain of selected muscles
    
    Inputs: 
    Normalized muscle lengths and lengths velocities
    
    Outputs:
    Muscle activations

    Levels of spasticity:
    - 0 : No spasticity
    - 1 : Mild spasticity (Ashworth 1)
    - 2 : Moderate spasticity (Ashworth 2)
    - 3 : Severe spasticity (Ashworth 3)
    - 4 : (Not implemented) Very severe spasticity (Ashworth 4)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.randomize_spasticity = kwargs.get('randomize_spasticity', True)
        if self.randomize_spasticity:
            self.spasticity_level = np.random.choice([0,1,2,3])  # Random level between 0 and 3
        else:
            self.spasticity_level = kwargs.get('spasticity_level', 0)  # Default: no spasticity
        self.spasticity_type = kwargs.get('spasticity_type', 'hyperreflexia')  # Default: hyperreflexia

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = True,
        options: Optional[dict] = None,
    ):
        obs, _ = super().reset() 
        self.KV_coeffs = self.adjust_KV_gains()
        print(f"Resetting environment with spasticity level {self.spasticity_level} and type {self.spasticity_type}. KV Coefficients: {self.KV_coeffs}")
        return obs, {}
        
    def before_step(self, action):
        #print("[DEBUG] action before spasticity:", action)
        action = super().before_step(action)
        action[:self.num_muscles] = self.apply_hyperreflexia(action[:self.num_muscles])
        action[:self.num_muscles] = self.apply_spastic_trigger(action[:self.num_muscles])
        #print("[DEBUG] action after spasticity:", action)
        
        return action
    
    def adjust_KV_gains(self):
        """ Adjust the spasticity gains based on the spasticity level
            Introducing a Gaussian noise to simulate variability in spasticity response
             Velocity dependent increase: KV * (normalized_muscle_velocity)
            KV interval => [0.0, 0.12] 
            Levels:
            - 0 : No spasticity
            - 1 : Mild spasticity (Ashworth 1) => KV = 0.033 
            - 2 : Moderate spasticity (Ashworth 2) => KV = 0.066 
            - 3 : Severe spasticity (Ashworth 3) => KV = 0.10 
            - 4 : (Not implemented) Very severe spasticity (Ashworth 4)

        """
        vel_spasticity_gains = np.zeros(self.num_muscles)
        if self.spasticity_level == 1:
            base_gain = 0.033 #0.033
        elif self.spasticity_level == 2:
            base_gain = 0.066 #0.066      
        elif self.spasticity_level == 3:
            base_gain = 0.10 #0.10
        else:
            base_gain = 0.0 # No spasticity

        # Type of spasticity can be different across muscles in terms of distal, joint-specific or complete.
        if self.spasticity_type == 'hyperreflexia':
            # Apply hyperreflexia to gastroc and soleus muscles with the same gain
            soleus_mus_idx = [i for i, muscle in enumerate(self.model.muscles()) if "soleus" in muscle.name()]
            gastroc_mus_idx = [i for i, muscle in enumerate(self.model.muscles()) if "gastroc" in muscle.name()]
            spas_idx = soleus_mus_idx + gastroc_mus_idx
            
            # Apply the same gain to all spastic muscles with added noise
            for i in spas_idx:
                noise = np.random.normal(0, 0.20 * base_gain)  # noise of 10% of the base gain
                vel_spasticity_gains[i] = max(0.0, base_gain + noise)  # Ensure non-negative gain

        elif self.spasticity_type == 'distal':
            # Apply spasticity with a coeff of [0.25, 0.5, 1.0] for hip, knee and ankle muscles respectively
            distal_muscle_indices = [i for i, muscle in enumerate(self.model.muscles()) if "distal" in muscle.name().lower()]
            for i in distal_muscle_indices:
                noise = np.random.normal(0, 0.010 * base_gain)  # noise of 10% of the base gain
                vel_spasticity_gains[i] = max(0.0, base_gain + noise)  # Ensure non-negative gain
        
        elif self.spasticity_type == 'joint_specific': #Oftenly ankle plantarflexors
            # Apply spasticity to muscles crossing specific joints (e.g., knee extensors)
            joint_specific_muscle_indices = [i for i, muscle in enumerate(self.model.muscles()) if "knee" in muscle.name().lower()]
            for i in joint_specific_muscle_indices:
                noise = np.random.normal(0, 0.010 * base_gain)  # noise of 10% of the base gain
                vel_spasticity_gains[i] = max(0.0, base_gain + noise)  # Ensure non-negative gain
        
        else: # Complete spasticity: apply to all muscles
            # Apply spasticity to all muscles
             for i in range(self.num_muscles):
                noise = np.random.normal(0, 0.20 * base_gain)  # noise of 10% of the base gain
                vel_spasticity_gains[i] = max(0.0, base_gain + noise)  # Ensure non-negative gain
        
        """
        Angle dependent increase: KA * (normalized_muscle_length)
        """
        return vel_spasticity_gains

    def apply_hyperreflexia(self, action):
        """ Apply hyperreflexia to selected muscles based on their velocity (Lassman et al. 2023)
            New activation = original_activation + KV * normalized_muscle_velocity
        """
        for i, muscle in enumerate(self.model.muscles()):
            muscle_velocities = abs(self.model.muscles()[i].fiber_velocity_norm())
            #print(f"[DEBUG] Muscle: {muscle.name()}, Velocity: {muscle_velocities:.4f}, KV Gain: {self.KV_coeffs[i]:.4f}")
        # Apply spasticity 
        if self.passive_subject: # No baseline activations, only spasticity contributions
            action = self.KV_coeffs * muscle_velocities
        else:  
            action += self.KV_coeffs * muscle_velocities
        
        return action    
    
    def apply_spastic_trigger(self, action):
        """ Apply a spastic trigger to a specific muscle based on its length
            If the normalized muscle length exceeds the threshold, increase activation by added_activation
    
        muscle_lengths = self.model.muscle_fiber_length_array()
        norm_muscle_lengths = muscle_lengths / (np.max(muscle_lengths) + 1e-6)  # Normalize lengths

        for i, muscle in enumerate(self.model.muscles()):
            if muscle.name() == muscle_name:
                if norm_muscle_lengths[i] > threshold:
                    return added_activation
        """
        return action


class WeaknessEnv(NairSconeGymEnv):
    """
    Base class for Weakness Environments
    - Replicate muscle weakness by decreasing the gain of selected muscles
    - Depending on the weakness level, decrease the gain of selected muscles
    
    Inputs: 
    Normalized muscle lengths and lengths velocities
    
    Outputs:
    Muscle activations

    Levels of weakness:
    - 0 : No weakness
    - 1 : Mild weakness 
    - 2 : Moderate weakness 
    - 3 : Severe weakness
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.randomize_weakness = kwargs.get('randomize_weakness', True)
        self.weakness_level = kwargs.get('weakness_level', 0)
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = True,
        options: Optional[dict] = None,
    ):
        obs, _ = super().reset() 
        
        if self.randomize_weakness:
            self.weakness_level = np.random.choice([0,1,2,3])  # Random level between 0 and 3
        
        self.weakness_coeffs = self.adjust_weakness()
        #print(f"Resetting environment with weakness level {self.weakness_level}. Weakness Coefficients: {self.weakness_coeffs}")
        return obs, {}
        
    def before_step(self, action):
        #print("[DEBUG] action before weakness:", action)
        action = super().before_step(action)
        action[:self.num_muscles] = self.apply_weakness(action[:self.num_muscles])
        #print("[DEBUG] action after weakness:", action)
        
        return action
    
    def adjust_weakness(self):
        """ Adjust the weakness gains based on the weakness level
            Introducing a Gaussian noise to simulate variability in weakness response
                Weakness increase: Original activation * (1 - base_gain)
            Levels:
            - 0 : No weakness
            - 1 : Mild weakness => KW = 10% 
            - 2 : Moderate weakness => KW = 25%
            - 3 : Severe weakness => KW = 50% 
        """
        weakness_gains = np.zeros(self.num_muscles)
        if self.weakness_level == 1:
            base_gain = 0.10 #10%
        elif self.weakness_level == 2:
            base_gain = 0.25 #25%      
        elif self.weakness_level == 3:
            base_gain = 0.50 #50%
        else:
            base_gain = 0.0 # No weakness

        # Type of weakness can be different across muscles in terms of distal, joint-specific or complete.
        # TODO: Implement different types of weakness across muscles (e.g., distal, joint-specific or complete)
                 
        # Apply weakness to muscles crossing specific joints (e.g., hip muscles)
        joint_specific_muscle_indices = [i for i, muscle in enumerate(self.model.muscles()) if "iliopsoas" in muscle.name().lower() or "hamstrings" in muscle.name().lower() or "glut_max" in muscle.name().lower() or "rect_fem" in muscle.name().lower() or "glut_med" in muscle.name().lower() or "add_mag" in muscle.name().lower()]
        for i in joint_specific_muscle_indices:
            noise = np.random.normal(0, 0.20 * base_gain)  # noise of 20% of the base gain
            weakness_gains[i] = max(0.0, base_gain + noise)  # Ensure non-negative gain
        #print(f"[DEBUG] Weakness gains for joint-specific muscles: {[weakness_gains[i] for i in joint_specific_muscle_indices]}")
        
        return weakness_gains

    def apply_weakness(self, action):
        """ Apply weakness to selected muscles
            New activation = original_activation * (1 - KW)
        """
        #print(f"[DEBUG] Action before applying weakness: {action[:self.num_muscles]}")
        #print(f"[DEBUG] Weakness coefficients: {self.weakness_coeffs}")
        # Apply weakness   
        action[:self.num_muscles] *= (1 - self.weakness_coeffs)
        #print(f"[DEBUG] Action after applying weakness: {action[:self.num_muscles]}")
        return action    

# ****************************************************************************
#       Definition of specific motor disorder environments 
# ****************************************************************************
  # SpasGaitEnv: Spasticity environment for gait    
class SpasGaitEnv(SpasticityEnv, SconeGaitGym):
    """
    Spasticity environment for gait
     - Replicate Hyperreflexia by increasing the gain of selected muscles
     - These activation are based on Lassman et al. 2023 Frontiers in Neurorobotics and the Ashworth scale  
     - Depending on the spasticity level, increase the gain of selected muscles
        - Inputs: Normalized muscle lengths and lengths velocities
        - Outputs: Muscle activations
    """
    def __init__(self, *args, **kwargs):
        # SpasticityEnv and SconeGaitGym share NairSconeGymEnv as a base
        # Python MRO ensures that NairSconeGymEnv.__init__ is called only once
        super().__init__(*args, **kwargs)
    
    def reset(self, *, seed: Optional[int] = None, return_info: bool = True, options: Optional[dict] = None):
        # Call the reset of SconeGaitGym (which includes GaitGym.reset)
        obs, info = SconeGaitGym.reset(self, seed=seed, return_info=return_info, options=options)
        # Apply the initialization of spasticity
        self.KV_coeffs = self.adjust_KV_gains()
        #print(f"Resetting environment with spasticity level {self.spasticity_level} and type {self.spasticity_type}. KV Coefficients: {self.KV_coeffs}")
        
        return obs, info

class WeakGaitEnv(WeaknessEnv, SconeGaitGym):
    """
    Weakness environment for gait
     - Replicate muscle weakness by decreasing the gain of selected muscles
     - Depending on the weakness level, decrease the gain of selected muscles
        - Inputs: Muscle activations
        - Outputs: Muscle activations with weakness applied
    """
    def __init__(self, *args, **kwargs):
        # WeaknessEnv and SconeGaitGym share NairSconeGymEnv as a base
        # Python MRO ensures that NairSconeGymEnv.__init__ is called only once
        
        super().__init__(*args, **kwargs)
    
    def reset(self, *, seed: Optional[int] = None, return_info: bool = True, options: Optional[dict] = None):
        # Call the reset of SconeGaitGym (which includes GaitGym.reset)
        obs, info = SconeGaitGym.reset(self, seed=seed, return_info=return_info, options=options)
        # Apply the initialization of weakness
        if self.randomize_weakness:
            self.weakness_level = np.random.choice([0,1,2,3])  # Random level between 0 and 3
        self.weakness_coeffs = self.adjust_weakness()
        print(f"Resetting environment with weakness level {self.weakness_level}. Weakness Coefficients: {self.weakness_coeffs}")
        
        return obs, info