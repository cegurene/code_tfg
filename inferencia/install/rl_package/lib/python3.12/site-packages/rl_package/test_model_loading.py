#!/usr/bin/env python3
"""Script de diagnóstico para verificar la carga de modelos RL.

Este script prueba la carga de los tres tipos de modelos:
1. rl_node.py / rl_node_v2.py: SAC con best_model.zip (Stable Baselines 3)
2. rl_node_v3.py: TorchRL con checkpoint_3272704.pt

Uso:
    python3 test_model_loading.py
"""

import torch
import numpy as np
import os
import sys
from pathlib import Path

print("="*80)
print("DIAGNÓSTICO DE CARGA DE MODELOS RL")
print("="*80)

# =============================================================================
# TEST 1: Modelo SAC (Stable Baselines 3) - usado por rl_node.py y rl_node_v2.py
# =============================================================================
print("\n[TEST 1] Cargando modelo SAC (best_model.zip)...")
print("-"*80)

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE_ROOT / 'models'
best_model_path = MODELS_DIR / 'best_model.zip'

try:
    from stable_baselines3 import SAC
    
    # Intentar carga directa
    try:
        model_sac = SAC.load(path=best_model_path, device='cpu')
        print("✓ Modelo SAC cargado CORRECTAMENTE (carga directa)")
    except Exception as e:
        print(f"✗ Error en carga directa: {e}")
        print("  Intentando con custom_objects...")
        
        # Fallback con custom_objects
        custom_objects = {"learning_rate": 3e-4, "lr_schedule": (lambda _: 3e-4)}
        model_sac = SAC.load(path=best_model_path, device='cpu', custom_objects=custom_objects)
        print("✓ Modelo SAC cargado CORRECTAMENTE (con custom_objects)")
    
    # Probar predicción con observaciones dummy
    print("\nProbando predicción con observaciones dummy...")
    dummy_obs = np.random.randn(52).astype(np.float32)  # Asumiendo 52 observaciones
    action, _states = model_sac.predict(torch.tensor(dummy_obs))
    print(f"✓ Predicción SAC exitosa")
    print(f"  Observaciones dummy: {dummy_obs[:5]}... (primeras 5)")
    print(f"  Acción predicha: {action}")
    print(f"  Tipo de acción: {type(action)}")
    if hasattr(action, '__len__') and len(action) > 0:
        print(f"  Valor escalar: {action[0].item() if hasattr(action[0], 'item') else action[0]}")
    
except FileNotFoundError:
    print(f"✗ ERROR: Archivo no encontrado en {best_model_path}")
except Exception as e:
    print(f"✗ ERROR al cargar modelo SAC: {e}")
    import traceback
    traceback.print_exc()

# =============================================================================
# TEST 2: Modelo TorchRL (.pt checkpoint) - usado por rl_node_v3.py
# =============================================================================
print("\n\n[TEST 2] Cargando modelo TorchRL (checkpoint_3272704.pt)...")
print("-"*80)

checkpoint_path = MODELS_DIR / 'checkpoint_3272704.pt'

try:
    # Cargar checkpoint
    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    
    print("✓ Checkpoint cargado exitosamente")
    print(f"  Claves en checkpoint: {list(checkpoint.keys())}")
    
    # Verificar dimensiones
    if 'obs_dim' in checkpoint and 'act_dim' in checkpoint:
        obs_dim = checkpoint['obs_dim']
        act_dim = checkpoint['act_dim']
        print(f"  Dimensiones: obs_dim={obs_dim}, act_dim={act_dim}")
    else:
        print("✗ ERROR: Checkpoint no tiene obs_dim/act_dim")
        obs_dim = None
        act_dim = None
    
    # Verificar state_dict del actor
    if 'actor' in checkpoint:
        actor_keys = list(checkpoint['actor'].keys())
        print(f"  Claves del actor ({len(actor_keys)} total):")
        for key in actor_keys[:10]:  # Primeras 10
            print(f"    - {key}: {checkpoint['actor'][key].shape if hasattr(checkpoint['actor'][key], 'shape') else 'N/A'}")
    else:
        print("✗ ERROR: Checkpoint no tiene 'actor'")
    
    # Intentar reconstruir el modelo
    if obs_dim is not None and act_dim is not None:
        print("\nReconstruyendo arquitectura del modelo...")
        
        try:
            from torchrl.modules import ProbabilisticActor
            from torchrl.modules.distributions.continuous import TanhNormal
            from tensordict.nn import TensorDictModule
            from tensordict import TensorDict
            import torch.nn as nn
            
            # Arquitectura del actor (debe coincidir con nair_agents.py)
            class ActorNetwork(nn.Module):
                def __init__(self, obs_dim, act_dim):
                    super().__init__()
                    self.net = nn.Sequential(
                        nn.Linear(obs_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, act_dim),
                    )
                    self.log_std = nn.Parameter(torch.zeros(act_dim))
                
                def forward(self, obs):
                    mean = self.net(obs)
                    std = torch.exp(self.log_std).expand_as(mean)
                    return mean, std
            
            actor_net = ActorNetwork(obs_dim, act_dim)
            
            policy_module = TensorDictModule(
                actor_net,
                in_keys=["observation"],
                out_keys=["loc", "scale"]
            )
            
            policy = ProbabilisticActor(
                module=policy_module,
                in_keys=["loc", "scale"],
                distribution_class=TanhNormal,
                distribution_kwargs={
                    "low": -1.0,
                    "high": 1.0,
                },
                return_log_prob=False,
            )
            
            # Cargar pesos
            policy.load_state_dict(checkpoint['actor'], strict=True)
            policy.eval()
            
            print("✓ Modelo TorchRL reconstruido CORRECTAMENTE")
            
            # Probar predicción
            print("\nProbando predicción con observaciones dummy...")
            dummy_obs = torch.randn(1, obs_dim, dtype=torch.float32)
            tensordict_obs = TensorDict(
                {"observation": dummy_obs},
                batch_size=[1],
            )
            
            with torch.no_grad():
                action_dict = policy(tensordict_obs)
                action = action_dict["action"].squeeze(0)
            
            print(f"✓ Predicción TorchRL exitosa")
            print(f"  Observaciones dummy: {dummy_obs[0][:5].tolist()}... (primeras 5)")
            print(f"  Acción predicha: {action.tolist()}")
            print(f"  Valor escalar: {action[0].item()}")
            
        except ImportError as e:
            print(f"✗ ERROR: No se pudo importar TorchRL: {e}")
            print("  Instala TorchRL con: pip install torchrl tensordict")
        except Exception as e:
            print(f"✗ ERROR al reconstruir modelo TorchRL: {e}")
            import traceback
            traceback.print_exc()
    
except FileNotFoundError:
    print(f"✗ ERROR: Archivo no encontrado en {checkpoint_path}")
except Exception as e:
    print(f"✗ ERROR al cargar checkpoint TorchRL: {e}")
    import traceback
    traceback.print_exc()

# =============================================================================
# RESUMEN
# =============================================================================
print("\n" + "="*80)
print("RESUMEN DEL DIAGNÓSTICO")
print("="*80)
print("Si ves '✓ Predicción exitosa' en ambos tests, los modelos se cargan bien.")
print("Si el motor se comporta igual con todos, el problema puede ser:")
print("  1. Las observaciones que llegan son siempre las mismas")
print("  2. El limitador de seguridad está siempre activo")
print("  3. El modelo predice valores muy pequeños/similares")
print("\nPróximo paso: Ejecuta este script y comparte el output completo.")
print("="*80)
