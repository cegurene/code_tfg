"""RL_NODE_V4 - Reinforcement Learning Agent WITHOUT Safety Limiter

ADVERTENCIA: Esta versión NO tiene limitador de seguridad por ángulos.
Usar SOLO para diagnóstico y verificar que el modelo controla el motor.
NO usar en producción sin supervisión constante.

Diferencias con v3:
  - SIN limitador de seguridad por ángulos
  - SIN calibración interactiva (no necesaria sin límites)
  - Logging detallado de predicciones del modelo
  - Publicación directa de torque del modelo al motor
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rl_interfaces.msg import MotionCommand as SimMotionCommand
try:
    from candle_ros2.msg import MotionCommand as CandleMotionCommand
except Exception:
    CandleMotionCommand = None

import numpy
import os
import sys
from pathlib import Path

import torch

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE_ROOT / 'models'
best_model_path = MODELS_DIR / 'checkpoint_3272704.pt'

class MinimalPublisher(Node):

    def __init__(self):
        """Initialize RL agent node WITHOUT safety limiter."""
        super().__init__('rl_node_v4')
        
        # Publishers: hardware (Candle) and simulator (MyoSuite/exo_publisher)
        self.hardware_publisher_ = None
        if CandleMotionCommand is not None:
            self.hardware_publisher_ = self.create_publisher(
                CandleMotionCommand, '/md80/motion_command', 10
            )

        sim_topic = '/rl/motion_command' if CandleMotionCommand is not None else '/md80/motion_command'
        self.sim_publisher_ = self.create_publisher(SimMotionCommand, sim_topic, 10)
        
        # Current motor state (updated by joint_state_callback)
        self.current_pos = None
        self.current_vel = None
        
        # Subscribe to joint states for monitoring only
        self.joint_subscription = self.create_subscription(
            JointState,
            '/md80/joint_states',
            self.joint_state_callback,
            10
        )
        self.joint_subscription
        
        # Motor ID input
        self.motor_id = int(input("Enter the motor ID to control: "))
        
        # Wait for first joint state message to verify motor connectivity
        print("\n⏳ Waiting for joint state data from motor...")
        wait_time = 0.0
        max_wait = 5.0
        timeout_per_spin = 0.2
        
        while self.current_pos is None and wait_time < max_wait:
            rclpy.spin_once(self, timeout_sec=timeout_per_spin)
            wait_time += timeout_per_spin
        
        if self.current_pos is None:
            self.get_logger().error("ERROR: No joint state received after 5 seconds.")
            print("\n⚠️ Motor not responding. Make sure Candle node is running and motor is enabled.")
            raise RuntimeError("Cannot connect to motor")
        
        print(f"✓ Motor connected. Initial position: {self.current_pos:.4f} rad\n")
        
        print("="*70)
        print("⚠️  WARNING: SAFETY LIMITER DISABLED")
        print("="*70)
        print("This version sends model predictions DIRECTLY to the motor.")
        print("Monitor the motor constantly and be ready to stop it manually.")
        print("Motor ID: {}".format(self.motor_id))
        print("="*70 + "\n")
        
        # Subscribe to observations
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/rl/observations',
            self.listener_callback,
            10)
        self.subscription

        self.get_logger().info('rl_node_v4 iniciado (SIN SAFETY LIMITER)')
        self._warned_no_model = False
        self.model = self._load_model(best_model_path)
        if self.model is None:
            self.get_logger().warn('Modelo no cargado. Se enviará torque 0.0.')
        else:
            self.get_logger().info('Modelo cargado correctamente.')

    def _load_model(self, model_path):
        """Load TorchRL PPO model from checkpoint (.pt file)."""
        try:
            checkpoint = torch.load(model_path, map_location='cpu')
            
            if 'obs_dim' not in checkpoint or 'act_dim' not in checkpoint:
                raise RuntimeError("Checkpoint sin obs_dim/act_dim. Usa un checkpoint más reciente.")
            
            obs_dim = checkpoint['obs_dim']
            act_dim = checkpoint['act_dim']
            
            self.get_logger().info(
                f"Cargando modelo: obs_dim={obs_dim}, act_dim={act_dim}"
            )
            
            from torchrl.modules import ProbabilisticActor
            from torchrl.modules.distributions.continuous import TanhNormal
            from tensordict.nn import TensorDictModule
            import torch.nn as nn
            
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
            
            policy.load_state_dict(checkpoint['actor'], strict=True)
            policy.eval()
            
            self.get_logger().info(
                f"✓ Modelo cargado exitosamente desde paso {checkpoint.get('step', '?')}"
            )
            
            return policy
            
        except FileNotFoundError:
            raise RuntimeError(f"Archivo de modelo no encontrado: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Error cargando modelo TorchRL: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def listener_callback(self, msg):
        """Process observations and publish model predictions WITHOUT safety override."""
        observations = msg.data
        
        # DIAGNÓSTICO: Mostrar observaciones cada 50 callbacks
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        self._callback_count += 1
        
        if self._callback_count % 50 == 1:
            self.get_logger().info(
                f'📥 Obs recibidas ({len(observations)}): [{observations[0]:.3f}, {observations[1]:.3f}, {observations[2]:.3f}, ...]'
            )

        if self.model is None:
            act = 0.0
            if not self._warned_no_model:
                self.get_logger().warn(
                    "Model not loaded. Publishing zero torque for testing."
                )
                self._warned_no_model = True
        else:
            # Convertir observaciones a TensorDict
            from tensordict import TensorDict
            obs_tensor = torch.tensor(observations, dtype=torch.float32).unsqueeze(0)
            
            tensordict_obs = TensorDict(
                {"observation": obs_tensor},
                batch_size=[1],
            )
            
            # Inferencia con el modelo TorchRL
            with torch.no_grad():
                action_dict = self.model(tensordict_obs)
                action_tensor = action_dict["action"].squeeze(0)
            
            # Extraer el valor escalar del tensor
            act = action_tensor[0].item() if action_tensor.numel() > 0 else 0.0
            
            # DIAGNÓSTICO: Mostrar acción cada 50 callbacks
            if self._callback_count % 50 == 1:
                self.get_logger().info(
                    f'🤖 TorchRL predice: act[0]={act:.4f} Nm (rango: [{action_tensor.min():.3f}, {action_tensor.max():.3f}])'
                )

        # NO HAY SAFETY LIMITER - Publicar directamente la acción del modelo
        
        # Publicar a hardware (Candle)
        if self.hardware_publisher_ is not None:
            msg_hw = CandleMotionCommand()
            msg_hw.drive_ids = [self.motor_id]
            msg_hw.target_position = [0.0]
            msg_hw.target_velocity = [0.0]
            msg_hw.target_torque = [act]
            self.hardware_publisher_.publish(msg_hw)

        # Publicar a simulación
        msg_sim = SimMotionCommand()
        msg_sim.drive_ids = [self.motor_id]
        msg_sim.target_position = [0.0]
        msg_sim.target_velocity = [0.0]
        msg_sim.target_torque = [act]
        self.sim_publisher_.publish(msg_sim)

        # Log detallado cada 50 callbacks
        if self._callback_count % 50 == 1:
            vel = self.current_vel if self.current_vel is not None else 0.0
            pos = self.current_pos if self.current_pos is not None else 0.0
            self.get_logger().info(
                f'🎯 DIRECT MODEL | Torque={act:.3f} Nm | Pos={pos:.3f} rad | Vel={vel:.3f} rad/s'
            )

    def joint_state_callback(self, msg):
        """Track current motor state for monitoring only."""
        if not msg.name:
            return
        
        target_name = f"Joint {self.motor_id}"
        for i, name in enumerate(msg.name):
            if name == target_name:
                if i < len(msg.position):
                    self.current_pos = msg.position[i]
                if i < len(msg.velocity):
                    self.current_vel = msg.velocity[i]
                return


def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
