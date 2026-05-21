"""RL_NODE - Reinforcement Learning Agent for Motor Control

This node loads a pre-trained SAC model and performs real-time inference.
It receives observations from aggregator and publishes torque commands to the motor.

Key Features:
  - SAC model loading with error handling and fallbacks
  - Angle safety limiter: Blocks torque if joint exceeds limits
  - Joint state monitoring for real-time position tracking
  - Graceful degradation: Falls back to zero torque if model fails to load

Changes History:
  - Added angle safety limiter: Monitors /md80/joint_states and blocks torque
  - Improved model loading: Multiple custom_objects fallbacks for version compatibility
  - Added joint_state_callback: Tracks current motor position in real-time
  - Input validation: Swaps angle_min/angle_max if user enters them reversed
  - Graceful fallback: Publishes zero torque if model cannot be loaded
  - Error logging: Spanish error messages for deserialization failures
"""

import rclpy
from rclpy.node import Node

from pathlib import Path
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

import torch

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE_ROOT / 'models'
best_model_path = MODELS_DIR / 'checkpoint_3272704.pt'

class MinimalPublisher(Node):


    def __init__(self):
        """Initialize RL agent node with model loading and interactive angle calibration."""
        super().__init__('rl_node')
        
        # Publishers: hardware (Candle) and simulator (MyoSuite/exo_publisher)
        # Hardware expects candle_ros2/msg/MotionCommand on /md80/motion_command
        self.hardware_publisher_ = None
        if CandleMotionCommand is not None:
            self.hardware_publisher_ = self.create_publisher(
                CandleMotionCommand, '/md80/motion_command', 10
            )

        # Simulator topic uses rl_interfaces/msg/MotionCommand
        sim_topic = '/rl/motion_command' if CandleMotionCommand is not None else '/md80/motion_command'
        self.sim_publisher_ = self.create_publisher(SimMotionCommand, sim_topic, 10)
        
        # Current motor state (updated by joint_state_callback)
        self.current_pos = None
        self.current_vel = None
        
        # Subscribe to joint states FIRST (must be active BEFORE calibration prompts)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/md80/joint_states',
            self.joint_state_callback,
            10
        )
        self.joint_subscription
        
        # Motor ID input
        self.motor_id = int(input("Enter the motor ID to control: "))
        
        # Wait for first joint state message to ensure motor connectivity
        # Use spin_once() to allow callbacks to execute while waiting
        print("\n⏳ Waiting for joint state data from motor...")
        wait_time = 0.0
        max_wait = 5.0  # Wait up to 5 seconds
        timeout_per_spin = 0.2  # 200ms timeout per spin
        
        while self.current_pos is None and wait_time < max_wait:
            rclpy.spin_once(self, timeout_sec=timeout_per_spin)
            wait_time += timeout_per_spin
        
        if self.current_pos is None:
            self.get_logger().error("ERROR: No joint state received after 5 seconds.")
            print("\n⚠️ CALIBRATION FAILED: Motor not responding.")
            print("Make sure:")
            print("  1. Candle ROS2 node is running:")
            print("     ros2 run candle_ros2 candle_ros2_node USB 1M")
            print("  2. Motor is enabled:")
            print("     ros2 service call /candle_ros2_node/enable_md80s '{}'")
            print("  3. Motor ID matches (currently set to: {})".format(self.motor_id))
            print("  4. Motor is not damaged or disconnected\n")
            raise RuntimeError("Cannot calibrate: no joint state data from motor")
        
        print(f"✓ Motor connected. Initial position: {self.current_pos:.4f} rad\n")
        
        # Interactive angle limit calibration
        self._calibrate_angle_limits()
        
        # Subscribe to observations
        self.subscription = self.create_subscription(
            Float32MultiArray,          # Message type used by the publisher
            '/rl/observations',         # topic name
            self.listener_callback,     # Method called each time a message is received
            10)                         # Queue size for incoming messages
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('rl_node iniciado')
        self._warned_no_model = False
        self.model = self._load_model(best_model_path)
        if self.model is None:
            self.get_logger().warn('Modelo no cargado. Se enviará torque 0.0.')
        else:
            self.get_logger().info('Modelo cargado correctamente.')

    def _calibrate_angle_limits(self):
        """Interactively calibrate joint angle limits by physical positioning.
        
        Process:
        1. User moves motor to MINIMUM position and presses ENTER
        2. Captures angle_min = current motor position
        3. User moves motor to MAXIMUM position and presses ENTER
        4. Captures angle_max = current motor position
        5. Auto-corrects if limits entered in reverse order
        6. Displays calibration summary
        
        This approach is much more precise than manual radian input.
        """
        import math
        import threading
        
        print("\n" + "="*70)
        print("INTERACTIVE ANGLE LIMITER CALIBRATION")
        print("="*70)
        
        # Step 1: Calibrate minimum angle
        print("\n📍 STEP 1: MINIMUM ANGLE LIMIT")
        print("-" * 70)
        print("1. Manually move the motor to the MINIMUM safe position")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at minimum position\n")
        
        self._wait_for_user_input_and_capture("min")
        print(f"✓ Minimum angle captured: {self.angle_min:.4f} rad ({math.degrees(self.angle_min):.2f}°)")
        
        # Step 2: Calibrate maximum angle
        print("\n📍 STEP 2: MAXIMUM ANGLE LIMIT")
        print("-" * 70)
        print("1. Manually move the motor to the MAXIMUM safe position")
        print("2. Current position will be shown below")
        print("3. Press ENTER when motor is at maximum position\n")
        
        self._wait_for_user_input_and_capture("max")
        print(f"✓ Maximum angle captured: {self.angle_max:.4f} rad ({math.degrees(self.angle_max):.2f}°)")
        
        # Auto-correct if values were swapped
        if self.angle_min > self.angle_max:
            self.get_logger().warn("Values were swapped. Auto-correcting...")
            self.angle_min, self.angle_max = self.angle_max, self.angle_min
        
        # Display calibration summary
        angle_range = self.angle_max - self.angle_min
        print("\n" + "="*70)
        print("✓ CALIBRATION COMPLETE")
        print("="*70)
        print(f"Minimum angle (angle_min): {self.angle_min:.4f} rad ({math.degrees(self.angle_min):.2f}°)")
        print(f"Maximum angle (angle_max): {self.angle_max:.4f} rad ({math.degrees(self.angle_max):.2f}°)")
        print(f"Safe range:                {angle_range:.4f} rad ({math.degrees(angle_range):.2f}°)")
        print(f"Motor ID:                  {self.motor_id}")
        print("="*70)
        print("Torque will be blocked if motor exceeds these limits.\n")
        
        self.get_logger().info(
            f"Angle limits calibrated - min: {self.angle_min:.4f} rad, max: {self.angle_max:.4f} rad"
        )

    def _wait_for_user_input_and_capture(self, limit_type):
        """Wait for user to press ENTER while continuously updating position display.
        
        Uses threading to allow ROS2 callbacks to execute while waiting for input.
        
        Args:
            limit_type (str): "min" or "max" - which angle limit to capture
        """
        import math
        import threading
        import sys
        
        # Flag to signal when user has pressed ENTER
        user_ready = threading.Event()
        
        def wait_for_input():
            """Run in separate thread to wait for user input without blocking ROS2."""
            input()  # Block until user presses ENTER
            user_ready.set()
        
        # Start input thread
        input_thread = threading.Thread(target=wait_for_input, daemon=True)
        input_thread.start()
        
        # Keep spinning ROS2 and updating display until user presses ENTER
        while not user_ready.is_set():
            # Spin ROS2 to allow callbacks to execute (100ms timeout)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Update position display
            if self.current_pos is not None:
                print(f"\rCurrent position: {self.current_pos:.4f} rad ({math.degrees(self.current_pos):.2f}°)  ", end="", flush=True)
        
        # Capture the final position
        if limit_type == "min":
            self.angle_min = self.current_pos
        elif limit_type == "max":
            self.angle_max = self.current_pos
        
        # Wait for thread to finish
        input_thread.join(timeout=1.0)

    def _load_model(self, model_path):
        """Load TorchRL PPO model from checkpoint (.pt file).
        
        Reconstructs the actor network architecture matching training code
        and loads the saved state_dict.
        
        Returns:
            ProbabilisticActor or None if loading fails
        """
        try:
            # Cargar checkpoint completo
            checkpoint = torch.load(model_path, map_location='cpu')
            
            # Verificar que tenga las dimensiones
            if 'obs_dim' not in checkpoint or 'act_dim' not in checkpoint:
                raise RuntimeError("Checkpoint sin obs_dim/act_dim. Usa un checkpoint más reciente.")
            
            obs_dim = checkpoint['obs_dim']
            act_dim = checkpoint['act_dim']
            
            self.get_logger().info(
                f"Cargando modelo: obs_dim={obs_dim}, act_dim={act_dim}"
            )
            
            # Imprimir las claves del state_dict para debug
            actor_keys = list(checkpoint['actor'].keys())
            self.get_logger().info(f"Claves del actor: {actor_keys[:5]}...")  # Primeras 5 claves
            
            # La arquitectura guardada tiene module.0.module.net y module.0.module.log_std
            # Esto significa que usa una arquitectura con log_std aprendible
            # Necesitamos reconstruir EXACTAMENTE esa estructura
            
            from torchrl.modules import ProbabilisticActor
            from torchrl.modules.distributions.continuous import TanhNormal
            from tensordict.nn import TensorDictModule
            import torch.nn as nn
            
            # Crear una clase de actor personalizada que coincida con nair_agents.py
            class ActorNetwork(nn.Module):
                def __init__(self, obs_dim, act_dim):
                    super().__init__()
                    # Red neuronal principal (coincide con checkpoint: 52->64->41)
                    self.net = nn.Sequential(
                        nn.Linear(obs_dim, 64),
                        nn.Tanh(),
                        nn.Linear(64, act_dim),
                    )
                    # log_std aprendible (parámetro separado)
                    self.log_std = nn.Parameter(torch.zeros(act_dim))
                
                def forward(self, obs):
                    # Salida: media (loc) y desviación estándar (scale)
                    mean = self.net(obs)
                    std = torch.exp(self.log_std).expand_as(mean)
                    return mean, std
            
            actor_net = ActorNetwork(obs_dim, act_dim)
            
            # Envolver en TensorDictModule
            policy_module = TensorDictModule(
                actor_net,
                in_keys=["observation"],
                out_keys=["loc", "scale"]
            )
            
            # Crear ProbabilisticActor
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
            
            # Cargar los pesos entrenados (state_dict)
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
        # Each time we receive the observations, we must pass it to the RL to obtain the actions, and then we must publish the actions
        observations = msg.data     # The observations are a float array found in msg.data
        
        # DIAGNÓSTICO: Mostrar primeras observaciones cada 100 callbacks
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        self._callback_count += 1
        if self._callback_count % 100 == 1:
            self.get_logger().info(
                f'📥 Obs recibidas ({len(observations)}): [{observations[0]:.3f}, {observations[1]:.3f}, {observations[2]:.3f}, ...]'
            )

        if self.model is None:
            # Fallback for pipeline testing when model cannot be loaded
            act = 0.0
            if not self._warned_no_model:
                self.get_logger().warn(
                    "Model not loaded. Publishing zero torque for testing."
                )
                self._warned_no_model = True
        else:
            # Convertir observaciones a TensorDict (formato esperado por TorchRL)
            from tensordict import TensorDict
            obs_tensor = torch.tensor(observations, dtype=torch.float32).unsqueeze(0)  # Batch dimension
            
            # Crear TensorDict con la clave "observation"
            tensordict_obs = TensorDict(
                {"observation": obs_tensor},
                batch_size=[1],
            )
            
            # Inferencia con el modelo TorchRL
            with torch.no_grad():
                action_dict = self.model(tensordict_obs)
                # El modelo devuelve un TensorDict con la clave "action"
                action_tensor = action_dict["action"].squeeze(0)  # Remover batch dimension
            
            # Extraer el valor escalar del tensor
            act = action_tensor[0].item() if action_tensor.numel() > 0 else 0.0
            
            # DIAGNÓSTICO: Mostrar acción del modelo cada 100 callbacks
            if self._callback_count % 100 == 1:
                self.get_logger().info(
                    f'🤖 TorchRL predice: act[0]={act:.4f} (vector completo: {action_tensor.shape})'
                )

        # SAFETY: Angle limiter - Apply return torque if motor exceeds configured range
        # This runs AFTER model prediction to override any dangerous commands
        # Use a proportional return torque to actively push the motor back into range
        safety_override = False
        if self.current_pos is not None:
            # Safety controller gains (tune if needed)
            limit_kp = 20.0           # Nm per rad beyond limit
            limit_kd = 0.5            # Nm per (rad/s) for damping
            max_return_torque = 25.0  # Absolute torque cap (Nm)

            vel = self.current_vel if self.current_vel is not None else 0.0

            if self.current_pos < self.angle_min:
                # Motor too low: apply positive torque to push it back up
                error = self.angle_min - self.current_pos
                return_torque = (limit_kp * error) - (limit_kd * vel)
                return_torque = min(max_return_torque, max(0.0, return_torque))
                self.get_logger().warn(
                    f"⚠️  LIMIT: Below min ({self.current_pos:.3f} < {self.angle_min:.3f}). "
                    f"OVERRIDE: {act:.3f} → +{return_torque:.2f} Nm (PD: kp*{error:.3f} - kd*{vel:.3f})"
                )
                act = return_torque
                safety_override = True
            elif self.current_pos > self.angle_max:
                # Motor too high: apply negative torque to push it back down
                error = self.current_pos - self.angle_max
                return_torque = -((limit_kp * error) + (limit_kd * vel))
                return_torque = max(-max_return_torque, min(0.0, return_torque))
                self.get_logger().warn(
                    f"⚠️  LIMIT: Above max ({self.current_pos:.3f} > {self.angle_max:.3f}). "
                    f"OVERRIDE: {act:.3f} → {return_torque:.2f} Nm (PD: kp*{error:.3f} + kd*{vel:.3f})"
                )
                act = return_torque
                safety_override = True
            else:
                # Within safe range - model action is used
                if self._callback_count % 100 == 1:
                    self.get_logger().info(
                        f'✓ Zona segura: pos={self.current_pos:.3f} rad, usando acción del modelo'
                    )

        # MotionCommand has uint32[] drive_ids, float32[] target_position, float32[] target_velocity, float32[] target_torque
        # Some drivers require all arrays to have the same length
        if self.hardware_publisher_ is not None:
            msg_hw = CandleMotionCommand()
            msg_hw.drive_ids = [self.motor_id]
            msg_hw.target_position = [0.0]
            msg_hw.target_velocity = [0.0]
            msg_hw.target_torque = [act]
            self.hardware_publisher_.publish(msg_hw)

        msg_sim = SimMotionCommand()
        msg_sim.drive_ids = [self.motor_id]
        msg_sim.target_position = [0.0]
        msg_sim.target_velocity = [0.0]
        msg_sim.target_torque = [act]
        self.sim_publisher_.publish(msg_sim)

        # Log detallado cada 100 callbacks, siempre si hay override de seguridad
        if self._callback_count % 100 == 1 or safety_override:
            status = "⚠️ SAFETY" if safety_override else "✓ MODEL"
            vel = self.current_vel if self.current_vel is not None else 0.0
            self.get_logger().info(
                f'{status} | Torque={act:.3f} Nm | Pos={self.current_pos:.3f} rad | Vel={vel:.3f} rad/s'
            )

    def joint_state_callback(self, msg):
        """Callback to track current motor position for angle safety limiter.
        
        Runs independently of main inference loop to ensure real-time safety.
        Filters by motor ID to handle multi-motor configurations.
        
        Args:
            msg (JointState): Motor state from Candle or exo_publisher
        """
        if not msg.name:
            return
        
        # Find our target motor in the joint state array
        target_name = f"Joint {self.motor_id}"
        for i, name in enumerate(msg.name):
            if name == target_name:
                if i < len(msg.position):
                    self.current_pos = msg.position[i]  # Update tracked position
                if i < len(msg.velocity):
                    self.current_vel = msg.velocity[i]
                return


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        # Destroy the node and shutdown the ROS2 Python client library
        minimal_publisher.destroy_node()
        rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
