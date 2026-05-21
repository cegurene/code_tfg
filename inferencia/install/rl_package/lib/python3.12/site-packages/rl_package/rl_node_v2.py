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
from rl_interfaces.msg import MotionCommand
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE_ROOT / 'models'
best_model_path = MODELS_DIR / 'best_model.zip'
import torch

class MinimalPublisher(Node):


    def __init__(self):
        """Initialize RL agent node with model loading and interactive angle calibration."""
        super().__init__('rl_node')
        
        # Publisher: Torque commands to motor (via Candle or exo_publisher)
        self.publisher_ = self.create_publisher(MotionCommand, '/md80/motion_command', 10)
        
        # Current motor position (updated by joint_state_callback)
        self.current_pos = None
        
        # Subscribe to joint states FIRST (must be active BEFORE calibration prompts)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/md80/joint_states',
            self.joint_state_callback,
            10
        )
        # Model checkpoint from training on 2026-01-25
        best_model_path = os.path.join(log_path, '2026-01-25/19-03-42/outputs/checkpoints/best_model.zip')
        
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
        """Load SAC model with multiple fallback strategies for version compatibility.
        
        Handles common deserialization issues when loading models trained with
        different Python or StableBaselines3 versions.
        
        Returns:
            SAC model or None if loading fails
        """
        try:
            from stable_baselines3 import SAC
            return SAC.load(path=model_path, device='cpu')
        except FileNotFoundError:
            raise RuntimeError(f"Model file not found at {model_path}")
        except Exception as e:
            # FALLBACK: Try with custom_objects to handle lr_schedule deserialization errors
            try:
                from stable_baselines3 import SAC
                custom_objects_attempts = [
                    # Attempt 1: Replace lr_schedule with lambda function
                    {"learning_rate": 3e-4, "lr_schedule": (lambda _: 3e-4)},
                    # Attempt 2: Just override learning_rate
                    {"learning_rate": 3e-4},
                ]
                for custom_objects in custom_objects_attempts:
                    try:
                        return SAC.load(path=model_path, device='cpu', custom_objects=custom_objects)
                    except Exception:
                        continue
            except Exception:
                pass

            # All attempts failed - log error and return None (will use zero torque)
            self.get_logger().error(
                "No se pudo cargar el modelo. Normalmente es incompatibilidad de versiones. "
                f"Python actual: {sys.version.split()[0]}."
            )
            self.get_logger().error(f"Error original: {e}")
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
            action, _states = self.model.predict(torch.tensor(observations))
            # Here we would pass the observations to the RL and obtain the actions
            act = action[0].item()
            
            # DIAGNÓSTICO: Mostrar acción del modelo cada 100 callbacks
            if self._callback_count % 100 == 1:
                self.get_logger().info(
                    f'🤖 Modelo predice: act[0]={act:.4f} (vector completo: {action.shape})'
                )

        # SAFETY: Angle limiter - Apply return torque if motor exceeds configured range
        # This runs AFTER model prediction to override any dangerous commands
        # Use a proportional return torque to actively push the motor back into range
        safety_override = False
        if self.current_pos is not None:
            limit_kp = 10.0           # Nm per rad beyond limit (tune if needed)
            max_return_torque = 25.0   # Absolute torque cap (Nm)

            if self.current_pos < self.angle_min:
                # Motor too low: apply positive torque to push it back up
                error = self.angle_min - self.current_pos
                return_torque = min(max_return_torque, limit_kp * error)
                self.get_logger().warn(
                    f"⚠️  LIMIT: Below min ({self.current_pos:.3f} < {self.angle_min:.3f}). "
                    f"OVERRIDE: {act:.3f} → +{return_torque:.2f} Nm"
                )
                act = return_torque
                safety_override = True
            elif self.current_pos > self.angle_max:
                # Motor too high: apply negative torque to push it back down
                error = self.current_pos - self.angle_max
                return_torque = -min(max_return_torque, limit_kp * error)
                self.get_logger().warn(
                    f"⚠️  LIMIT: Above max ({self.current_pos:.3f} > {self.angle_max:.3f}). "
                    f"OVERRIDE: {act:.3f} → {return_torque:.2f} Nm"
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
        msg_pub = MotionCommand()   # We create a MotionCommand message
        msg_pub.drive_ids = [self.motor_id]  # Use the motor ID from user input
        msg_pub.target_torque = [act]   # We introduce the torque obtained from the RL into the message
        # Publish the message
        self.publisher_.publish(msg_pub)       
        
        # Log detallado cada 100 callbacks, siempre si hay override de seguridad
        if self._callback_count % 100 == 1 or safety_override:
            status = "⚠️ SAFETY" if safety_override else "✓ MODEL"
            self.get_logger().info(
                f'{status} | Torque={act:.3f} Nm | Pos={self.current_pos:.3f} rad'
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
