"""MYOSUITE_PUBLISHER NODE - Biomechanical Simulation Environment

This node runs the MyoSuite biomechanical simulator and publishes observations to RL.
It simulates a full musculoskeletal system with 40 muscles + 1 exoskeleton actuator.

Environment: M1640KneeExoSim2Real-v0
  - 41 actuators: 1 exoskeleton motor + 40 biological muscles
  - 91 observations: joint positions, velocities, accelerations, muscle activations
  - Trained for knee exoskeleton assistance during walking

Key Features:
  - Environment fallback: Tries M1640KneeExoSim2Real-v0, falls back to any nair env
  - Motor ID filtering: Only responds to commands for configured motor
  - Muscle damping: Applies vmax to 40 muscles for stability
  - Action vector construction: Index 0 = exo torque, indices 1-40 = muscle activations (zeros)

Changes History:
  - Added environment fallback logic for different MyoSuite versions
  - Added motor ID filtering to support multi-motor setups
  - Improved error handling for environment loading
  - Added logging for environment selection and step execution
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from rl_interfaces.msg import MotionCommand

import numpy
import sys
import os
import threading
import time
import myosuite
from pathlib import Path
import myosuite.envs

LOCAL_NAIR_ENV_DIR = Path(__file__).resolve().parents[3] / "myosuite_local" / "myosuite" / "envs"
if str(LOCAL_NAIR_ENV_DIR) not in myosuite.envs.__path__:
    myosuite.envs.__path__.insert(0, str(LOCAL_NAIR_ENV_DIR))
import myosuite.envs.nair  # IMPORTANT: This registers the NAIR custom environments
from myosuite.utils import gym


class MinimalPublisher(Node):
    #env = gym.make('M1640KneeExoSim2Real-v0')

    def __init__(self):
        # We create the MyoSuite environment, the subscriber to listen to the actions and the publisher to send the observations
        super().__init__('myosuite')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/env/myosuite_obs', 10)

        self.motor_id = int(input("Enter the motor ID to control: "))  # Get motor ID from user input

        # Optional muscle baseline for sim2real diagnostics.
        # Training policy outputs 41 actions (1 exo + 40 muscles), but in deployment
        # we often keep muscles at zero. This baseline allows quick testing of whether
        # a non-zero muscle background improves policy responsiveness.
        baseline_raw = os.getenv("RL_MUSCLE_BASELINE", "0.0")
        try:
            self.muscle_baseline = float(baseline_raw)
        except ValueError:
            self.muscle_baseline = 0.0
            self.get_logger().warn(f"Invalid RL_MUSCLE_BASELINE='{baseline_raw}', using 0.0")
        self.muscle_baseline = float(numpy.clip(self.muscle_baseline, 0.0, 1.0))
        self.get_logger().info(f"Muscle baseline activation: {self.muscle_baseline:.3f}")

        # Environment loading with fallback strategy
        # PRIMARY: M1640KneeExoSim2Real-v0 (knee exoskeleton for walking assistance)
        # FALLBACK: Any registered 'nair' environment
        render_mode = os.getenv("RL_RENDER_MODE", "human").strip()
        if render_mode.lower() in ("", "false", "0", "none"):
            render_mode = None
        
        try:
            # Try with render_mode first
            self.env = gym.make(id='M1640KneeExoSim2Real-v0', render_mode=render_mode)
            self.get_logger().info(f'M1640KneeExoSim2Real-v0 environment loaded successfully (render_mode={render_mode}).')
        except TypeError:
            # Fallback: Create without render_mode if environment doesn't support it
            self.get_logger().warn(f"Environment doesn't support render_mode='{render_mode}', creating without it.")
            self.env = gym.make(id='M1640KneeExoSim2Real-v0')
            self.get_logger().info('M1640KneeExoSim2Real-v0 environment loaded (render_mode not supported).')
        except (gym.error.NameNotFound, gym.error.DeprecatedEnv, Exception) as e:
            # If primary environment not found, search for alternatives
            try:
                # Search gymnasium registry for any NAIR environment
                nair_envs = [env_id for env_id in gym.envs.registry.keys() if env_id.startswith("nair")]
                if nair_envs:
                    env_id = nair_envs[0]
                    try:
                        self.env = gym.make(env_id, render_mode=render_mode)
                        self.get_logger().info(f"{env_id} environment loaded (render_mode={render_mode})")
                    except TypeError:
                        self.env = gym.make(env_id)
                        self.get_logger().info(f"{env_id} environment loaded (render_mode not supported)")
                else:
                    raise RuntimeError("No nair environment found in registry")
            except Exception as e2:
                raise RuntimeError(f"Failed to load any MyoSuite environment. Error: {str(e2)}")
        
        obs = self.env.reset()
        self.get_logger().info('MyoSuite environment created and reset.')

        # Optional: Enable rendering visualization
        self.viewer_enabled = os.getenv("RL_VISUALIZE_SIM", "1").strip().lower() in ("1", "true", "yes")
        self.render_timer = None
        self._render_fail_count = 0
        self._max_render_failures = 20
        if self.viewer_enabled:
            try:
                # Gymnasium environments (MyoSuite inherits from them) support rendering
                # Keep a steady render loop so the window stays open.
                self.render_timer = self.create_timer(1.0 / 30.0, self._render_timer_callback)
                self.get_logger().info('✓ Rendering enabled. Visualization window should open in a moment...')
            except Exception as e:
                self.get_logger().warn(f'Could not enable rendering: {e}')
                self.viewer_enabled = False
        else:
            self.get_logger().info('Visualization disabled (RL_VISUALIZE_SIM=0)')

        # Publish initial observation
        msg_pub = Float32MultiArray()
        msg_pub.data = obs[0].tolist()
        self.publisher_.publish(msg_pub)

        # Subscribe to BOTH possible command topics for compatibility:
        # - /rl/motion_command: simulation command topic used by rl_node_v3/v4/v5
        # - /md80/motion_command: legacy/single-topic setups
        self.subscription_rl = self.create_subscription(
            MotionCommand,
            '/rl/motion_command',
            self.listener_callback,
            10)
        self.subscription_rl

        self.subscription_md80 = self.create_subscription(
            MotionCommand,
            '/md80/motion_command',
            self.listener_callback,
            10)
        self.subscription_md80

    def _render_timer_callback(self):
        """Render the simulator at a fixed rate to keep the viewer window alive."""
        if not self.viewer_enabled:
            return

        try:
            self.env.mj_render()
            self._render_fail_count = 0
        except Exception:
            self._render_fail_count += 1
            if self._render_fail_count >= self._max_render_failures:
                self.viewer_enabled = False
                self.get_logger().warn(
                    'Rendering disabled after repeated failures (window closed or backend unavailable).'
                )


    def listener_callback(self, msg):
        # We get the action, perform a step in the MyoSuite environment and publish the obtained observations
        # The action will have 41 floats and the observations will have 91 floats, probably as a numpy array

        # Log the received message to the console
        #self.get_logger().info(f'Torque received: "{msg.target_torque}"')

        # Filter: Only process commands for our configured motor ID
        if(msg.drive_ids[0] == self.motor_id):
            #self.get_logger().info(f'Receiving message for motor with ID: {self.motor_id}')

            # Construct action vector for MyoSuite environment
            # Action space: [exo_torque, muscle_1, muscle_2, ..., muscle_40]
            # Total: 41 actuators (1 exoskeleton + 40 muscles)
            action = numpy.zeros(41)
            action[0] = msg.target_torque[0]  # Exoskeleton torque from RL agent
            action[1:41] = self.muscle_baseline  # Optional diagnostic baseline

            # Execute simulation step
            obs = self.env.step(action)
            #print(self.env.action_space.sample())
            #print(type(self.env.action_space.sample()))
            #self.get_logger().info('Step performed in MyoSuite environment.')
            #print(obs)
            #print(type(obs))
            #print(len(obs))
            #print(type(obs[0]))
            #print(len(obs[0]))
            # array = numpy.array([0.0,msg.target_torque[0],2.0,3.0,4.0,5.0])
            array = obs[0]
            msg_pub = Float32MultiArray() # The published message is an array of floats
            msg_pub.data = array.tolist() # We convert the numpy array into a Python list and save it into the message
            #msg_pub.data = obs[0].tolist()
            self.publisher_.publish(msg_pub)
            #self.get_logger().info('Publishing MyoSuite observations: "%s"' % msg_pub.data)
            



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher() # We create the node
    shutdown_called = False

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        try:
            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            minimal_publisher.env.close()  # Close the MyoSuite environment
            minimal_publisher.get_logger().info('MyoSuite environment closed.')
            minimal_publisher.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        
        # Only shutdown if not already shutdown
        if not shutdown_called:
            try:
                rclpy.shutdown()
            except RuntimeError:
                # Already shutdown, that's okay
                pass

# Entry point of the script
if __name__ == '__main__':
    main()
