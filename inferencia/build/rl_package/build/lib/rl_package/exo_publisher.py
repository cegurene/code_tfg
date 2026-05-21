"""EXO_PUBLISHER NODE - Mock Motor Interface for Testing

This is a PLACEHOLDER/MOCK node that simulates MD80 motor responses.
It does NOT communicate with real hardware - only for testing simulator-only mode.

Purpose:
  - Receives torque commands from rl_node
  - Publishes fake joint states (hardcoded position) to aggregator
  - Allows testing ROS2 architecture without physical motor

For REAL motor control, use Candle ROS2 node instead:
  ros2 run candle_ros2 candle_ros2_node USB 1M

Changes History:
  - Original implementation: Hardcoded joint position response
  - TODO: Replace with actual MD80 driver for real hardware deployment
"""

import rclpy
from rclpy.node import Node

from rl_interfaces.msg import MotionCommand
from sensor_msgs.msg import JointState



class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the node with the custom name 'exo_publisher'
        # We create a publisher to send the motor joint state and a subscriber to listener for the motion_commands from the RL node
        super().__init__('exo_publisher')
        
        # JointState has a string[] name, double[] position, double[] velocity and double[] effort
        self.publisherJoint_ = self.create_publisher(JointState, '/md80/joint_states', 10) 
        
        self.subscription = self.create_subscription(
          MotionCommand,              # Message type used by the publisher
          '/rl/motion_command',       # topic name (sim-only commands)
          self.listener_callback,     # Method called each time a message is received
          10)                         # Queue size for incoming messages
        self.subscription  # Prevent unused variable warning
        
        self.msgMd80ExoPos = [0]
        self.msgMd80ExoVel = [0]
        self.msgMd80ExoAcc = [0]
        self.msgMd80ExoEffort = [0]

    def listener_callback(self, msg):
        """Callback for motion commands from rl_node.
        
        MOCK BEHAVIOR: Ignores received torque and publishes hardcoded position.
        In real implementation, this should:
          1. Send torque command to MD80 motor controller
          2. Read actual encoder position/velocity from motor
          3. Publish real joint state feedback
        
        Args:
            msg (MotionCommand): Contains drive_ids[], target_torque[]
        """
        self.get_logger().info(f'Received torque: "{msg.target_torque}"')
        
        # MOCK: Create fake joint state (real motor would read from encoder)
        msgJoint = JointState()
        msgJoint.position = [10.0, 0.0]  # Hardcoded fake position [rad]
        msgJoint.name = ["Joint 308"]     # Motor ID must match aggregator filter
        
        self.publisherJoint_.publish(msgJoint)
        #self.get_logger().info(f'Publishing position: "{msgJoint.position}"')



# Define the main function to run the node
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
