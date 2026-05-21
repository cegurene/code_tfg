# Import the ROS2 Python client library and all the necessary packages
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
            '/md80/motion_command',     # topic name
            self.listener_callback,     # Method called each time a message is received
            10)                         # Queue size for incoming messages
        self.subscription  # Prevent unused variable warning
        

    def listener_callback(self, msg):
        # Each time we receive a message from the RL, we publish a position in the joint_states, even though this may not be the configuration of the real MD80 ROS node
        # MotionCommand has uint32[] drive_ids, float32[] target_position, float32[] target_velocity, float32[] target_torque
        self.get_logger().info(f'Received torque: "{msg.target_torque}"')
        msgJoint = JointState() #JointState has a string[] name, double[] position, double[] velocity and double[] effort
        msgJoint.position = [10.0, 0.0]
        self.publisherJoint_.publish(msgJoint)
        self.get_logger().info(f'Publishing position: "{msgJoint.position}"')



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
