# We import the necessary packages

import rclpy
from rclpy.node import Node

# Note: the Float32MultiArray may not work on ROS2 Humble, as ROS2 developers don't like the use of generic messages types and is supposed to be depreciated in Foxy.
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

import numpy



class MinimalPublisher(Node):

    def __init__(self, control_type):
        # We create the publishers and subscribers necessary depending on the control_type selected
        super().__init__('aggregator')
        # We create a publisher to send to the RL a float array with the topic name /rl/observations
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rl/observations', 10)

        self.control_type = control_type

        # We define the flags and messages used by the control_type 1, using the myosuite environment and MD80 motor
        self.flagMyosuite = False
        self.flagMd80Exo = False
        self.msgMyosuite = []
        self.msgMd80Exo = []

        if (control_type == 0):
            # This control_type uses only the virtual MyoSuite environment, receiving the messages and directly sending them to the RL node
            print('Environment selected: MyoSuite simulator only')

            # We create a subscriber to listen to the MyoSuite environment observations
            self.subscription = self.create_subscription(
                Float32MultiArray,              # Message type used by the publisher
                '/env/myosuite_obs',            # topic name
                self.listener_0_callback,       # Method called each time a message is received
                10)                             # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

        elif (control_type == 1):
            # This control type uses both the MyoSuite environment and the MD80 exoskeleton
            # Once it has received a new message from both sources, it will combine them and publish the result to the RL node
            print('Environment selected: MyoSuite simulator and MD80 exoskeleton')

            # We create a subscriber to listen to the MyoSuite environment observations
            self.subscription = self.create_subscription(
                Float32MultiArray,                  # Message type used by the publisher
                '/env/myosuite_obs',                # topic name
                self.listener_1_myosuite_callback,  # Method called each time a message is received
                10)                                 # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

            # We create a subscriber to listen to the MD80 motor
            self.subscription = self.create_subscription(
                JointState,                         # Message type used by the publisher
                '/md80/joint_states',               # topic name
                self.listener_1_exo_callback,       # Method called each time a message is received
                10)                                 # Queue size for incoming messages
            self.subscription  # Prevent unused variable warning

        else:
            print('The environment selected is not valid, please restart the node')

    def listener_0_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info(f'Myosuite: "{msg.data}"')
        # Directly try to publish the message received by the MyoSuite enviroment, as they have the same structure
        self.publish(msg)

    def listener_1_myosuite_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info(f'Myosuite data: "{msg.data}"')

        self.flagMyosuite = True    # Alert that we have received an update from the MyoSuite environment
        self.get_logger().info('FlagMyosuite to True')
        self.msgMyosuite = msg.data     # Save the array from the message
        self.publish(msg)   # Try to send the observations to the RL node 

    def listener_1_exo_callback(self, msg):
        # Log the received message to the console
        # JointState has a string[] name, double[] position, double[] velocity and double[] effort
        self.get_logger().info(f'Exo data: "{msg.position}"')

        self.flagMd80Exo = True # Alert that we have received an update from the MD80 motor
        self.get_logger().info('FlagExo to True')
        self.msgMd80Exo = msg.position  # Save the position value (float[]) from the message
        self.publish(msg)   # Try to send the observations to the RL node

    def publish(self, msg):
        
        if (self.control_type == 0):
            # As the message structure is the same for the MyoSuite environment and the RL node, send this message directly
            msg_pub = Float32MultiArray() # Create a message as a float array
            msg_pub.data = msg.data
            self.publisher_.publish(msg_pub)
            self.get_logger().info('Publishing RL observation: "%s"' % msg_pub.data)

        elif (self.control_type == 1) & self.flagMyosuite & self.flagMd80Exo:
            # If we have received an update from both the MyoSuite environment and the MD80 motor, we can send a new observation to the RL node
            msg_pub = Float32MultiArray() # Create a message as a float array
            observations = self.msgMyosuite     # Get the array from the MyoSuite environment
            observations[0] = self.msgMd80Exo[0]    # Change the motor position value with the real position from the exoskeleton motor 
            msg_pub.data = observations # Save the array into the message
            self.publisher_.publish(msg_pub)    # Publish the observation message to the RL node
            self.get_logger().info('Publishing RL observation: "%s"' % msg_pub.data)

            # Update the flags to wait for a new set of observations to come
            self.flagMd80Exo = False
            self.flagMyosuite = False
            self.get_logger().info('Flags to false')
            

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    print('Introduce an int to select the desired environment for the RL: \n 0: MyoSuite simulator only\n 1: MyoSuite simulator and md80 exoskeleton motor')
    control_type = int(input('Select RL environment: '))
    # Create an instance of the MinimalPublisher node
    minimal_pubsub = MinimalPublisher(control_type)

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_pubsub)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        # Destroy the node and shutdown the ROS2 Python client library
        minimal_pubsub.destroy_node()
        rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
