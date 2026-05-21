# We import the necessary packages
import rclpy
from rclpy.node import Node

# Note: the Float32MultiArray may not work on ROS2 Humble, as ROS2 developers don't like the use of generic messages types and is supposed to be depreciated in Foxy.
from std_msgs.msg import Float32MultiArray
from rl_interfaces.msg import MotionCommand

import numpy
#import sys

#print(sys.version)
#sys.path.append('/home/athens/miniconda/envs/myosuite/lib/python3.9/site-packages')

#from myosuite.utils import gym


class MinimalPublisher(Node):
    #env = gym.make('ExoLegSpasticityFlexoExtEnv-v0')

    def __init__(self):
        # We create the MyoSuite environment, the subscriber to listen to the actions and the publisher to send the observations
        super().__init__('myosuite')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/env/myosuite_obs', 10)

        #env = gym.make('ExoLegSpasticityFlexoExtEnv-v0')
        #self.env.reset()


        self.subscription = self.create_subscription(
            MotionCommand,              # Message type used by the publisher
            '/md80/motion_command',     # topic name
            self.listener_callback,     # Method called each time a message is received
            10)                         # Queue size for incoming messages
        self.subscription  # Prevent unused variable warning


    def listener_callback(self, msg):
        # We get the action, perform a step in the MyoSuite environment and publish the obtained observations
        # The action will have 41 floats and the observations will have 91 floats, probably as a numpy array

        # Log the received message to the console
        self.get_logger().info(f'Torque received: "{msg.target_torque}"')

        # Here we would do the env.step to obtain the observations

        array = numpy.array([0.0,msg.target_torque[0],2.0,3.0,4.0,5.0])
        msg_pub = Float32MultiArray() # The published message is an array of floats
        msg_pub.data = array.tolist() # We convert the numpy array into a Python list and save it into the message
        self.publisher_.publish(msg_pub)
        self.get_logger().info('Publishing MyoSuite observations: "%s"' % msg_pub.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher() # We create the node

    try:
        # Run the node until interrupted
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        #minimal_publisher.close_env()
        minimal_publisher.destroy_node()
        rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
