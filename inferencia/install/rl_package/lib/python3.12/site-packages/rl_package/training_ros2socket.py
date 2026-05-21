# We import the packages necessary for ROS2
import rclpy
from rclpy.node import Node

# Here are the packages for the messages
from rl_interfaces.msg import MotionCommand
from sensor_msgs.msg import JointState

# We import the packages necessary for the communication with the environment
import socket


class MinimalSubscriber(Node):

    def __init__(self):
        # We create the publisher for the RL actions and the subscriber for the RL observations
        super().__init__('training_ros2socket')
        # The actions are the torque for the motors, published in the structure accepted by the MD80 motor.
        self.publisher_ = self.create_publisher(MotionCommand, '/md80/motion_command', 10)
        self.motor_id = int(input("Enter the motor ID to control: "))  # Get motor ID from user input
        self.port = int(input("Enter port to connect to (by default 9998): "))  # Get port from user input

        self.subscription = self.create_subscription(
            JointState,             # Message type used by the publisher
            '/md80/joint_states',   # Topic name
            self.listener_callback, # Method called each time a message is received
            10)                     # Queue size for incoming messages
        self.subscription  # prevent unused variable warning

        self.actionFlag = False     # Flag to determine if we have received the actions from the environment
        self.msgMd80ExoHz = 100     # /md80/joint_states frequency, to calculate the acceleration
        self.msgMd80ExoVel = 0      # Initial motor velocity
        

    def listener_callback(self, msg):
        # JointState has a string[] name, double[] position, double[] velocity and double[] effort
        self.get_logger().info('Exo data: "%s"' % msg.position)

        print("Starting client")
        # We create the socket client to connect to the server in the environment
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        if not self.actionFlag: # If we have not received an action (a message from the environment)
            try:
                self.socket_client.connect(('localhost', self.port))    # We try to connect with the server, if it is down, an error will be caught
                response = self.socket_client.recv(1024)                # We receive the message with the action
                print(f"Action from server: {response.decode()}")

                act = float(response.decode())  # Assuming the server sends a float action (motor torque from 0 to 1)

                msg_pub = MotionCommand()   # We create a MotionCommand message
                msg_pub.drive_ids = [self.motor_id]  # Use the motor ID from user input
                msg_pub.target_torque = [act]   # We introduce the torque obtained from the RL into the message
                # Publish the message
                self.publisher_.publish(msg_pub)       
                # Log the published message to the console
                self.get_logger().info(f'Publishing torque: "{msg_pub.target_torque}"')

                self.actionFlag = True  # We flag that we have received an action

            except Exception as e:
                print(f"Error occurred: {e}")
            finally:
                self.socket_client.close()  # Whether we received a message or not, we close the connection
                print("Connection closed.")
        
        else:   # If we have received an action in the previous iteration
            try:
                self.socket_client.connect(('localhost', self.port))    # We try to connect with the server, if it is down, an error will be caught
                print(f"Connected to server on port {self.port}")

                # We get all the parameters
                position = msg.position[0]
                acceleration = (msg.velocity[0] - self.msgMd80ExoVel) /(2*(1/self.msgMd80ExoHz))
                self.msgMd80ExoVel = msg.velocity[0]
                velocity = msg.velocity[0]
                torque = msg.effort[0]

                # We construct the message and send it through the socket
                message = f"{position}, {velocity}, {acceleration}, {torque}"
                self.socket_client.sendall(message.encode())
                print("Observation sent!")

                self.actionFlag = False # We reset the action Flag

            except Exception as e:
                print(f"Error occurred: {e}")
            finally:
                self.socket_client.close()  # Whether we received a message or not, we close the connection
                print("Connection closed.")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_subscriber = MinimalSubscriber()

    # Run the node until interrupted
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()