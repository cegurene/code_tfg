import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import socket


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.actionFlag = False

        self.i = 0.0
        

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        print("Starting client")

        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = 9998

        if not self.actionFlag:
            try:
                self.socket_client.connect(('localhost', self.port))
                response = self.socket_client.recv(1024)
                print(f"Action from server: {response.decode()}")
                self.actionFlag = True

            except Exception as e:
                print(f"Error occurred: {e}")
            finally:
                self.socket_client.close()
                print("Connection closed.")
        
        else:
            try:
                self.socket_client.connect(('localhost', self.port))
                print(f"Connected to server on port {self.port}")

                position = self.i
                self.i += 0.1

                velocity = 0.0
                torque = 0.0

                message = f"{position}, {velocity}, {torque}"
                self.socket_client.sendall(message.encode())
                print("Observation sent!")
                self.actionFlag = False

            except Exception as e:
                print(f"Error occurred: {e}")
            finally:
                self.socket_client.close()
                print("Connection closed.")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()