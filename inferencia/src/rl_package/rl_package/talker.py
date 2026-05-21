import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#import socket


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        """
        print("Starting server")
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_server.bind(('localhost', 9998))
        print("Server started on port 9998")
        self.socket_server.listen(1)

        client_socket, addr = self.socket_server.accept() #Esta llamada bloquea hasta que un cliente se conecta
        print(f"Connection from {addr} has been established!")

        try:
            while True:
                data = client_socket.recv(1024) #Se guarda en data aunque no lo leas directamente
                print(f"Received data: {data.decode()}")
                message = "Received your message. Thank you!"
                client_socket.sendall(message.encode())
                if not data:
                    print("No more data received, closing connection.")
                
                    break
        finally:
            client_socket.close()
            print("Connection closed.")
        """

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()