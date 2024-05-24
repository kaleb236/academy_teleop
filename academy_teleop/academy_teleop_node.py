import rclpy
import socket
import threading
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SocketTeleop(Node):
    def __init__(self, host='192.168.31.28', port=65432) -> None:
        super().__init__('academy_teleop_node')
        self.host = host
        self.port = port
        self.vel = Twist()
        self.conn = None

        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, 'wheel/odom', self.odom_callback, 10)

        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            print(f'Server listening on {self.host}:{self.port}')
            self.conn, addr = s.accept()
            with self.conn:
                print(f'Connected by {addr}')
                while True:
                    try:
                        data = self.conn.recv(1024)
                        if not data:
                            break
                        print(f'Received from client: {data.decode()}')
                        try:
                            vel = data.decode().split(',')
                            self.publish_vel(vel[0], vel[1])
                        except:
                            self.get_logger().info("invalid velocit format")
                        self.conn.sendall(data)
                    except Exception as e:
                        self.get_logger().info("Connection faill")
                        s.close()
                        self.start_server()
    
    def publish_vel(self, x, z):
        self.vel.linear.x = float(x)
        self.vel.angular.z = float(z)
        self.pub_vel.publish(self.vel)
    
    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        if self.conn is not None:
            try:
                self.conn.sendall(f'{x}, {y}'.encode())
                # time.sleep(0.1)
            except Exception as e:
                print('connection error', e)
                self.conn = None

def main(args=None):
    rclpy.init(args=args)
    socket_teleop = SocketTeleop()
    rclpy.spin(socket_teleop)
    socket_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()