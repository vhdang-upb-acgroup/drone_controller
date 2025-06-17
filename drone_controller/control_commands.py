import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators  # Make sure this matches your message package
from std_msgs.msg import Header
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Actuators()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set motor velocities (rad/s), adjust as needed
        msg.velocity = [0.0, 0.0, 00.0, 20.0]

        # Optional: clear other fields
        msg.position = []
        msg.normalized = []

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published motor velocities: {msg.velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
