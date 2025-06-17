import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.subscription = self.create_subscription(
            PoseArray,
            '/world/quadcopter/pose/info',
            self.pose_callback,
            10)

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) > 1:
            pose = msg.poses[1]  # Second pose
            self.get_logger().info(
                f"Second Pose: Position({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) "
                f"Orientation({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, "
                f"{pose.orientation.z:.2f}, {pose.orientation.w:.2f})"
            )
        else:
            self.get_logger().warn("Received PoseArray with fewer than 2 poses")

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

