import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plots
import csv
import os
from datetime import datetime

class LiveTrajectory3DPlot(Node):
    def __init__(self):
        super().__init__('live_trajectory_3d_plotter')

        # Folder to save csv files (create if not exists)
        self.csv_folder = os.path.expanduser('~/nlcars_ws/trajectory_logs')
        os.makedirs(self.csv_folder, exist_ok=True)

        # Subscriptions
        self.create_subscription(PoseArray, '/world/quadcopter/pose/info', self.pose_callback, 10)
        self.create_subscription(PointStamped, '/drone/ref_pos', self.ref_callback, 10)

        # Data storage
        self.actual_x, self.actual_y, self.actual_z = [], [], []
        self.ref_x, self.ref_y, self.ref_z = [], [], []

        # Setup 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line_actual, = self.ax.plot([], [], [], 'b-', label='Actual Path')
        self.line_ref, = self.ax.plot([], [], [], 'r--', label='Reference Path')

        self.ax.set_title("Drone 3D Trajectory")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.legend()
        self.ax.grid()

        # Start periodic plot update every 0.2s
        self.create_timer(0.2, self.update_plot)

        # Non-blocking plot mode
        plt.ion()
        plt.show()

    def save_to_csv(self):
        if not self.actual_x or not self.ref_x:
            return  # nothing to save

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        actual_file = os.path.join(self.csv_folder, f'actual_path_{timestamp}.csv')
        ref_file = os.path.join(self.csv_folder, f'ref_path_{timestamp}.csv')

        # Save actual path
        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for x, y, z in zip(self.actual_x, self.actual_y, self.actual_z):
                writer.writerow([x, y, z])

        # Save reference path
        with open(ref_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            for x, y, z in zip(self.ref_x, self.ref_y, self.ref_z):
                writer.writerow([x, y, z])

        self.get_logger().info(f"Saved actual path to {actual_file}")
        self.get_logger().info(f"Saved reference path to {ref_file}")

    def reset_plot(self):
        self.save_to_csv()
        self.actual_x.clear()
        self.actual_y.clear()
        self.actual_z.clear()
        self.ref_x.clear()
        self.ref_y.clear()
        self.ref_z.clear()

        self.line_actual.set_data([], [])
        self.line_actual.set_3d_properties([])

        self.line_ref.set_data([], [])
        self.line_ref.set_3d_properties([])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def pose_callback(self, msg):
        if len(msg.poses) < 2:
            return
        p = msg.poses[1].position
        self.actual_x.append(p.x)
        self.actual_y.append(p.y)
        self.actual_z.append(p.z)

    def ref_callback(self, msg):
        p = msg.point
        self.ref_x.append(p.x)
        self.ref_y.append(p.y)
        self.ref_z.append(p.z)

    def update_plot(self):
        if len(self.actual_x) < 2 or len(self.ref_x) < 2:
            return  # wait until we have enough data

        self.line_actual.set_data(self.actual_x, self.actual_y)
        self.line_actual.set_3d_properties(self.actual_z)

        self.line_ref.set_data(self.ref_x, self.ref_y)
        self.line_ref.set_3d_properties(self.ref_z)

        # Autoscale axes manually
        all_x = self.actual_x + self.ref_x
        all_y = self.actual_y + self.ref_y
        all_z = self.actual_z + self.ref_z
        margin = 1.0

        self.ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        self.ax.set_zlim(min(all_z) - margin, max(all_z) + margin)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Example: reset plot if actual path is longer than 1000 points (adjust as needed)
        if len(self.actual_x) > 10000:
            self.get_logger().info("Resetting plot and saving data...")
            self.reset_plot()


def main():
    rclpy.init()
    node = LiveTrajectory3DPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
