# ROS PY libs
import rclpy
from rclpy.node import Node

# ROS msg Libs
from geometry_msgs.msg import PoseArray, PointStamped
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header

# Python libs
import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler_scipy(w, x, y, z):
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=True)

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def pid_controller(set_point, actual, last_error, last_integral, dt, kp, ki, kd):
    error = set_point - actual
    integral = last_integral + error * dt
    derivative = (error - last_error) / dt
    return kp * error + ki * integral + kd * derivative, integral, error

def feedback_linearization(pos, vel, pos_ref, vel_ref, int_error, psi=0.0):
    g = 9.81  # gravity
    m = 1.55  # mass of the quadrotor

    Kp = np.diag([2.0, 2.0, 13.0])
    Kd = np.diag([1.5, 1.5, 12.5])
    Ki = np.diag([1.0, 1.0, 5.0])  # Tune this

    a_des = Kp @ (pos_ref - pos) + Kd @ (vel_ref - vel) + Ki @ int_error
    u0 = m * (g + a_des[2])
    denom = g + a_des[2] if abs(g + a_des[2]) > 1e-4 else 1e-4

    phi = psi - a_des[1] / denom
    theta = a_des[0] / denom - phi * psi

    return u0, phi, theta

class TrajectoryTrackingController(Node):
    def __init__(self):
        super().__init__('Trajectory_Controller')

        # PID states
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        self.last_error_roll = 0.0
        self.last_error_pitch = 0.0

        self.last_time = self.get_clock().now()

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.integrated_position_error = np.zeros(3)
        

        self.subscription = self.create_subscription(
            PoseArray, '/world/quadcopter/pose/info', self.pose_callback, 1)
        self.publisher_ = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 1)
        self.ref_pub = self.create_publisher(PointStamped, '/drone/ref_pos', 1)

    def get_reference(self, t):
        """Circular trajectory in x-y plane at constant height"""
        radius = 2.0
        speed = 0.15  # m/s
        omega = speed / radius

        x_ref = radius * np.cos(omega * t)
        y_ref = radius * np.sin(omega * t)
        z_ref = 3.0  # constant altitude

        dx_ref = -radius * omega * np.sin(omega * t)
        dy_ref = radius * omega * np.cos(omega * t)
        dz_ref = 0.0

        return np.array([x_ref, y_ref, z_ref]), np.array([dx_ref, dy_ref, dz_ref])

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return

        pose = msg.poses[1]
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_z = pose.position.z

        self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z
        )

        self.control_loop()

    def control_loop(self):
        if self.current_z is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.001:
            return

        pos = np.array([self.current_x, self.current_y, self.current_z])
        vel = np.array([
            (self.current_x - self.previous_x) / dt,
            (self.current_y - self.previous_y) / dt,
            (self.current_z - self.previous_z) / dt
        ])

        t = now.nanoseconds / 1e9  # time in seconds
        pos_des, vel_des = self.get_reference(t)
        yaw_des = 0.0

        # Accumulate position error (with anti-windup)
        pos_error = pos_des - pos
        self.integrated_position_error += pos_error * dt
        self.integrated_position_error = np.clip(self.integrated_position_error, -2.0, 2.0)  # anti-windup

        u0, roll_des, theta_des = feedback_linearization(pos, vel, pos_des, vel_des, self.integrated_position_error, yaw_des)

        omega_roll, self.integral_roll, self.last_error_roll = pid_controller(
            roll_des, self.roll, self.last_error_roll, self.integral_roll, dt, 1.0, 0.0, 1.0)
        omega_pitch, self.integral_pitch, self.last_error_pitch = pid_controller(
            theta_des, self.pitch, self.last_error_pitch, self.integral_pitch, dt, 1.0, 0.0, 1.0)

        omega_z = u0

        base_rpm = 630.36
        motor0 = clamp(base_rpm - omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor1 = clamp(base_rpm + omega_roll + omega_pitch + omega_z, 400.0, 800.0)
        motor2 = clamp(base_rpm + omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor3 = clamp(base_rpm - omega_roll + omega_pitch + omega_z, 400.0, 800.0)

        self.get_logger().info(
            f"pos: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) | "
            f"ref: ({pos_des[0]:.2f}, {pos_des[1]:.2f}, {pos_des[2]:.2f}) | "
            f"u0: {u0:.2f} | motors: {motor0:.1f}, {motor1:.1f}, {motor2:.1f}, {motor3:.1f}"
        )

        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        cmd.velocity = [motor0, motor1, motor2, motor3]
        self.publisher_.publish(cmd)

        ref_msg = PointStamped()
        ref_msg.header.stamp = now.to_msg()
        ref_msg.point.x = pos_des[0]
        ref_msg.point.y = pos_des[1]
        ref_msg.point.z = pos_des[2]
        self.ref_pub.publish(ref_msg)

        self.last_time = now
        self.previous_x = self.current_x
        self.previous_y = self.current_y
        self.previous_z = self.current_z

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
