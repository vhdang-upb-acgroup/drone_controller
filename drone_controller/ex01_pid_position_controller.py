# ROS PY libs
import rclpy
from rclpy.node import Node
# ROS msg Libs
from geometry_msgs.msg import PoseArray, PointStamped
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header

# Basic python libs
import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler_scipy(w, x, y, z):
    # Reorder to [x, y, z, w] for scipy
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=True)  # Roll, pitch, yaw in degree

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


def generate_square_waypoints(size=2.0, height=2.0):
    """
    Generate waypoints for a square trajectory centered at (0,0) at a fixed height.

    Args:
        size (float): length of one side of the square (meters)
        height (float): z-coordinate (altitude) for all waypoints

    Returns:
        list of [x, y, z] waypoints forming a square loop
    """
    half = size / 2.0
    waypoints = [
        [ half,  half, height],  # Top-right
        [-half,  half, height],  # Top-left
        [-half, -half, height],  # Bottom-left
        [ half, -half, height],  # Bottom-right
        [ half,  half, height],  # Back to start to close loop
    ]
    return waypoints

class PositionController(Node):
    def __init__(self):
        super().__init__('Position_Controller')

        # Step 1: Define several parameters or variables
        # Memories for integral, last error
        self.integral_z = 0.0
        self.last_error_z = 0.0

        self.integral_y = 0.0
        self.last_error_y = 0.0

        self.integral_x = 0.0
        self.last_error_x = 0.0

        self.integral_roll = 0.0
        self.last_error_roll = 0.0

        self.integral_pitch = 0.0
        self.last_error_pitch = 0.0


        # Declare last_time as current time, current x, y and z
        self.last_time = self.get_clock().now()

        
        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.roll = None
        self.pitch = None
        self.yaw = None

        self.current_wp_idx = 0
        self.wp_threshold = 0.05



        # Step 2: 
        # Set traj_type:
        traj_type = "default" #"Helix" #"Figure8", "Random_waypoints", "Cool_acrobatic"

        if traj_type == "default":
            self.trajectory = generate_square_waypoints(size=3.0, height=4.0)

        #Step 3: 
        # Step 3.1: Get the feedback from gazebo
        self.subscription = self.create_subscription(
            PoseArray, # Ros msg datatype
            '/world/quadcopter/pose/info', # Topic name
            self.pose_callback, # Callback function 
            10)
        # Step 3.2: 
        self.publisher_ = self.create_publisher(
            Actuators, # Ros msg datatype
            '/X3/gazebo/command/motor_speed', # Topic name
            10)
        #Step 4: Create a publisher to publish the ref pos for plotting
        self.ref_pub = self.create_publisher(PointStamped, '/drone/ref_pos', 10)

    #############################################################################################################

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) <2:
            self.get_logger().warn("PoseArray has fewer than 2 poses")
            return
        # Step 3.1 in details: Retrieve the linear position and orientation
        self.current_z = msg.poses[1].position.z
        self.current_y = msg.poses[1].position.y
        self.current_x = msg.poses[1].position.x
        self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
            msg.poses[1].orientation.w,
            msg.poses[1].orientation.x,
            msg.poses[1].orientation.y,
            msg.poses[1].orientation.z
        )
        #################################
        self.control_loop()

    def control_loop(self):
        if self.current_z is None:
            # initialization code...
            return

        # Ensure current_wp_idx is valid before accessing
        if self.current_wp_idx >= len(self.trajectory):
            self.current_wp_idx = 0

        self.desired_pos = self.trajectory[self.current_wp_idx]

        dist = np.linalg.norm([
            self.desired_pos[0] - self.current_x,
            self.desired_pos[1] - self.current_y,
            self.desired_pos[2] - self.current_z
        ])

        if dist < self.wp_threshold:
            self.current_wp_idx += 1  # Move to next waypoint
            if self.current_wp_idx >= len(self.trajectory):
                self.current_wp_idx = 0  # Loop back to start

        self.desired_pos = self.trajectory[self.current_wp_idx]


        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.001:
            return

        '''
            PID controller for altitude control loop
        '''
        omega_z, self.integral_z, self.last_error_z = pid_controller(self.desired_pos[2], self.current_z, self.last_error_z,self.integral_z, dt, 40, 3.50, 50)

        '''
            Design PID controllers for x, y tracking.
            Note that outputs of these controller are desired roll and pitch angles that are handled by attitude control loop
        '''
        desired_roll, self.integral_y, self.last_error_y = pid_controller(self.desired_pos[1], self.current_y, self.last_error_y,self.integral_y, dt, 50, 1.0, 50)
        desired_roll = clamp(desired_roll, -5, 5)
        desired_pitch, self.integral_x, self.last_error_x = pid_controller(self.desired_pos[0], self.current_x, self.last_error_x,self.integral_x, dt, 50, 1.0, 50)
        desired_pitch = clamp(desired_pitch, -5, 5)
        '''
            This is attitude control loop
        '''
        omega_roll, self.integral_roll, self.last_error_roll = pid_controller(-desired_roll, self.roll, self.last_error_roll,self.integral_roll, dt, 1, 0, 1)
        omega_pitch, self.integral_pitch, self.last_error_pitch = pid_controller(desired_pitch, self.pitch, self.last_error_pitch,self.integral_pitch, dt, 1, 0, 1)
        
        self.integral_z = clamp(self.integral_z, -10, 10.0)
        self.integral_roll = clamp(self.integral_roll, -1.0, 1.0)
        self.integral_pitch = clamp(self.integral_pitch, -1.0, 1.0)
        self.integral_y = clamp(self.integral_y, -1.0, 1.0)
        self.integral_x = clamp(self.integral_x, -1.0, 1.0)
        # Motor speeds:
        motor0 = clamp(636.0 - omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor1 = clamp(636.0 + omega_roll + omega_pitch + omega_z, 400.0, 800.0)
        motor2 = clamp(636.0 + omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor3 = clamp(636.0 - omega_roll + omega_pitch + omega_z, 400.0, 800.0)

        #########################################################################
        # Publish the angular velocities to gazebo
        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        cmd.velocity = [motor0, motor1, motor2, motor3]
        self.publisher_.publish(cmd)


        self.last_time = now

        self.get_logger().info(f"x, y, z: {self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f} m |\
                                        Desired roll, pitch angles: {desired_roll:.2f}, {desired_pitch:.2f}| \
                                        Command: {motor0:.2f}, {motor1:.2f}, {motor2:.2f}, {motor3:.2f} rad/s | \
                                        roll: {self.roll:.2f} | pitch: {self.pitch:.2f}")
        

        # Publish reference position for plotting
        ref_msg = PointStamped()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.point.x = self.desired_pos[0]
        ref_msg.point.y = self.desired_pos[1]
        ref_msg.point.z = self.desired_pos[2]
        self.ref_pub.publish(ref_msg)

def pid_controller(set_point, actual, last_error, last_integral, dt, kp, ki, kd):
        error = set_point-actual
        integral = last_integral + error * dt
        derivative = (error - last_error) / dt

        return kp * error + ki * integral + kd * derivative, integral, error



def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
