3. Implement the control system in drone_controller package
```bash
   # PLEASE MAKE SURE YOU HAVE YOUR OWN ROS2 PACKAGE, e.g., I have drone_controller package, and I now can add a node inside it
   # Navigate to your local working directory, then /src/drone_controller
   # Create a python file, called pid_z_controller_update.py
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PoseArray
   from actuator_msgs.msg import Actuators
   from std_msgs.msg import Header
   import numpy as np
   from scipy.spatial.transform import Rotation as R

   def quaternion_to_euler_scipy(w, x, y, z):
      # Reorder to [x, y, z, w] for scipy
      r = R.from_quat([x, y, z, w])
      return r.as_euler('xyz', degrees=True)  # Roll, pitch, yaw in degree
   def clamp(val, min_val, max_val):
      return max(min(val, max_val), min_val)


   class HoverController(Node):
      def __init__(self):
         super().__init__('hover_controller')

         # Memories for integral, last error, and integral limit
         self.integral_z = 0.0
         self.last_error_z = 0.0
         self.integral_limit_z = 50.0


         self.integral_y = 0.0
         self.last_error_y = 0.0
         self.integral_limit_y = 50.0

         self.integral_x = 0.0
         self.last_error_x = 0.0
         self.integral_limit_x = 50.0

         ###########################
         self.integral_roll = 0.0
         self.last_error_roll = 0.0
         self.integral_limit_roll = 50.0

         self.integral_pitch = 0.0
         self.last_error_pitch = 0.0
         self.integral_limit_pitch = 50.0


         # Declare last_time as current time, current x, y and z
         self.last_time = self.get_clock().now()
         self.current_z = None
         self.current_x = None
         self.current_y = None

         self.roll = None
         self.pitch = None
         self.yaw = None

         self.subscription = self.create_subscription(
               PoseArray,
               '/world/quadcopter/pose/info',
               self.pose_callback,
               10)

         self.publisher_ = self.create_publisher(
               Actuators,
               '/X3/gazebo/command/motor_speed',
               10)

      def pose_callback(self, msg: PoseArray):
         if len(msg.poses) <= 1:
               self.get_logger().warn("PoseArray has fewer than 2 poses")
               return
         self.current_z = msg.poses[1].position.z
         self.current_y = msg.poses[1].position.y
         self.current_x = msg.poses[1].position.x
         self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
               msg.poses[1].orientation.w,
               msg.poses[1].orientation.x,
               msg.poses[1].orientation.y,
               msg.poses[1].orientation.z
         )
         self.control_loop()

      def control_loop(self):
         if self.current_z is None:
               self.integral_z = 0.0
               self.integral_roll = 0.0
               self.integral_pitch = 0.0
               self.last_time = self.get_clock().now()
               return

         now = self.get_clock().now()
         dt = (now - self.last_time).nanoseconds / 1e9
         if dt <= 0.001:
               return

         '''
               Define PID controller for z level control
               Step 1: Calculate error_z = z_d - current_z
               Step 2: Calculate omega_z based on PID controller
               Step 3: Add it to the base angular velocity
         '''
         omega_z, self.integral_z, self.last_error_z = pid_controller(2.0, self.current_z, self.last_error_z,self.integral_z, dt, 80, 1.0, 50)
         
         '''
               Make sure roll and pitch angles are zero
         '''
         omega_roll, self.integral_roll, self.last_error_roll = pid_controller(0.0, self.roll, self.last_error_roll,self.integral_roll, dt, 0.1, 0.0, 1)
         omega_pitch, self.integral_pitch, self.last_error_pitch = pid_controller(0.0, self.pitch, self.last_error_pitch,self.integral_pitch, dt, 0.1, 0.0, 1)
         
         self.integral_z = clamp(self.integral_z, -15.0, 15.0)
         self.integral_roll = clamp(self.integral_roll, -1.0, 1.0)
         self.integral_pitch = clamp(self.integral_pitch, -1.0, 1.0)
         # Motor speeds:
         motor0 = clamp(636.0 - omega_roll - omega_pitch + omega_z, 400.0, 800.0)
         motor1 = clamp(636.0 + omega_roll + omega_pitch + omega_z, 400.0, 800.0)
         motor2 = clamp(636.0 + omega_roll - omega_pitch + omega_z, 400.0, 800.0)
         motor3 = clamp(636.0 - omega_roll + omega_pitch + omega_z, 400.0, 800.0)


         cmd = Actuators()
         cmd.header = Header()
         cmd.header.stamp = now.to_msg()
         cmd.velocity = [motor0, motor1, motor2, motor3]

         self.publisher_.publish(cmd)
         self.last_time = now
         self.get_logger().info(f"Height: {self.current_z:.2f} m | Command: {motor0:.2f}, {motor1:.2f}, {motor2:.2f}, {motor3:.2f} rad/s || roll: {self.roll:.2f} | pitch: {self.pitch:.2f}")
         

   def pid_controller(set_point, actual, last_error, last_integral, dt, kp, ki, kd):
         error = set_point-actual
         integral = last_integral + error * dt
         derivative = (error - last_error) / dt

         return kp * error + ki * integral + kd * derivative, integral, error



   def main(args=None):
      rclpy.init(args=args)
      node = HoverController()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()


   if __name__ == '__main__':
      main()

```
# Test our pid_z_controller_update.py
1. using colcon build the drone_controller package
```bash
   colcon build --packages-select drone_controller
```
2. Remember to source before run the node
```bash
   source install/setup.bash
   ros2 run drone_controller <node name you specify in setup.py>
```
# The result will be like this
![Hovering result](docs/hovering_task.gif)

## Exercise 5: Design a control system that allows the drone performs way-point tracking
1. First we need to design a control system scheme that need to consider both attitude and altitude control loops. The control scheme should look like this
![way-point tracking controller scheme](docs/pid_control_system_for_way_point_tracking.png)
2. Then you create another python node inside drone_controller package
```bash
   # You should navigate to drone_controller/drone_controller
   # Create a python file, named pid_pos_controller.py
   # I recommend you to implement it by yourself at first. You can consider my implementation as below
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler_scipy(w, x, y, z):
    # Reorder to [x, y, z, w] for scipy
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=True)  # Roll, pitch, yaw in degree
def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')

        # Memories for integral, last error, and integral limit
        self.integral_z = 0.0
        self.last_error_z = 0.0
        self.integral_limit_z = 50.0


        self.integral_y = 0.0
        self.last_error_y = 0.0
        self.integral_limit_y = 50.0

        self.integral_x = 0.0
        self.last_error_x = 0.0
        self.integral_limit_x = 50.0

        ###########################
        self.integral_roll = 0.0
        self.last_error_roll = 0.0
        self.integral_limit_roll = 50.0

        self.integral_pitch = 0.0
        self.last_error_pitch = 0.0
        self.integral_limit_pitch = 50.0


        # Declare last_time as current time, current x, y and z
        self.last_time = self.get_clock().now()
        self.current_z = None
        self.current_x = None
        self.current_y = None

        self.roll = None
        self.pitch = None
        self.yaw = None

        self.subscription = self.create_subscription(
            PoseArray,
            '/world/quadcopter/pose/info',
            self.pose_callback,
            10)

        self.publisher_ = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10)

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) <= 1:
            self.get_logger().warn("PoseArray has fewer than 2 poses")
            return
        self.current_z = msg.poses[1].position.z
        self.current_y = msg.poses[1].position.y
        self.current_x = msg.poses[1].position.x
        self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
            msg.poses[1].orientation.w,
            msg.poses[1].orientation.x,
            msg.poses[1].orientation.y,
            msg.poses[1].orientation.z
        )
        self.control_loop()

    def control_loop(self):
        if self.current_z is None:
            self.integral_z = 0.0
            self.integral_roll = 0.0
            self.integral_pitch = 0.0
            self.last_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.001:
            return

        '''
            PID controller for altitude control loop
        '''
        omega_z, self.integral_z, self.last_error_z = pid_controller(5.0, self.current_z, self.last_error_z,self.integral_z, dt, 80, 1.0, 50)

        '''
            Design PID controllers for x, y tracking.
            Note that outputs of these controller are desired roll and pitch angles that are handled by attitude control loop
        '''
        desired_roll, self.integral_y, self.last_error_y = pid_controller(4.0, self.current_y, self.last_error_y,self.integral_y, dt, 80, 1.0, 50)
        desired_roll = clamp(desired_roll, -5, 5)
        desired_pitch, self.integral_x, self.last_error_x = pid_controller(4.0, self.current_x, self.last_error_x,self.integral_x, dt, 80, 1.0, 50)
        desired_pitch = clamp(desired_pitch, -5, 5)
        '''
            This is attitude control loop
        '''
        omega_roll, self.integral_roll, self.last_error_roll = pid_controller(-desired_roll, self.roll, self.last_error_roll,self.integral_roll, dt, 0.1, 0.0, 1)
        omega_pitch, self.integral_pitch, self.last_error_pitch = pid_controller(desired_pitch, self.pitch, self.last_error_pitch,self.integral_pitch, dt, 0.1, 0.0, 1)
        
        self.integral_z = clamp(self.integral_z, -15.0, 15.0)
        self.integral_roll = clamp(self.integral_roll, -10.0, 10.0)
        self.integral_pitch = clamp(self.integral_pitch, -10.0, 10.0)
        # Motor speeds:
        motor0 = clamp(636.0 - omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor1 = clamp(636.0 + omega_roll + omega_pitch + omega_z, 400.0, 800.0)
        motor2 = clamp(636.0 + omega_roll - omega_pitch + omega_z, 400.0, 800.0)
        motor3 = clamp(636.0 - omega_roll + omega_pitch + omega_z, 400.0, 800.0)


        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        cmd.velocity = [motor0, motor1, motor2, motor3]


        self.publisher_.publish(cmd)
        self.last_time = now
        self.get_logger().info(f"x, y, z: {self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f} m | Desired roll, pitch angles: {desired_roll:.2f}, {desired_pitch:.2f}| Command: {motor0:.2f}, {motor1:.2f}, {motor2:.2f}, {motor3:.2f} rad/s || roll: {self.roll:.2f} | pitch: {self.pitch:.2f}")
        

def pid_controller(set_point, actual, last_error, last_integral, dt, kp, ki, kd):
        error = set_point-actual
        integral = last_integral + error * dt
        derivative = (error - last_error) / dt

        return kp * error + ki * integral + kd * derivative, integral, error



def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
# Test your pid_pos_controller.py
1. using colcon build the drone_controller package
```bash
   colcon build --packages-select drone_controller
```
2. Remember to source before run the node
```bash
   source install/setup.bash
   ros2 run drone_controller <node name you specify in setup.py>
```
# The result will be like this
![way-point-tracking result](docs/way-point-tracking.gif)

## Exercise 6: Make something more interesting: eight-figure trajectory?
1. All you need to do is to define your own trajectory function for which the drone need to tracking
```bash
   def figure8_trajectory(t, A=1.0, B=1.0, omega=1.0, z=1.0):
      """
      Generate desired x, y, z positions following a figure-eight trajectory.
      
      Args:
         t (float): time or trajectory parameter
         A (float): amplitude in x direction
         B (float): amplitude in y direction
         omega (float): angular frequency (controls speed)
         z (float): constant altitude
         
      Returns:
         (x, y, z): desired position tuple
      """
      x = A * math.sin(omega * t)
      y = B * math.sin(omega * t) * math.cos(omega * t)
      return x, y, z
```
2. Can you design and implement more acrobatic trajectory? Now is your time