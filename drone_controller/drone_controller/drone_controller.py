#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, Header


class DroneController(Node):
    """
    ROS2 drone controller node that converts velocity commands to drone control commands.
    
    Subscribes to:
        - /cmd_vel (geometry_msgs/Twist): Velocity commands
        - /cmd_vel_stamped (geometry_msgs/TwistStamped): Timestamped velocity commands
    
    Publishes:
        - /drone/cmd_vel (geometry_msgs/Twist): Drone velocity control commands
        - /drone/thrust (std_msgs/Float64MultiArray): Thrust commands [thrust, roll, pitch, yaw]
    """

    def __init__(self):
        super().__init__("drone_controller")
        
        # Parameters
        self.declare_parameter("max_linear_velocity", 5.0)
        self.declare_parameter("max_angular_velocity", 2.0)
        self.declare_parameter("max_thrust", 1.0)
        self.declare_parameter("base_thrust", 0.5)  # Hover thrust
        self.declare_parameter("control_rate", 50.0)  # Hz
        
        self.max_linear_vel = self.get_parameter("max_linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.max_thrust = self.get_parameter("max_thrust").value
        self.base_thrust = self.get_parameter("base_thrust").value
        control_rate = self.get_parameter("control_rate").value
        
        # State variables
        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10
        )
        
        self.cmd_vel_stamped_sub = self.create_subscription(
            TwistStamped,
            "cmd_vel_stamped",
            self.cmd_vel_stamped_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "drone/cmd_vel",
            10
        )
        
        self.thrust_pub = self.create_publisher(
            Float64MultiArray,
            "drone/thrust",
            10
        )
        
        # Control timer
        timer_period = 1.0 / control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info("Drone controller initialized")
        self.get_logger().info(f"  Max linear velocity: {self.max_linear_vel} m/s")
        self.get_logger().info(f"  Max angular velocity: {self.max_angular_vel} rad/s")
        self.get_logger().info(f"  Control rate: {control_rate} Hz")

    def cmd_vel_callback(self, msg):
        """Handle velocity command messages"""
        self.current_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        
    def cmd_vel_stamped_callback(self, msg):
        """Handle timestamped velocity command messages"""
        self.current_cmd = msg.twist
        self.last_cmd_time = self.get_clock().now()

    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))

    def control_loop(self):
        """
        Main control loop that converts velocity commands to drone control commands.
        
        For a quadcopter, typically:
        - linear.x -> pitch (forward/backward)
        - linear.y -> roll (left/right)
        - linear.z -> vertical thrust (up/down)
        - angular.z -> yaw (rotation)
        """
        # Check for stale commands (timeout after 0.5 seconds)
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > 0.5:
            # No commands received recently, send stop command
            self.current_cmd = Twist()
        
        # Clamp velocities to limits
        cmd_linear_x = self.clamp(
            self.current_cmd.linear.x,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        cmd_linear_y = self.clamp(
            self.current_cmd.linear.y,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        cmd_linear_z = self.clamp(
            self.current_cmd.linear.z,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        cmd_angular_z = self.clamp(
            self.current_cmd.angular.z,
            -self.max_angular_vel,
            self.max_angular_vel
        )
        
        # Convert to drone control commands
        # Normalize velocities to [-1, 1] range
        pitch = cmd_linear_x / self.max_linear_vel if self.max_linear_vel > 0 else 0.0
        roll = -cmd_linear_y / self.max_linear_vel if self.max_linear_vel > 0 else 0.0
        yaw = cmd_angular_z / self.max_angular_vel if self.max_angular_vel > 0 else 0.0
        
        # Vertical thrust: base thrust + vertical velocity command
        # linear.z > 0 means up, so add to base thrust
        vertical_vel_norm = cmd_linear_z / self.max_linear_vel if self.max_linear_vel > 0 else 0.0
        thrust = self.base_thrust + (vertical_vel_norm * (self.max_thrust - self.base_thrust))
        thrust = self.clamp(thrust, 0.0, self.max_thrust)
        
        # Publish Twist command for compatibility
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = cmd_linear_x
        cmd_vel_msg.linear.y = cmd_linear_y
        cmd_vel_msg.linear.z = cmd_linear_z
        cmd_vel_msg.angular.z = cmd_angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Publish thrust commands [thrust, roll, pitch, yaw]
        # This is a common format for drone control
        thrust_msg = Float64MultiArray()
        thrust_msg.data = [thrust, roll, pitch, yaw]
        self.thrust_pub.publish(thrust_msg)
        
        # Log at lower frequency to avoid spam
        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
            self.get_logger().debug(
                f"Cmd: vx={cmd_linear_x:.2f} vy={cmd_linear_y:.2f} vz={cmd_linear_z:.2f} "
                f"yaw={cmd_angular_z:.2f} | "
                f"Thrust: T={thrust:.2f} R={roll:.2f} P={pitch:.2f} Y={yaw:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


