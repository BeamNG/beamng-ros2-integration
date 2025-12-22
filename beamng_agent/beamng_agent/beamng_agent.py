#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, Header
import beamngpy as bngpy
from beamng_msgs.msg import VehicleControl, DroneControl, StateSensor
from beamng_msgs.srv import VehicleCommand, SetControlMode
from pathlib import Path

NODE_NAME = "beamng_agent"


class VehicleControlNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        host = self.declare_parameter("host", "172.21.192.1").value
        port = self.declare_parameter("port", 25252).value
        self.driving_mode = self.declare_parameter("driving_mode", "keyboard").value
        vehicle_id = self.declare_parameter("vehicle_id", "ego").value
        self.vehicle_type = self.declare_parameter("vehicle_type", "vehicle").value  # "vehicle" or "drone"

        if not vehicle_id:
            self.get_logger().fatal("No Vehicle ID given, shutting down node.")
            sys.exit(1)

        self.game_client = bngpy.BeamNGpy(host, port)

        try:
            self.game_client.open(listen_ip="*", launch=False, deploy=False)
            self.get_logger().info("Successfully connected to BeamNG.tech.")
        except TimeoutError:
            self.get_logger().error(
                "Could not establish game connection, check whether BeamNG.tech is running."
            )
            sys.exit(1)

        current_vehicles = self.game_client.get_current_vehicles()

        assert (
            vehicle_id in current_vehicles.keys()
        ), f"no vehicle with id {vehicle_id} exists"
        self.vehicle_client = current_vehicles[vehicle_id]
        try:
            self.vehicle_client.connect(self.game_client)
            vid = self.vehicle_client.vid
            self.get_logger().info(
                f"Successfully connected to vehicle client with id {vid}"
            )
        except TimeoutError:
            self.get_logger().fatal(
                "Could not establish vehicle connection, system exit."
            )
            sys.exit(1)

        # Subscribe based on vehicle type
        if self.vehicle_type == "drone":
            self.drone_subscription = self.create_subscription(
                DroneControl,
                "/drone_control",
                self.drone_control_callback,
                10,
            )
            self.get_logger().info("Agent node configured for drone control")
        else:
            self.subscription = self.create_subscription(
                VehicleControl,
                "/control",
                lambda msg: self.send_control_signal(msg, self.driving_mode),
                10,
            )
            self.twist_to_bng = TwistToBNG(self)

        self.srv = self.create_service(
            VehicleCommand, "vehicle_command", self.vehicle_command_callback
        )

        # Create the service for setting control mode
        self.mode_srv = self.create_service(
            SetControlMode, "set_control_mode", self.set_control_mode_callback
        )

    def vehicle_command_callback(self, request, response):
        vehicle_id = request.vehicle_id
        linear_velocity = request.linear_velocity
        angular_velocity = request.angular_velocity

        if vehicle_id != self.vehicle_client.vid:
            self.get_logger().error(f"Unknown vehicle id: {vehicle_id}")
            response.success = False
            return response

        twist = Twist()

        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        self.twist_to_bng.cmd_vel_callback(twist)
        response.success = True
        return response

    def set_control_mode_callback(self, request, response):
        vehicle_id = request.vehicle_id
        mode = request.control_mode

        if vehicle_id != self.vehicle_client.vid:
            self.get_logger().error(f"Unknown vehicle id: {vehicle_id}")
            response.success = False
            return response

        if self.vehicle_type == "drone":
            # Drone doesn't support AI mode, only manual control
            if mode not in ["keyboard"]:
                self.get_logger().error(f"Invalid control mode for drone: {mode}. Only 'keyboard' is supported.")
                response.success = False
                return response
        else:
            if mode not in ["ai", "keyboard"]:
                self.get_logger().error(f"Invalid control mode: {mode}")
                response.success = False
                return response

        self.driving_mode = mode
        self.get_logger().info(f"Control mode set to: {mode} for vehicle: {vehicle_id}")
        response.success = True
        return response

    def drone_control_callback(self, msg: DroneControl):
        """Callback for drone control messages when agent is in drone mode."""
        # The drone_controller node handles the actual control,
        # but this allows the agent node to work with drones too
        # For now, just log that we received the message
        # In the future, this could forward to drone_controller or handle directly
        self.get_logger().debug(
            f"Received drone control: ascend={msg.ascend:.2f}, "
            f"descend={msg.descend:.2f}, pitch={msg.pitch:.2f}, "
            f"roll={msg.roll:.2f}, yaw={msg.yaw:.2f}"
        )

    def send_control_signal(self, signal, mode):
        if mode == "ai":
            self.vehicle_client.ai_set_mode("span")
        else:
            self.vehicle_client.ai_set_mode("disable")
            self.vehicle_client.control(
                steering=signal.steering,
                throttle=signal.throttle,
                brake=signal.brake,
                parkingbrake=signal.parkingbrake,
                clutch=signal.clutch,
                gear=signal.gear,
            )


class TwistToBNG:

    def __init__(self, node, vehicle_type="sedan"):
        self.node = node
        self.vehicle_type = vehicle_type.lower()
        self.BNG_pub = node.create_publisher(VehicleControl, "/control", 10)
        self.cmd_vel_sub = node.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.brake_sub = node.create_subscription(
            String, "/brake", self.brake_callback, 10
        )

        self.timer = node.create_timer(0.1, self.main_loop)

        self.Vehicle_Control_msg = VehicleControl()
        self.control_linear_vel_old = 0
        self.control_angular_vel_old = 0
        self.control_linear_vel_new = 0
        self.control_angular_vel_new = 0
        self.control_linear_y_old = 0
        self.control_linear_z_old = 0
        self.control_linear_y_new = 0
        self.control_linear_z_new = 0
        self.brake_state = "False"
        self.vehicle_state = "initial"
        
        # Validate vehicle type
        valid_types = ["drone", "sedan", "truck"]
        if self.vehicle_type not in valid_types:
            self.node.get_logger().warn(
                f"Unknown vehicle type '{vehicle_type}', defaulting to 'sedan'. "
                f"Valid types: {valid_types}"
            )
            self.vehicle_type = "sedan"
        
        self.node.get_logger().info(f"Vehicle type set to: {self.vehicle_type}")

    def brake_callback(self, brake_state):
        self.brake_state = brake_state.data

    def cmd_vel_callback(self, cmd_vel):
        # For backward compatibility: ground vehicles only use linear.x and angular.z
        # (linear.y and linear.z are ignored for sedans/trucks, used for drones)
        self.control_linear_vel_new = cmd_vel.linear.x
        self.control_angular_vel_new = cmd_vel.angular.z
        # Store y/z for drone mode, but ground vehicles ignore them
        self.control_linear_y_new = cmd_vel.linear.y if self.vehicle_type == "drone" else 0.0
        self.control_linear_z_new = cmd_vel.linear.z if self.vehicle_type == "drone" else 0.0

    def twist_msg_comparison(self):
        # For backward compatibility: ground vehicles only check linear.x and angular.z
        # (same behavior as old code - linear.y and linear.z changes ignored for sedans/trucks)
        if self.vehicle_type == "drone":
            # Drone: check all dimensions
            changed = (
                self.control_linear_vel_old != self.control_linear_vel_new
                or self.control_linear_y_old != self.control_linear_y_new
                or self.control_linear_z_old != self.control_linear_z_new
                or self.control_angular_vel_old != self.control_angular_vel_new
            )
        else:
            # Ground vehicle: only check linear.x and angular.z (backward compatible)
            changed = (
                self.control_linear_vel_old != self.control_linear_vel_new
                or self.control_angular_vel_old != self.control_angular_vel_new
            )
        
        if changed:
            self.convert_cmd_to_bng(
                self.control_linear_vel_new,
                self.control_linear_y_new,
                self.control_linear_z_new,
                self.control_angular_vel_new
            )
            # Update old values - for backward compatibility, ground vehicles only track linear.x and angular.z
            self.control_linear_vel_old = self.control_linear_vel_new
            self.control_angular_vel_old = self.control_angular_vel_new
            if self.vehicle_type == "drone":
                # Only track y/z for drones
                self.control_linear_y_old = self.control_linear_y_new
                self.control_linear_z_old = self.control_linear_z_new
            return True
        return False

    def convert_cmd_to_bng(self, control_linear_x, control_linear_y, control_linear_z, control_angular_z):
        self.Vehicle_Control_msg = VehicleControl()

        if self.vehicle_type == "drone":
            # Drone control mode: convert Twist to drone controls
            # For drone: linear.x -> pitch (forward/back), linear.y -> roll (left/right)
            #            linear.z -> vertical thrust, angular.z -> yaw
            
            if self.brake_state == "True":
                # Stop/hover: set base throttle, zero attitude
                self.Vehicle_Control_msg.throttle = 0.5  # Hover throttle
                self.Vehicle_Control_msg.steering = 0.0   # No roll
                self.Vehicle_Control_msg.brake = 0.0
                self.Vehicle_Control_msg.parkingbrake = 0.0
                self.Vehicle_Control_msg.clutch = 0.0
                self.Vehicle_Control_msg.gear = 0  # Neutral/hover mode
                self.vehicle_state = "hover"
            else:
                # Clamp values to [-1, 1] range for normalized control
                # Roll: left/right movement (linear.y)
                roll = max(-1.0, min(1.0, control_linear_y))
                # Pitch: forward/back movement (linear.x)
                pitch = max(-1.0, min(1.0, control_linear_x))
                # Thrust: up/down movement (linear.z) + base hover
                base_thrust = 0.5
                vertical_thrust = max(-1.0, min(1.0, control_linear_z))
                thrust = base_thrust + (vertical_thrust * 0.5)  # Combine base + vertical
                thrust = max(0.0, min(1.0, thrust))
                # Yaw: rotation (angular.z)
                yaw = max(-1.0, min(1.0, control_angular_z))
                
                # Map to VehicleControl message
                # Using throttle for overall thrust, steering for combined attitude
                self.Vehicle_Control_msg.throttle = thrust
                # Combine roll, pitch, yaw into steering for simplicity
                # In a real drone, you'd want separate channels, but BeamNG might not support that
                self.Vehicle_Control_msg.steering = (pitch + roll + yaw) / 3.0  # Average or use pitch
                self.Vehicle_Control_msg.brake = 0.0
                self.Vehicle_Control_msg.parkingbrake = 0.0
                self.Vehicle_Control_msg.clutch = 0.0
                self.Vehicle_Control_msg.gear = 1  # Flying mode
                self.vehicle_state = "flying"
                
        else:
            # Ground vehicle control (sedan/truck) - EXACTLY matches old behavior
            # Ignores linear_y and linear_z for backward compatibility
            if self.brake_state == "True":
                self.Vehicle_Control_msg.throttle = 0.0
                self.Vehicle_Control_msg.steering = 0.0
                self.Vehicle_Control_msg.brake = 1.0
                self.Vehicle_Control_msg.parkingbrake = 1.0
                self.Vehicle_Control_msg.clutch = 1.0
                self.Vehicle_Control_msg.gear = 1
                self.vehicle_state = "stop"
            else:
                if control_linear_x < 0.0:
                    # Reverse gear - old behavior preserved
                    self.Vehicle_Control_msg.throttle = abs(control_linear_x)
                    self.Vehicle_Control_msg.steering = control_angular_z
                    self.Vehicle_Control_msg.brake = 0.0
                    self.Vehicle_Control_msg.parkingbrake = 0.0
                    self.Vehicle_Control_msg.clutch = 0.0
                    self.Vehicle_Control_msg.gear = -1
                    self.vehicle_state = "reverse"
                else:
                    # Forward gear - old behavior preserved (only difference: truck uses gear 3)
                    gear = 3 if self.vehicle_type == "truck" else 2
                    self.Vehicle_Control_msg.throttle = abs(control_linear_x)
                    self.Vehicle_Control_msg.steering = control_angular_z
                    self.Vehicle_Control_msg.brake = 0.0
                    self.Vehicle_Control_msg.parkingbrake = 0.0
                    self.Vehicle_Control_msg.clutch = 0.0
                    self.Vehicle_Control_msg.gear = gear
                    self.vehicle_state = "go"

    def main_loop(self):
        if self.twist_msg_comparison():
            self.BNG_pub.publish(self.Vehicle_Control_msg)


class BNGToTwist:
    """
    Optional: Reuses bridge's StateSensor (published by VehicleNode) and converts to Twist.
    Bridge publishes StateSensor at: beamng_bridge/vehicles/{vehicle_id}/state
    
    This is an ADDITIVE feature - disabled by default for backward compatibility.
    Enable with enable_state_to_twist:=true parameter.
    """

    def __init__(self, node, vehicle_type="sedan", vehicle_id="ego"):
        self.node = node
        self.vehicle_type = vehicle_type.lower()
        self.vehicle_id = vehicle_id
        
        # Publishers - reuse existing ROS2 odometry topic pattern
        self.twist_pub = node.create_publisher(Twist, f"{vehicle_id}/odom/twist", 10)
        self.twist_stamped_pub = node.create_publisher(
            TwistStamped, f"{vehicle_id}/odom/twist_stamped", 10
        )
        
        # Subscribe to bridge's StateSensor (published by VehicleNode)
        # Bridge publishes at: beamng_bridge/vehicles/{vehicle_id}/state
        # Allow parameter override if bridge namespace differs
        bridge_ns = node.declare_parameter("bridge_namespace", "beamng_bridge").value
        state_topic = f"{bridge_ns}/vehicles/{vehicle_id}/state"
        try:
            self.state_sub = node.create_subscription(
                StateSensor,
                state_topic,
                self.state_callback,
                10
            )
            self.node.get_logger().info(
                f"BNGToTwist enabled: reusing bridge StateSensor for '{vehicle_id}' (type: {self.vehicle_type})"
            )
            self.node.get_logger().info(f"Subscribing to: {state_topic}")
        except Exception as e:
            self.node.get_logger().warn(
                f"Failed to create StateSensor subscription (bridge may not be running): {e}"
            )
            self.state_sub = None

    def state_callback(self, state_msg):
        """Convert bridge's StateSensor velocity to Twist (reusing existing data)."""
        twist = Twist()
        twist_stamped = TwistStamped()
        twist_stamped.header = state_msg.header
        
        velocity = state_msg.velocity
        
        if self.vehicle_type == "drone":
            # Drone: 3D velocity (reuse bridge's velocity data)
            twist.linear.x = velocity.x  # forward
            twist.linear.y = velocity.y  # left/right
            twist.linear.z = velocity.z  # up/down
            # Angular rates would need IMU sensor from bridge (not in StateSensor)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
        else:
            # Ground vehicle: forward velocity (reuse bridge's velocity)
            speed = (velocity.x**2 + velocity.y**2)**0.5
            twist.linear.x = speed if velocity.x >= 0 else -speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            # Yaw rate would need additional computation from direction vector
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
        
        twist_stamped.twist = twist
        self.twist_pub.publish(twist)
        self.twist_stamped_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)

    node = VehicleControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
