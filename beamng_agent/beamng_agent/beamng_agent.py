#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import beamngpy as bngpy
from beamng_msgs.msg import VehicleControl, DroneControl
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

    def __init__(self, node):
        self.node = node
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
        self.brake_state = "False"
        self.vehicle_state = "initial"

    def brake_callback(self, brake_state):
        self.brake_state = brake_state.data

    def cmd_vel_callback(self, cmd_vel):
        self.control_linear_vel_new = cmd_vel.linear.x
        self.control_angular_vel_new = cmd_vel.angular.z

    def twist_msg_comparison(self):
        if (
            self.control_linear_vel_old != self.control_linear_vel_new
            or self.control_angular_vel_old != self.control_angular_vel_new
        ):
            self.convert_cmd_to_bng(
                self.control_linear_vel_new, self.control_angular_vel_new
            )
            self.control_linear_vel_old = self.control_linear_vel_new
            self.control_angular_vel_old = self.control_angular_vel_new
            return True
        return False

    def convert_cmd_to_bng(self, control_linear_vel, control_angular_vel):
        self.Vehicle_Control_msg = VehicleControl()

        if self.brake_state == "True":
            self.Vehicle_Control_msg.throttle = 0.0
            self.Vehicle_Control_msg.steering = 0.0
            self.Vehicle_Control_msg.brake = 1.0  # Change to float
            self.Vehicle_Control_msg.parkingbrake = 1.0  # Change to float
            self.Vehicle_Control_msg.clutch = 1.0  # Change to float
            self.Vehicle_Control_msg.gear = 1
            self.vehicle_state = "stop"
        else:
            if control_linear_vel < 0.0:
                self.Vehicle_Control_msg.throttle = abs(control_linear_vel)
                self.Vehicle_Control_msg.steering = control_angular_vel
                self.Vehicle_Control_msg.brake = 0.0  # Change to float
                self.Vehicle_Control_msg.parkingbrake = 0.0  # Change to float
                self.Vehicle_Control_msg.clutch = 0.0  # Change to float
                self.Vehicle_Control_msg.gear = -1
                self.vehicle_state = "reverse"
            else:
                self.Vehicle_Control_msg.throttle = abs(control_linear_vel)
                self.Vehicle_Control_msg.steering = control_angular_vel
                self.Vehicle_Control_msg.brake = 0.0  # Change to float
                self.Vehicle_Control_msg.parkingbrake = 0.0  # Change to float
                self.Vehicle_Control_msg.clutch = 0.0  # Change to float
                self.Vehicle_Control_msg.gear = 2
                self.vehicle_state = "go"

    def main_loop(self):
        if self.twist_msg_comparison():
            self.BNG_pub.publish(self.Vehicle_Control_msg)


def main(args=None):
    rclpy.init(args=args)

    node = VehicleControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
