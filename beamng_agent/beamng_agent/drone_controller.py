#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from beamng_msgs.msg import DroneControl
from beamng_msgs.srv import SetControlMode
import beamngpy as bngpy
from pathlib import Path

NODE_NAME = "beamng_drone_controller"


class DroneControllerNode(Node):
    """ROS2 node for controlling BeamNG drone vehicle."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        host = self.declare_parameter("host", "172.21.192.1").value
        port = self.declare_parameter("port", 25252).value
        vehicle_id = self.declare_parameter("vehicle_id", "drone").value

        if not vehicle_id:
            self.get_logger().fatal("No Vehicle ID given, shutting down node.")
            sys.exit(1)

        # Connect to BeamNG
        self.game_client = bngpy.BeamNGpy(host, port)

        try:
            self.game_client.open(listen_ip="*", launch=False, deploy=False)
            self.get_logger().info("Successfully connected to BeamNG.tech.")
        except TimeoutError:
            self.get_logger().error(
                "Could not establish game connection, check whether BeamNG.tech is running."
            )
            sys.exit(1)

        # Get vehicle - wait indefinitely (level may still be loading or scenario not started)
        current_vehicles = {}
        attempt = 0
        while True:
            current_vehicles = self.game_client.get_current_vehicles()
            if current_vehicles:
                break
            attempt += 1
            if attempt == 1 or attempt % 5 == 0:
                self.get_logger().info(
                    f"Waiting for vehicles... (attempt {attempt})"
                )
            time.sleep(2)
        # Try by key first, then by vid (BeamNG.drive may use different IDs)
        self.vehicle_client = None
        if vehicle_id in current_vehicles.keys():
            self.vehicle_client = current_vehicles[vehicle_id]
        else:
            # Fallback: match by vehicle vid (e.g. when launched with -vehicle drone)
            for vid, vehicle in current_vehicles.items():
                if hasattr(vehicle, 'vid') and vehicle.vid == vehicle_id:
                    self.vehicle_client = vehicle
                    self.get_logger().info(
                        f"Found vehicle by vid: '{vehicle_id}' (key was '{vid}')"
                    )
                    break
            if self.vehicle_client is None:
                # Fallback: use player vehicle when only one exists
                if len(current_vehicles) == 1:
                    self.vehicle_client = next(iter(current_vehicles.values()))
                    self.get_logger().info(
                        f"Using sole vehicle '{self.vehicle_client.vid}' "
                        f"(requested '{vehicle_id}' not in keys: {list(current_vehicles.keys())})"
                    )
        if self.vehicle_client is None:
            self.get_logger().error(
                f"No vehicle with id '{vehicle_id}' found. "
                f"Available: {list(current_vehicles.keys())} "
                f"(vids: {[getattr(v, 'vid', '?') for v in current_vehicles.values()]})"
            )
            sys.exit(1)
        try:
            self.vehicle_client.connect(self.game_client)
            vid = self.vehicle_client.vid
            self.get_logger().info(
                f"Successfully connected to drone vehicle with id {vid}"
            )
        except TimeoutError:
            self.get_logger().fatal(
                "Could not establish vehicle connection, system exit."
            )
            sys.exit(1)

        # Subscribe to drone control commands
        self.subscription = self.create_subscription(
            DroneControl,
            "/drone_control",
            self.drone_control_callback,
            10,
        )

        # Track previous state for toggle commands
        self.last_takeoff_land = False
        self.last_toggle_mode = False
        self.flight_active = False
        
        # Store the latest control message
        self.latest_control_msg = None
        
        # Create a timer to send commands at regular intervals (20 Hz)
        self.control_timer = self.create_timer(0.05, self.send_control_timer_callback)

        self.get_logger().info(
            f"Drone controller node initialized. Listening on /drone_control topic."
        )
        self.get_logger().info(
            "IMPORTANT: Press 't' in the teleop to toggle takeoff/land before the drone can move!"
        )
        
        # Verify droneInput controller exists once after a short delay
        self._verify_timer = self.create_timer(2.0, self._verify_drone_controller_once)
        
        # Message counter for debugging
        self.message_count = 0
        self.toggle_received_count = 0

    def drone_control_callback(self, msg: DroneControl):
        """Handle incoming drone control commands."""
        try:
            self.message_count += 1
            
            # Log toggle messages for debugging
            if msg.takeoff_land or msg.toggle_mode:
                self.toggle_received_count += 1
                self.get_logger().info(
                    f"[MSG #{self.message_count}] Received control msg: takeoff_land={msg.takeoff_land}, "
                    f"toggle_mode={msg.toggle_mode}, last_takeoff={self.last_takeoff_land}, "
                    f"last_mode={self.last_toggle_mode} (Total toggles received: {self.toggle_received_count})"
                )
            
            # Handle toggle commands (only trigger when going from False to True - edge-triggered)
            if msg.takeoff_land and not self.last_takeoff_land:
                # Only trigger when transitioning from False to True
                self.get_logger().info(f"*** TRIGGERING TAKEOFF/LAND TOGGLE ***")
                self.get_logger().info(f"Received takeoff_land toggle: {msg.takeoff_land} (was: {self.last_takeoff_land})")
                self._toggle_takeoff_land()
            self.last_takeoff_land = msg.takeoff_land

            if msg.toggle_mode and not self.last_toggle_mode:
                # Only trigger when transitioning from False to True
                self.get_logger().info(f"*** TRIGGERING FLIGHT MODE TOGGLE ***")
                self.get_logger().info(f"Received toggle_mode: {msg.toggle_mode} (was: {self.last_toggle_mode})")
                self._toggle_flight_mode()
            self.last_toggle_mode = msg.toggle_mode

            # Store the latest control message (will be sent by timer)
            self.latest_control_msg = msg
            
            # Periodic status (every 100 messages)
            if self.message_count % 100 == 0:
                self.get_logger().info(
                    f"Status: Processed {self.message_count} messages, "
                    f"received {self.toggle_received_count} toggle commands, "
                    f"flight_active={self.flight_active}"
                )

        except Exception as e:
            self.get_logger().error(f"Error processing drone control: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _verify_drone_controller_once(self):
        """Verify droneInput controller exists (runs once at startup)."""
        self._verify_timer.cancel()
        try:
            check_cmd = "local ctrl = controller.getControllerSafe('droneInput'); if ctrl then log('I', 'DRONE_CTRL', 'droneInput controller FOUND') else log('E', 'DRONE_CTRL', 'droneInput controller NOT FOUND') end"
            self.vehicle_client.queue_lua_command(check_cmd)
            self.vehicle_client.poll_sensors()
            self.get_logger().info("Drone ready. Run 'ros2 run beamng_teleop_keyboard drone_teleop' in another terminal to control.")
        except Exception as e:
            self.get_logger().warn(f"Could not verify droneInput controller: {e}")
            self.get_logger().warn("The drone may not have the droneInput controller loaded!")
    
    def send_control_timer_callback(self):
        """Timer callback to send control commands at regular intervals."""
        if self.latest_control_msg is not None:
            self._send_drone_inputs(self.latest_control_msg)

    def _toggle_takeoff_land(self):
        """Toggle takeoff/land state."""
        try:
            self.get_logger().info(f"=== TOGGLE TAKEOFF/LAND ===")
            self.get_logger().info(f"Vehicle ID: {self.vehicle_client.vid}")
            
            # Try multiple Lua command formats
            commands_to_try = [
                "controller.getControllerSafe('droneInput').toggleTakeOffLand()",
                "if controller.getControllerSafe('droneInput') then controller.getControllerSafe('droneInput').toggleTakeOffLand() end",
                "local ctrl = controller.getControllerSafe('droneInput'); if ctrl then ctrl.toggleTakeOffLand() end"
            ]
            
            command_executed = False
            for i, lua_command in enumerate(commands_to_try):
                try:
                    self.get_logger().info(f"Trying command format {i+1}: {lua_command}")
                    self.vehicle_client.queue_lua_command(lua_command)
                    self.get_logger().info("Lua command queued successfully")
                    
                    # Execute immediately to ensure the command is processed
                    try:
                        self.vehicle_client.poll_sensors()
                        self.get_logger().info("Polled sensors to execute command")
                        command_executed = True
                        break
                    except Exception as e1:
                        self.get_logger().warn(f"poll_sensors failed: {e1}, trying step()")
                        try:
                            self.vehicle_client.step(1)
                            self.get_logger().info("Stepped simulation to execute command")
                            command_executed = True
                            break
                        except Exception as e2:
                            self.get_logger().warn(f"step() also failed: {e2}")
                            continue
                except Exception as e:
                    self.get_logger().warn(f"Command format {i+1} failed: {e}")
                    continue
            
            if not command_executed:
                self.get_logger().error("All command formats failed! The droneInput controller may not be loaded.")
                self.get_logger().error("Make sure the drone vehicle has 'droneInput' controller in its JBeam file or options.")
            
            self.flight_active = not self.flight_active
            self.get_logger().info(
                f"Takeoff/Land toggled. Flight active: {self.flight_active}"
            )
            self.get_logger().info("=== TOGGLE COMPLETE ===")
        except Exception as e:
            self.get_logger().error(f"Error toggling takeoff/land: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _toggle_flight_mode(self):
        """Toggle between Crawl and Fixed Wing flight modes."""
        try:
            # Call the toggleFlightMode function in the drone controller
            lua_command = "controller.getControllerSafe('droneInput').toggleFlightMode()"
            self.vehicle_client.queue_lua_command(lua_command)
            self.get_logger().info("Flight mode toggled")
        except Exception as e:
            self.get_logger().error(f"Error toggling flight mode: {str(e)}")

    def _send_drone_inputs(self, msg: DroneControl):
        """Send continuous control inputs to the drone."""
        try:
            # Send input events to the drone controller
            # The drone controller expects input events in the format:
            # input.event('input_name', value, filter_type)

            # Build all commands first, then send them together
            commands = []

            # Vertical controls
            if msg.ascend > 0.0:
                commands.append(f"input.event('ascend', {msg.ascend}, 'gamepad')")
            elif msg.descend > 0.0:
                commands.append(f"input.event('descend', {msg.descend}, 'gamepad')")
            else:
                # Send zero for both when neither is active
                commands.append("input.event('ascend', 0.0, 'gamepad')")
                commands.append("input.event('descend', 0.0, 'gamepad')")

            # Orientation controls (always send, even if 0.0, to maintain control)
            pitch_value = -msg.pitch if abs(msg.pitch) > 0.001 else 0.0  # Negated to match drone controller
            commands.append(f"input.event('pitch', {pitch_value}, 'gamepad')")

            roll_value = msg.roll if abs(msg.roll) > 0.001 else 0.0
            commands.append(f"input.event('roll', {roll_value}, 'gamepad')")

            # Yaw control (can use either centered yaw or left/right buttons)
            if abs(msg.yaw) > 0.001:
                yaw_value = -msg.yaw  # Negated to match drone controller
                commands.append(f"input.event('yaw', {yaw_value}, 'gamepad')")
            elif msg.yaw_left > 0.0:
                commands.append(f"input.event('yaw_l', {msg.yaw_left}, 'gamepad')")
            elif msg.yaw_right > 0.0:
                commands.append(f"input.event('yaw_r', {msg.yaw_right}, 'gamepad')")
            else:
                commands.append("input.event('yaw', 0.0, 'gamepad')")
                commands.append("input.event('yaw_l', 0.0, 'gamepad')")
                commands.append("input.event('yaw_r', 0.0, 'gamepad')")

            # Send all commands and ensure they're executed
            for cmd in commands:
                self.vehicle_client.queue_lua_command(cmd)
            
            # Poll sensors or step to ensure queued commands are executed
            try:
                self.vehicle_client.poll_sensors()
            except Exception:
                try:
                    # Alternative: step the simulation to execute commands
                    self.vehicle_client.step(1)
                except Exception:
                    pass  # Ignore errors if methods aren't available

            # Logging disabled - was flooding terminal at ~20 Hz

        except Exception as e:
            self.get_logger().error(f"Error sending drone inputs: {str(e)}")

    def destroy_node(self):
        """Clean up on node destruction."""
        try:
            if hasattr(self, 'vehicle_client'):
                self.vehicle_client.disconnect()
            if hasattr(self, 'game_client'):
                self.game_client.close()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DroneControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
