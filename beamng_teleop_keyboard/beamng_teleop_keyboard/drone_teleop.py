#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from beamng_msgs.msg import DroneControl

import sys, select, os

if os.name == "nt":
    import msvcrt
else:
    import tty, termios

DRONE_MAX_ASCEND = 1.0
DRONE_MAX_DESCEND = 1.0
DRONE_MAX_PITCH = 1.0
DRONE_MAX_ROLL = 1.0
DRONE_MAX_YAW = 1.0

STEP_SIZE = 0.1

msg = """
Control your drone in BeamNG.Tech simulator !
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : ascend/descend
a/d : roll left/right
q/e : yaw left/right
i/k : pitch forward/backward

t : takeoff/land toggle
m : toggle flight mode
space key, s : hover (stop all movement)

CTRL-C to quit
"""

e = """
Communications Failed 

^^^^^^^^^^
Please revise back your scripts and launch files 
"""


def getKey(settings):
    if os.name == "nt":
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input


class DroneTeleopNode(Node):
    def __init__(self):
        super().__init__("drone_teleop")
        self.pub = self.create_publisher(DroneControl, "/drone_control", 10)

        self.status = 0
        self.ascend = 0.0
        self.descend = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.takeoff_land_state = False
        self.toggle_mode_state = False
        self.last_takeoff_land = False
        self.last_toggle_mode = False
        self.takeoff_land_pending = False  # Flag to send toggle on next message
        self.toggle_mode_pending = False  # Flag to send toggle on next message
        self.takeoff_land_counter = 0  # Counter to send toggle for multiple messages
        self.toggle_mode_counter = 0  # Counter to send toggle for multiple messages

        # Check if stdin is a TTY (interactive terminal)
        if os.name != "nt":
            if sys.stdin.isatty():
                try:
                    self.settings = termios.tcgetattr(sys.stdin)
                except (termios.error, OSError) as e:
                    self.get_logger().error(
                        f"Cannot access terminal. Please run this node in an interactive terminal.\n"
                        f"Error: {e}\n"
                        f"Try running: ros2 run beamng_teleop_keyboard drone_teleop"
                    )
                    sys.exit(1)
            else:
                self.get_logger().error(
                    "stdin is not a TTY. This node requires an interactive terminal.\n"
                    "Please run it directly in a terminal: ros2 run beamng_teleop_keyboard drone_teleop"
                )
                sys.exit(1)
        else:
            self.settings = None

        self.get_logger().info(msg)
        try:
            self.run()
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            control_msg = DroneControl()
            control_msg.ascend = 0.0
            control_msg.descend = 0.0
            control_msg.pitch = 0.0
            control_msg.roll = 0.0
            control_msg.yaw = 0.0
            self.pub.publish(control_msg)

            if os.name != "nt" and hasattr(self, 'settings') and self.settings is not None:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                except (termios.error, OSError):
                    pass  # Ignore errors when restoring terminal settings

    def run(self):
        while True:
            key = getKey(self.settings)
            
            # Vertical controls
            if key == "w":
                self.ascend = constrain(self.ascend + STEP_SIZE, 0.0, DRONE_MAX_ASCEND)
                self.descend = 0.0
                self.status += 1
                self.get_logger().info(f"Ascend: {self.ascend:.2f}")
            elif key == "x":
                self.descend = constrain(self.descend + STEP_SIZE, 0.0, DRONE_MAX_DESCEND)
                self.ascend = 0.0
                self.status += 1
                self.get_logger().info(f"Descend: {self.descend:.2f}")
            
            # Roll controls
            elif key == "a":
                self.roll = constrain(self.roll - STEP_SIZE, -DRONE_MAX_ROLL, DRONE_MAX_ROLL)
                self.status += 1
                self.get_logger().info(f"Roll left: {self.roll:.2f}")
            elif key == "d":
                self.roll = constrain(self.roll + STEP_SIZE, -DRONE_MAX_ROLL, DRONE_MAX_ROLL)
                self.status += 1
                self.get_logger().info(f"Roll right: {self.roll:.2f}")
            
            # Yaw controls
            elif key == "q":
                self.yaw = constrain(self.yaw - STEP_SIZE, -DRONE_MAX_YAW, DRONE_MAX_YAW)
                self.status += 1
                self.get_logger().info(f"Yaw left: {self.yaw:.2f}")
            elif key == "e":
                self.yaw = constrain(self.yaw + STEP_SIZE, -DRONE_MAX_YAW, DRONE_MAX_YAW)
                self.status += 1
                self.get_logger().info(f"Yaw right: {self.yaw:.2f}")
            
            # Pitch controls
            elif key == "i":
                self.pitch = constrain(self.pitch + STEP_SIZE, -DRONE_MAX_PITCH, DRONE_MAX_PITCH)
                self.status += 1
                self.get_logger().info(f"Pitch forward: {self.pitch:.2f}")
            elif key == "k":
                self.pitch = constrain(self.pitch - STEP_SIZE, -DRONE_MAX_PITCH, DRONE_MAX_PITCH)
                self.status += 1
                self.get_logger().info(f"Pitch backward: {self.pitch:.2f}")
            
            # Toggle controls
            elif key == "t":
                self.takeoff_land_pending = True  # Set flag to send toggle on next message
                self.takeoff_land_counter = 3  # Send toggle for 3 messages to ensure it's received
                self.takeoff_land_state = not self.takeoff_land_state
                self.get_logger().info(f"Takeoff/Land toggle: {self.takeoff_land_state}")
            elif key == "m":
                self.toggle_mode_pending = True  # Set flag to send toggle on next message
                self.toggle_mode_counter = 3  # Send toggle for 3 messages to ensure it's received
                self.toggle_mode_state = not self.toggle_mode_state
                self.get_logger().info(f"Flight mode toggle: {self.toggle_mode_state}")
            
            # Stop/hover
            elif key == " " or key == "s":
                self.ascend = 0.0
                self.descend = 0.0
                self.pitch = 0.0
                self.roll = 0.0
                self.yaw = 0.0
                self.get_logger().info("Hover mode - all controls reset")
            else:
                if key == "\x03":
                    break

            if self.status == 20:
                self.get_logger().info(msg)  # msg here refers to the module-level help string
                self.status = 0

            # Publish drone control message
            control_msg = DroneControl()
            control_msg.ascend = self.ascend
            control_msg.descend = self.descend
            control_msg.pitch = self.pitch
            control_msg.roll = self.roll
            control_msg.yaw = self.yaw
            control_msg.yaw_left = 0.0
            control_msg.yaw_right = 0.0
            
            # Handle toggle commands (edge-triggered)
            # Send toggle command for multiple messages to ensure it's received
            if self.takeoff_land_counter > 0:
                control_msg.takeoff_land = True
                self.takeoff_land_counter -= 1
                if self.takeoff_land_counter == 0:
                    self.takeoff_land_pending = False  # Clear flag after all messages sent
                self.get_logger().debug(f"Sending takeoff_land=True in message (counter: {self.takeoff_land_counter})")
            else:
                control_msg.takeoff_land = False
            
            if self.toggle_mode_counter > 0:
                control_msg.toggle_mode = True
                self.toggle_mode_counter -= 1
                if self.toggle_mode_counter == 0:
                    self.toggle_mode_pending = False  # Clear flag after all messages sent
                self.get_logger().debug(f"Sending toggle_mode=True in message (counter: {self.toggle_mode_counter})")
            else:
                control_msg.toggle_mode = False

            self.pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

