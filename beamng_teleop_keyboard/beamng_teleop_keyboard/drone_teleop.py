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

# Keys that affect drone control; only these are shown in the status line
ACCEPTED_KEYS = frozenset("wxadqeki tms")

msg = """
Control your drone in BeamNG.Tech simulator !
---------------------------

Setup (Terminal 1 - launch):
  cd ~/ros2_ws
  source install/setup.bash
  ros2 launch beamng_bringup beamng_drone_all.launch.py host:=<IP> port:=25252

Setup (Terminal 2 - this teleop):
  cd ~/ros2_ws
  source install/setup.bash
  ros2 run beamng_teleop_keyboard drone_teleop

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
m : toggle flight mode (Crawl ↔ Fixed Wing) — status line shows current mode
space key, s : hover (stop all movement)

CTRL-C to quit

"""

e = """
Communications Failed 

^^^^^^^^^^
Please revise back your scripts and launch files 
"""


def getKey(fd, raw_mode=False):
    """Read single key. If raw_mode=True, fd is already in raw mode (no echo)."""
    if os.name == "nt":
        return msvcrt.getch()

    if raw_mode:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        return sys.stdin.read(1) if rlist else ""

    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
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

        # Track flight mode for display (BeamNG default is Crawl)
        self._flight_mode_name = "Crawl"
        # Track last key for display
        self._last_key = "-"

        # Check if stdin is a TTY (interactive terminal)
        if os.name != "nt":
            if sys.stdin.isatty():
                try:
                    self.term_fd = sys.stdin.fileno()
                    self.settings = termios.tcgetattr(self.term_fd)
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
            self.term_fd = None  # Not used on Windows

        print(msg, end="", flush=True)
        try:
            if os.name != "nt":
                tty.setraw(self.term_fd)  # Keep raw for whole session - no key echo
            self.run()
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            print()  # Newline after status line
            control_msg = DroneControl()
            control_msg.ascend = 0.0
            control_msg.descend = 0.0
            control_msg.pitch = 0.0
            control_msg.roll = 0.0
            control_msg.yaw = 0.0
            self.pub.publish(control_msg)

            if os.name != "nt" and hasattr(self, 'settings') and self.settings is not None:
                try:
                    termios.tcsetattr(self.term_fd, termios.TCSADRAIN, self.settings)
                except (termios.error, OSError):
                    pass  # Ignore errors when restoring terminal settings

    def _key_display(self, k):
        """Return readable key name for display (accepted drone keys only)."""
        if k not in ACCEPTED_KEYS:
            return None  # Don't update display for unaccepted keys
        if k == " ":
            return "Space"
        return k

    def _status_str(self):
        """Build status string (fixed width for clean overwrite)."""
        return (
            f"  Key:{self._last_key:>6}  |  Reading: Asc:{self.ascend:+5.2f} Desc:{self.descend:+5.2f} "
            f"Pit:{self.pitch:+5.2f} Rol:{self.roll:+5.2f} Yaw:{self.yaw:+5.2f}  |  Mode:{self._flight_mode_name}"
        )

    def _print_status_if_changed(self):
        """Print status only when it changed (avoids terminal flood)."""
        new_status = self._status_str()
        if new_status != getattr(self, "_last_status", None):
            self._last_status = new_status
            STATUS_WIDTH = 95
            padding = " " * max(0, STATUS_WIDTH - len(new_status))
            print(f"\r{new_status}{padding}", end="", flush=True)

    def run(self):
        self._last_status = None
        self._print_status_if_changed()
        while True:
            key = getKey(self.term_fd, raw_mode=(os.name != "nt"))
            if key:
                display_key = self._key_display(key)
                if display_key is not None:
                    self._last_key = display_key

            # Vertical controls
            if key == "w":
                self.ascend = constrain(self.ascend + STEP_SIZE, 0.0, DRONE_MAX_ASCEND)
                self.descend = 0.0
            elif key == "x":
                self.descend = constrain(self.descend + STEP_SIZE, 0.0, DRONE_MAX_DESCEND)
                self.ascend = 0.0
            
            # Roll controls
            elif key == "a":
                self.roll = constrain(self.roll - STEP_SIZE, -DRONE_MAX_ROLL, DRONE_MAX_ROLL)
            elif key == "d":
                self.roll = constrain(self.roll + STEP_SIZE, -DRONE_MAX_ROLL, DRONE_MAX_ROLL)
            
            # Yaw controls
            elif key == "q":
                self.yaw = constrain(self.yaw - STEP_SIZE, -DRONE_MAX_YAW, DRONE_MAX_YAW)
            elif key == "e":
                self.yaw = constrain(self.yaw + STEP_SIZE, -DRONE_MAX_YAW, DRONE_MAX_YAW)
            
            # Pitch controls
            elif key == "i":
                self.pitch = constrain(self.pitch + STEP_SIZE, -DRONE_MAX_PITCH, DRONE_MAX_PITCH)
            elif key == "k":
                self.pitch = constrain(self.pitch - STEP_SIZE, -DRONE_MAX_PITCH, DRONE_MAX_PITCH)

            # Toggle controls
            elif key == "t":
                self.takeoff_land_pending = True
                self.takeoff_land_counter = 3
                self.takeoff_land_state = not self.takeoff_land_state
            elif key == "m":
                starting_new_toggle = self.toggle_mode_counter == 0
                self.toggle_mode_pending = True
                self.toggle_mode_counter = 3
                self.toggle_mode_state = not self.toggle_mode_state
                if starting_new_toggle:
                    self._flight_mode_name = "Fixed Wing" if self._flight_mode_name == "Crawl" else "Crawl"
            
            # Stop/hover
            elif key == " " or key == "s":
                self.ascend = 0.0
                self.descend = 0.0
                self.pitch = 0.0
                self.roll = 0.0
                self.yaw = 0.0
            else:
                if key == "\x03":
                    break

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
            self._print_status_if_changed()


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

