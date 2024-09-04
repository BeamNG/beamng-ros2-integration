#!/usr/bin/env python3

# Copyright (c) 2011, Abdul Rahman , Inc.
# All rights reserved.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

import json
import subprocess
import os
import beamngpy as bngpy
from beamng_msgs.msg import VehicleControl

AGV_MAX_LIN_VEL = 1.0
AGV_MAX_ANG_VEL = 1.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05

msg = """
Control your vehicle in BeamNG.Tech simulator !
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed 

^^^^^^^^^^
Please revise back your scripts and launch files 
"""

def getKey(settings):
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)  # vel change frequency
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return f"currently:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -AGV_MAX_LIN_VEL, AGV_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -AGV_MAX_ANG_VEL, AGV_MAX_ANG_VEL)
    return vel


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.brakepub = self.create_publisher(String, 'brake', 10)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.brake_state = "False"

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None

        self.get_logger().info(msg)
        try:
            self.run()
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            self.pub.publish(twist)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        while True:
            key = getKey(self.settings)
            if key == 'w':
                self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
                self.status += 1
                self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
                self.brake_state = "False"
            elif key == 'x':
                self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
                self.status += 1
                self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
                self.brake_state = "False"
            elif key == 'd':
                self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
                self.status += 1
                self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
                self.brake_state = "False"
            elif key == 'a':
                self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
                self.status += 1
                self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
                self.brake_state = "False"
            elif key == ' ' or key == 's':
                self.target_linear_vel = 0.0
                self.control_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.control_angular_vel = 0.0
                self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
                self.brake_state = "True"
            else:
                if key == '\x03':
                    break

            if self.status == 20:
                self.get_logger().info(msg)
                self.status = 0

            twist = Twist()
            self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = self.control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.control_angular_vel

            self.pub.publish(twist)
            self.brakepub.publish(String(data=self.brake_state))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
