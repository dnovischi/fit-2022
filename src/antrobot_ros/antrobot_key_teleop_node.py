#!/usr/bin/env python3
# Copyright (C) 2021 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import select
import sys
import rospy
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios


class KeyboardTeleoperation:
    def __init__(
            self, key_mapping: dict = None,
            max_linear_velocity: float = 0.3,
            max_angular_velocity: float = 3.0,
            linear_step_size: float = 0.025,
            angular_step_size: float = 0.25
    ):
        self.key_mapping = key_mapping
        if key_mapping is None:
            self.key_mapping = {
                "forward": ['w'],
                "backward": ['s'],
                "left": ['a'],
                "right": ['d'],
                "stop": [' ']
            }

        self.usage_msg = (
                "\n"
                "Control Ant Robot!\n"
                "------------------\n"
                "Moving around:\n\n"
                "        " + str('forward') + "\n\n"
                "   " + str('left') + "         " + str('right') + "\n\n"
                "        " + str('backward') + "\n"
                "         " + str('stop') + "\n\n"

                "Keyboard mappings:\n\n"
                "\t" + str('forward') + ": " + str(self.key_mapping['forward']) + " - increase linear velocity \n"
                "\t" + str('backward') + ": " + str(self.key_mapping['backward']) + " - decrease linear velocity \n"
                "\t" + str('left') + ": " + str(self.key_mapping['left']) + " - increase angular velocity \n"
                "\t" + str('right') + ": " + str(self.key_mapping['right']) + " - decrease angular velocity\n"
                "\t" + str('stop') + ": " + str(self.key_mapping['stop']) + " - force stop\n"
                "\tCTRL-C to quit\n"
        )

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.linear_step_size = linear_step_size
        self.angular_step_size = angular_step_size
        self.ref_linear_velocity = 0.0
        self.ref_angular_velocity = 0.0


    # TODO 1: Implement a command step that moves the output towards
    #         a diseried target with a specific slope.
    # Hint: Use the min(), max() functions to ensure that the output will be within the bounds of the target value
    @classmethod
    def __cmd_step__(cls, output, target, slope):
        return output


    # TODO 2: Implement a command step that moves the output towards
    #         a diseried target with a specific slope.
    @classmethod
    def __threshold__(cls, target, low_limit, high_limit):
        return target

    @classmethod
    def __get_key__(cls):
        if os.name == 'nt':
            if sys.version_info[0] >= 3:
                return msvcrt.getch().decode()
            else:
                return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key_pressed = sys.stdin.read(1)
        else:
            key_pressed = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key_pressed

    def __velocities_string__(self):
        return "currently:\tlinear velocity %s\t angular velocity %s " % \
               (self.ref_linear_velocity, self.ref_angular_velocity)

    def __set_ref_linear_velocity__(self, target_velocity):
        # TODO 3: Set the reference for the linear velocity "self.ref_linear_velocity"
        #         with the thresholded target linear velocity "target_velocity
        pass

    def __set_ref_angular_velocity__(self, target_velocity):
        # TODO 4: Set the reference for the angular velocity "self.ref_angular_velocity"
        #         with the thresholded target angular velocity "target_velocity"
        pass

    def run(self):
        rospy.init_node('antrobot_teleop_key')
        pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        linear_velocity_ctrl = 0
        angular_velocity_ctrl = 0
        cmd_vel_msg = Twist()
        rate = rospy.Rate(1000)
        print(self.usage_msg)
        try:
            while not rospy.is_shutdown():
                key_press = KeyboardTeleoperation.__get_key__()
                # TODO 5: Increase or dicrease the robot command refereces with linear step
                #         "self.linear_step_size" or angular step "self.angular_step_size" according to the
                #         to the pressed keys: forward/backward, left/right (5a - 5d).
                if key_press in self.key_mapping['forward']:
                    # TODO 5a
                    print(self.__velocities_string__())
                elif key_press in self.key_mapping['backward']:
                    # TODO 5b
                    print(self.__velocities_string__())
                elif key_press in self.key_mapping['left']:
                    # TODO 5c
                    print(self.__velocities_string__())
                elif key_press in self.key_mapping['right']:
                    # TODO 5d
                    print(self.__velocities_string__())
                elif key_press in self.key_mapping['stop']:
                    self.ref_linear_velocity = 0
                    self.ref_angular_velocity = 0
                    linear_velocity_ctrl = 0
                    angular_velocity_ctrl = 0
                    print(self.__velocities_string__())
                elif key_press == '\x03':
                    break

                linear_velocity_ctrl = self.__cmd_step__(
                    linear_velocity_ctrl, self.ref_linear_velocity, self.linear_step_size / 2.0
                )

                angular_velocity_ctrl = self.__cmd_step__(
                    angular_velocity_ctrl, self.ref_angular_velocity, self.angular_step_size / 2.0
                )

                cmd_vel_msg.linear.x = linear_velocity_ctrl
                cmd_vel_msg.linear.y = 0
                cmd_vel_msg.linear.z = 0
                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = angular_velocity_ctrl
                pub_cmd_vel.publish(cmd_vel_msg)
                rate.sleep()
        except:
            rospy.logerr("KeyboardTeleoperation: com error!\n")
        finally:
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0
            pub_cmd_vel.publish(cmd_vel_msg)

if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    ant_teleoperation = KeyboardTeleoperation()
    ant_teleoperation.run()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
