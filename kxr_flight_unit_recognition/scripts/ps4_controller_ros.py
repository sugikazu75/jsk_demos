#!/usr/bin/env python

from ps4_controller import PS4Controller
import rospy
from sensor_msgs.msg import Joy

def deadzone(value, thresh):
    if abs(value) < thresh:
        return 0.0
    else:
        return value

class PS4ControllerROS(PS4Controller, object):
    def __init__(self):
        super(PS4ControllerROS, self).__init__()
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.joy_state = Joy()
        self.joy_stick_deadzone = 0.1

    def joyCallback(self, msg):
        self.joy_state = msg

    def update(self):
        self.button_square(self.joy_state.buttons[0])
        self.button_cross(self.joy_state.buttons[1])
        self.button_circle(self.joy_state.buttons[2])
        self.button_triangle(self.joy_state.buttons[3])
        self.button_l1(self.joy_state.buttons[4])
        self.button_r1(self.joy_state.buttons[5])
        self.button_l2(self.joy_state.buttons[6])
        self.button_r2(self.joy_state.buttons[7])
        self.button_share(self.joy_state.buttons[8])
        self.button_options(self.joy_state.buttons[9])
        self.button_l3(self.joy_state.buttons[10])
        self.button_r3(self.joy_state.buttons[11])
        self.button_ps4(self.joy_state.buttons[12])
        self.button_tpad(self.joy_state.buttons[13])

        self.axis_l_h(deadzone(self.joy_state.axes[0], self.joy_stick_deadzone))
        self.axis_l_v(deadzone(self.joy_state.axes[1], self.joy_stick_deadzone))
        self.axis_r_h(deadzone(self.joy_state.axes[2], self.joy_stick_deadzone))
        self.axis_l2(deadzone(self.joy_state.axes[3], self.joy_stick_deadzone))
        self.axis_r2(deadzone(self.joy_state.axes[4], self.joy_stick_deadzone))
        self.axis_r_v(deadzone(self.joy_state.axes[5], self.joy_stick_deadzone))

        if self.joy_state.axes[10] == 1:
            self.button_up(1)
        else:
            self.button_up(0)

        if self.joy_state.axes[10] == -1:
            self.button_down(1)
        else:
            self.button_down(0)

        if self.joy_state.axes[9] == 1:
            self.button_left(1)
        else:
            self.button_left(0)

        if self.joy_state.axes[9] == -1:
            self.button_right(1)
        else:
            self.button_right(0)

if __name__ == '__main__':
    rospy.init_node('ps4_controller_ros')
    node = PS4ControllerROS()
    rospy.spin()
