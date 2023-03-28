#!/usr/bin/env python

import rospy
from ps4_controller_ros import PS4ControllerROS
from std_msgs.msg import Int16

class JoyLispCommandConverter:
    def __init__(self):
        self.lisp_command_pub = rospy.Publisher('lisp_command', Int16, queue_size=1)
        self.lisp_command_msg = Int16()
        self.ps4joy = PS4ControllerROS()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

    def update(self, event):
        self.ps4joy.update()
        # print(self.ps4joy.button_square.wasPressed())
        # print(self.ps4joy.button_square.wasReleased())
        # print(self.ps4joy.button_square.isPressed())
        # print(self.ps4joy.axis_l_h.state)
        # print()
        if self.ps4joy.button_circle.wasPressed() and self.ps4joy.button_right.wasPressed():
            self.lisp_command_msg.data = 1
            self.lisp_command_pub.publish(self.lisp_command_msg)
            print('circle right')

        if self.ps4joy.button_circle.wasPressed() and self.ps4joy.button_up.wasPressed():
            self.lisp_command_msg.data = 2
            self.lisp_command_pub.publish(self.lisp_command_msg)
            print('circle up')

        if self.ps4joy.button_circle.wasPressed() and self.ps4joy.button_down.wasPressed():
            self.lisp_command_msg.data = 3
            self.lisp_command_pub.publish(self.lisp_command_msg)
            print('circle down')

if __name__ == '__main__':
    rospy.init_node('joy_lisp_command_converter')
    node = JoyLispCommandConverter()
    rospy.spin()

