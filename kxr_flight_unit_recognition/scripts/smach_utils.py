#!/usr/bin/env python

import rospy
import time
from smach import State
from std_msgs.msg import Bool

class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.do_demo_flag_sub = rospy.Subscriber('wait_command', Bool, self.doDemoFlagCallback)
        self.do_demo_flag = False

    def execute(self, userdata):
        self.do_demo_flag = False
        while not self.do_demo_flag:
            time.sleep(1.0)
            print('waiting flag')
        return 'done'

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data

