#!/usr/bin/env python

import rospy
import time
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Bool

class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.do_demo_flag_sub = rospy.Subscriber('wait_command', Bool, self.doDemoFlagCallback)
        self.do_demo_flag = False
        self.count = 0

    def execute(self, userdata):
        self.do_demo_flag = False

        while not self.do_demo_flag:
            time.sleep(1.0)
            print('waiting flag')
        return 'done'

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data

if __name__ == '__main__':
    node = rospy.init_node('wait')
    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('WAIT', Wait(), transitions={'done':'WAIT'})

    sis = smach_ros.IntrospectionServer('smach_viewer', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
