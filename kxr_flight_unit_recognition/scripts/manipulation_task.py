#!/usr/bin/env python

import rospy
import smach
import smach_ros
from kxr_interface import kxrInterface
import time

class BaseState(smach.State):
    def __init__(self, robot, outcomes=[]):
        smach.State.__init__(self, outcomes)

        self.robot = robot

class Approach(BaseState):
    def __init__(self, robot, outcomes=['sucsessed']):
        BaseState.__init__(self, robot, outcomes=outcomes)

    def execute(self, userdata):
        time.sleep(0.5)
        return 'sucsessed'

class Wait(BaseState):
    def __init__(self, robot, outcomes=['sucsessed']):
        BaseState.__init__(self, robot, outcomes=outcomes)

    def execute(self, userdata):
        time.sleep(0.5)
        return 'sucsessed'


if __name__ == '__main__':
    rospy.init_node('manipulation_task_node')
    robot = kxrInterface()

    sm_top = smach.StateMachine(outcomes=['sucsessed'])
    with sm_top:
        smach.StateMachine.add('APPROACH', Approach(robot), transitions={'sucsessed':'WAIT'})
        smach.StateMachine.add('WAIT', Wait(robot), transitions={'sucsessed':'APPROACH'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
