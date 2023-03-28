#! /usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros

import attach_task_smach
import locomotion_task_smach
import navigation_smach
import smach_utils

if __name__ == '__main__':
    node = rospy.init_node('transport_task')
    sm_top = StateMachine(outcomes=['success'])

    with sm_top:
        StateMachine.add('WAIT_ROTATE_YAW_COMMAND', smach_utils.Wait(), transitions={'done':'ROTATE_YAW'})
        StateMachine.add('ROTATE_YAW', locomotion_task_smach.RotateYaw(), transitions={'done':'WAIT_GO_POS_COMMAND'})
        StateMachine.add('WAIT_GO_POS_COMMAND', smach_utils.Wait(), transitions={'done':'GO_POS'})
        StateMachine.add('GO_POS', locomotion_task_smach.GoPos(), transitions={'done':'Z_I_CONTROL_ENABLE'})
        StateMachine.add('Z_I_CONTROL_ENABLE', navigation_smach.zIControlEnable(), transitions={'done':'WAIT_ATTACH_TASK'})
        StateMachine.add('WAIT_ATTACH_TASK', smach_utils.Wait(), transitions={'done':'ATTACH_TASK'})
        sm_attach = StateMachine(outcomes=['done_attach_task'])
        with sm_attach:
            StateMachine.add('APPROACH_WALL', attach_task_smach.Approach(), transitions={'done':'LEAVE'})
            StateMachine.add('LEAVE', attach_task_smach.Leave(), transitions={'done':'done_attach_task'})

        StateMachine.add('ATTACH_TASK', sm_attach, transitions={'done_attach_task':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
