#! /usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros

import color_extraction
import smach_utils

if __name__ == '__main__':
    node = rospy.init_node('pickup_task')
    sm_top = StateMachine(outcomes=['success'])
    with sm_top:
        StateMachine.add('WAIT1', smach_utils.Wait(), transitions={'done':'GRID_SEARCH'})
        StateMachine.add('GRID_SEARCH', color_extraction.gridSearch(), transitions={'found':'WAIT2', 'not_found':'GRID_SEARCH'})
        StateMachine.add('WAIT2', smach_utils.Wait(), transitions={'done':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

