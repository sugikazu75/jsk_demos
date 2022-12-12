#!/usr/bin/env python

import rospy
import numpy as np
import time
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Int16
import smach_utils

def isConverged(value, thresh):
    if abs(value) < thresh:
        return True
    else:
        return False

class Approach(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)

        ############## parameter ##################
        self.safety_x = 1.1
        self.approach_done_thresh = 0.1
        self.approach_done_count_max = 3
        self.approach_x_safety_max = 1.2
        self.approach_x_step = 0.3
        ###########################################

        self.count = 0
        self.pre_xy = np.zeros(2)
        self.cur_xy = np.zeros(2)
        self.tmp_xy = np.zeros(2)
        self.approach_done_count = 0

    def execute(self, userdata):
        self.odom = None
        self.count = 0
        self.pre_xy = np.zeros(2)
        self.cur_xy = np.zeros(2)
        self.tmp_xy = np.zeros(2)
        self.approach_done_count = 0

        while self.odom is None:
            pass
        while True:
            time.sleep(0.01)

            self.go_pos_x_msg.data = min(self.cur_xy[0] + self.approach_x_step, self.safety_x)
            self.go_pos_x_pub.publish(self.go_pos_x_msg)

            ################ 1 Hz process ####################
            if self.count % 100 == 0:
                self.pre_xy = np.copy(self.tmp_xy)
                self.tmp_xy = np.copy(self.cur_xy)
                if np.linalg.norm(self.pre_xy - self.cur_xy) < self.approach_done_thresh:
                    self.approach_done_count = self.approach_done_count + 1
                else:
                    self.approach_done_count = 0
                print('approach count = {}'.format(self.approach_done_count))
                print('pre=[{}, {}], cur=[{}, {}]'.format(self.pre_xy[0], self.pre_xy[1], self.cur_xy[0], self.cur_xy[1]))
                self.count = 0
            ##################################################

            self.count = self.count + 1

            if self.approach_done_count >= self.approach_done_count_max:
                return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y


class Leave(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.gripper_msg = Int16()
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)
        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)

        ################## parameter ########################
        self.leave_x_distance = 0.3
        #####################################################

        self.cur_xy = np.zeros(2)

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass
        self.gripper_msg.data = 3
        self.gripper_pub.publish(self.gripper_msg)

        self.go_pos_x_msg.data = self.cur_xy[0] - self.leave_x_distance
        self.go_pos_x_pub.publish(self.go_pos_x_msg)

        return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y

if __name__ == '__main__':
    rospy.init_node('attact_task')
    sm_top = StateMachine(outcomes=['success'])

    with sm_top:
        StateMachine.add('WAIT', smach_utils.Wait(), transitions={'done':'APPROACH'})
        StateMachine.add('APPROACH', Approach(), transitions={'done':'LEAVE'})
        StateMachine.add('LEAVE', Leave(), transitions={'done':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
