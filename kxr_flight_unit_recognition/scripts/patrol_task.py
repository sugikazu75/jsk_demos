#! /usr/bin/env python

import rospy
import numpy as np
import tf
from time import sleep
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes = ['wait', 'move', 'search', 'done'], input_keys=['target_pos'], output_keys=['target_pos'])
        self.move_sub = rospy.Subscriber('move_flag', Vector3, self.targetCallback)
        self.target = Vector3()
        self.flag = False

    def targetCallback(self, msg):
        self.target.x = msg.x
        self.target.y = msg.y
        self.target.z = msg.z
        print(msg)
        self.flag = True

    def execute(self, userdata):
        sleep(0.01)
        if self.flag:
            userdata.target_pos.x = self.target.x
            userdata.target_pos.y = self.target.y
            userdata.target_pos.z = self.target.z
            self.flag = False
            return 'move'
        else:
            return 'wait'

class Search(State):
    def __init__(self):
        State.__init__(self, outcomes = ['searching', 'done'])
        self.yaw_setpoint_msg = Float32()
        self.yaw_setpoint_pub = rospy.Publisher('/go_pos_interpolation/yaw', Float32, queue_size=1)
        self.odom = PoseStamped()
        self.mocap_sub = rospy.Subscriber('birotor/mocap/pose', PoseStamped, self.mocapCallback)
        self.euler = None
        self.yaw_angle = None

    def mocapCallback(self, msg):
        self.odom = msg
        self.euler = tf.transformations.euler_from_quaternion(self.odom.pose.orientation.x, self.odom.pose.orientation.y, self.odom.pose.orientation.z, self.odom.pose.orientation.w)
        self.yaw_angle = self.euler[2]

    def execute(self, userdata):
        if self.yaw_angle is None:
            return 'searching'
        else:
            self.yaw_setpoint_msg.data = self.yaw_angle + 1.57
            self.yaw_setpoint_pub.publish(self.yaw_setpoint_msg)
            return 'done'

class Move(State):
    def __init__(self):
        State.__init__(self, outcomes = ['done', 'moving'], input_keys=['target_pos'])
        self.odom = PoseStamped()
        self.mocap_sub = rospy.Subscriber('birotor/mocap/pose', PoseStamped, self.mocapCallback)
        self.target = np.zeros(3)

    def mocapCallback(self, msg):
        self.odom = msg

    def execute(self, userdata):
        sleep(1)
        self.target[0] = userdata.target_pos.x
        self.target[1] = userdata.target_pos.y
        self.target[2] = userdata.target_pos.z
        if np.linalg.norm((self.odom.pose.position.x, self.odom.pose.position.y, self.odom.pose.position.z) - self.target) < 0.1:
            print('distance = {}'.format(np.linalg.norm(self.target - (self.odom.pose.position.x, self.odom.pose.position.y, self.odom.pose.position.z))))
            return 'done'
        else:
            print('distance = {}'.format(np.linalg.norm(self.target - (self.odom.pose.position.x, self.odom.pose.position.y, self.odom.pose.position.z))))
            return 'moving'


if __name__ == '__main__':
    node = rospy.init_node('patrol_smach')
    sm = StateMachine(outcomes=['success'])
    sm.userdata.target_pos = Vector3()

    with sm:
        StateMachine.add('WAIT', Wait(), transitions={'wait':'WAIT', 'move':'MOVE', 'search' : 'SEARCH', 'done' : 'success'}, remapping={'target_pos':'target_pos'})
        StateMachine.add('MOVE', Move(), transitions={'moving' : 'MOVE', 'done' : 'WAIT'})
        StateMachine.add('SEARCH', Search(), transitions={'searching' : 'SEARCH', 'done' : 'WAIT'})
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
