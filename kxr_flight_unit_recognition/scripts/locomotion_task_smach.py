#! /usr/bin/env python

import rospy
import numpy as np
import tf
from time import sleep
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

def isConverged(value, thresh):
    return abs(value) < thresh

class GoUp(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)
        self.go_pos_z_msg = Float32()

        ################### set for your environment ####################
        self.go_up_target_z = 1.0
        self.go_up_height_thresh = 0.05
        #################################################################

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass
        self.go_pos_z_msg.data = self.go_up_target_z
        self.go_pos_z_pub.publish(self.go_pos_z_msg)

        while not isConverged(self.odom.pose.position.z - self.go_up_target_z, self.go_up_height_thresh):
            sleep(1.0)
            pass

        return 'done'

    def mocapCallback(self, msg):
        self.odom = msg


class GoPos(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_x_msg = Float32()
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()

        ################### set for your environment ####################
        self.go_pos_target_x = 0.6
        self.go_pos_target_y = 0.0
        self.go_pos_distance_thresh = 0.1
        #################################################################

        self.cur_xy = np.zeros(2)
        self.target_xy = np.array((self.go_pos_target_x, self.go_pos_target_y))

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass

        self.go_pos_x_msg.data = self.go_pos_target_x
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        self.go_pos_y_msg.data = self.go_pos_target_y
        self.go_pos_y_pub.publish(self.go_pos_y_msg)
        print('go to [{}, {}]'.format(self.go_pos_target_x, self.go_pos_target_y))

        while not isConverged(np.linalg.norm(self.cur_xy - self.target_xy), self.go_pos_distance_thresh):
            sleep(1.0)
            pass

        return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y


class RotateYaw(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_yaw_pub = rospy.Publisher('go_pos/yaw', Float32, queue_size=1)
        self.go_pos_yaw_msg = Float32()

        self.cur_yaw = None
        ################### set for your environment ####################
        self.rotate_yaw_target_angle = 0.0
        self.rotate_yaw_angle_thresh = 0.05
        #################################################################

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass

        self.go_pos_yaw_msg.data = self.rotate_yaw_target_angle
        self.go_pos_yaw_pub.publish(self.go_pos_yaw_msg)
        print('published yaw setpoint {}'.format(self.rotate_yaw_target_angle))

        while not isConverged(self.rotate_yaw_target_angle - self.cur_yaw, self.rotate_yaw_angle_thresh):
            sleep(1.0)
            print('wait for yaw rotation convergence')

        print('yaw rotation is completed')
        return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_yaw = tf.transformations.euler_from_quaternion((self.odom.pose.orientation.x, self.odom.pose.orientation.y, self.odom.pose.orientation.z, self.odom.pose.orientation.w))[2]



