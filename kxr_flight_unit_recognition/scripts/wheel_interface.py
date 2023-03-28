#!/usr/bin/env python

import rospy
import tf
from smach import State, StateMachine
import smach_ros
import time
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Bool, Float32

def isConverged(value, thresh):
    return abs(value) < thresh

class flightToWheel(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)
        self.go_pos_z_msg = Float32()
        self.z_offset_pub = rospy.Publisher('pid/setoffset/z', Float32, queue_size=1)
        self.z_offset_msg = Float32()
        self.z_i_control_pub = rospy.Publisher('pid/i_control/z' Bool, queue_size=1)
        self.z_i_control_msg = Bool()

        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.odom = PoseStamped()

        self.rate = rospy.Rate(10)

        #############
        self.wheel_mode_height = 0.5  # TODO: reference rosbag
        self.height_thresh = 0.05
        self.wheel_mode_z_offset = 6.0
        #############

    def execute(self, userdata):
        self.go_pos_z_msg.data = self.wheel_mode_height
        self.go_pos_z_pub.publish(self.go_pos_z_msg)

        while(True):
            if not isConverged(self.odom.pose.position.z - self.wheel_mode_height, self.height_thresh):
                pass
            else:
                self.z_offset_msg.data = self.wheel_mode_z_offset
                self.z_offset_pub.publish(self.z_offset_msg)
                self.z_i_control_msg.data = False
                self.z_i_control_pub.publish(self.z_i_control_msg)
                return 'done'

            self.rate.sleep()

    def mocapCallback(self, msg):
        self.odom = msg

class wheelToFlight(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)
        self.go_pos_z_msg = Float32()
        self.z_i_control_pub = rospy.Publisher('pid/i_control/z' Bool, queue_size=1)
        self.z_i_control_msg = Bool()

        ##################
        self.takeoff_height = 1.0
        ##################

    def execute(self, userdata):
        self.go_pos_z_msg.data = self.takeoff_height
        self.go_pos_z_pub.publish(self.go_pos_z_msg)
        self.z_i_control_msg.data = True
        self.z_i_control_pub.publish(self.z_i_control_msg)

        return 'done'

