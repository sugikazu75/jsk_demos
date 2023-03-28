#!/usr/bin/env python

import rospy
import tf
import time
import numpy as np
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class mocapTransformer:
    def __init__(self):
        self.mocap_pose_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)

        self.br = tf.TransformBroadcaster()

    def mocapCallback(self, msg):
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), rospy.Time.now(), 'flight_unit', 'world')

if __name__ == '__main__':
    rospy.init_node('integrated_experiment_node')
    mocap_transformer = mocapTransformer()
    rospy.spin()
