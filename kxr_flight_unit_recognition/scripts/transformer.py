#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class mocapTransformer:
    def __init__(self):
        self.mocap_pose_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)

        self.br = tf.TransformBroadcaster()

    def mocapCallback(self, msg):
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), rospy.Time.now(), 'flight_unit', 'world')

class cameraTransformer:
    def __init__(self):
        self.head_neck_y_sub = rospy.Subscriber('head_neck_y', Float32, self.headNeckYawCallback)
        self.head_neck_p_sub = rospy.Subscriber('head_neck_p', Float32, self.headNeckPitchCallback)
        self.head_neck_y_angle = 0.0
        self.head_neck_p_angle = 0.0

        self.neck_y_to_neck_p = rospy.get_param('~neck_yaw_to_neck_pitch')
        self.neck_pitch_to_camera_center = rospy.get_param('~neck_pitch_to_camera_center')

        self.br = tf.TransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, event):
        self.br.sendTransform(self.neck_y_to_neck_p, tf.transformations.quaternion_from_euler(0, 0, self.head_neck_y_angle), rospy.Time.now(), 'head_neck_p', 'head_neck_y')
        self.br.sendTransform(self.neck_pitch_to_camera_center, tf.transformations.quaternion_from_euler(0, self.head_neck_p_angle, 0), rospy.Time.now(), 'camera_center', 'head_neck_p')

    def headNeckYawCallback(self, msg):
        self.head_neck_y_angle = msg.data

    def headNeckPitchCallback(self, msg):
        self.head_neck_p_angle = msg.data


if __name__ == "__main__":
    rospy.init_node("transformer_node")
    mocap_transformer = mocapTransformer()
    camera_transformer = cameraTransformer()
    rospy.spin()
