#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

class mocapTransformer:
    def __init__(self):
        self.mocap_pose_sub = rospy.Subscriber("birotor/mocap/pose", PoseStamped, self.mocapCallback)

        self.br = tf.TransformBroadcaster()

    def mocapCallback(self, msg):
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), rospy.Time.now(), 'flight_unit', 'world')


if __name__ == "__main__":
    rospy.init_node("transformer_node")
    node = mocapTransformer()
    rospy.spin()
