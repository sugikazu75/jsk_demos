#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32

class ApriltagNavigator:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # self.pos_setpoint_msg = Wrench()
        # self.pos_setpoint_pub = rospy.Publisher('pos_setpoint', Wrench, queue_size=1)
        self.x_setpoint_msg = Float32()
        self.y_setpoint_msg = Float32()
        self.z_setpoint_msg = Float32()
        self.x_setpoint_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.y_setpoint_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.z_setpoint_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)

        self.do_demo_flag = False
        self.do_demo_flag_sub = rospy.Subscriber('~do_apriltag_demo', Bool, self.doDemoFlagCallback)

        self.odom = PoseStamped()
        self.odom_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.odomCallback)

        self.world_name = 'world'
        self.tag_list = ['0', '1', '2']
        self.target_tag = self.tag_list[0]
        self.target_tag_sub = rospy.Subscriber('~target_tag', String, self.setTargetTagCallback)

        ################# parameter ######################
        self.offset = Wrench()
        self.offset.force.x = 0.2
        self.offset.force.y = 0.0
        self.offset.force.z = 0.6
        self.offset.torque.z = 0.0
        self.offset_sub = rospy.Subscriber('~pos_setoffset', Wrench, self.setOffsetCallback)

        self.average_num = 20
        ###################################################

        self.verbose = False
        self.trans = None
        self.trans_all = np.zeros((self.average_num, 3))
        self.rot = None
        self.rot_all = np.zeros((self.average_num, 4))
        self.euler = None
        self.index = 0
        self.count = 0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(self.world_name, self.target_tag, rospy.Time(0))
            self.trans_all[self.index] = trans
            self.rot_all[self.index] = rot
            self.trans = np.average(self.trans_all, axis=0)
            self.rot = np.average(self.rot_all, axis=0)
            self.euler = tf.transformations.euler_from_quaternion((self.rot[0], self.rot[1], self.rot[2], self.rot[3]))
            self.count = self.count + 1
            self.index = (self.index + 1) % self.average_num

            self.broadcaster.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 1.57, 1.57), rospy.Time.now(), 'world_0', self.target_tag)
            self.broadcaster.sendTransform((-1.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), 'target', 'world_0')

            if self.verbose and self.count >= self.average_num:
                print('trans={}, euler={}'.format(self.trans, self.euler))

            if self.do_demo_flag:
                print('trans={}, euler={}'.format(self.trans, self.euler))
                # self.pos_setpoint_msg.force.x = self.trans[0] + self.offset.force.x
                # self.pos_setpoint_msg.force.y = self.trans[1] + self.offset.force.y
                # self.pos_setpoint_msg.force.z = self.trans[2] + self.offset.force.z
                # self.pos_setpoint_msg.torque.z = self.offset.torque.z
                # self.pos_setpoint_pub.publish(self.pos_setpoint_msg)
                self.x_setpoint_msg.data = self.trans[0] + self.offset.force.x
                self.y_setpoint_msg.data = self.trans[1] + self.offset.force.y
                self.z_setpoint_msg.data = self.trans[2] + self.offset.force.z
                self.x_setpoint_pub.publish(self.x_setpoint_msg)
                self.y_setpoint_pub.publish(self.y_setpoint_msg)
                self.z_setpoint_pub.publish(self.z_setpoint_msg)
                print('published target point x = {}, y = {}, z = {}'.format(self.x_setpoint_msg.data, self.y_setpoint_msg.data, self.z_setpoint_msg.data))
                self.do_demo_flag = False

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.count = 0
            print(e)

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data
        rospy.loginfo('do demo flag = {}'.format(msg.data))

    def odomCallback(self, msg):
        self.odom = msg

    def setTargetTagCallback(self, msg):
        self.target_tag = msg.data
        rospy.loginfo('target tag = {}'.format(self.target_tag))

    def setOffsetCallback(self, msg):
        self.offset = msg
        rospy.loginfo('set offset: x = {}, y = {}, z = {}, yaw = {}'.format(msg.force.x, msg.force.y, msg.force.z, msg.torque.z))


if __name__ == '__main__':
    rospy.init_node('apriltag_navigator')
    node = ApriltagNavigator()
    rospy.spin()
