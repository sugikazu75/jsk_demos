#!/usr/bin/env python

import rospy
import tf
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from kxr_mujoco_robot_control.msg import PositionControlInput6D

class CircleRun:
    def __init__(self):
        self.go_pos_x_msg = Float32()
        self.go_pos_y_msg = Float32()
        self.go_pos_yaw_msg = Float32()
        self.pose_control_input_6d_msg = PositionControlInput6D()

        ###################### real robot ###########################
        # self.odom = PoseStamped()  # real robot
        # self.odom_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)  #real robot
        # self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)  # real robot
        # self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)  # real robot
        # self.go_pos_yaw_pub = rospy.Publisher('go_pos/yaw', Float32, queue_size=1)  # real robot
        #############################################################

        ##################### simulation ############################
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber('mujoco_sim_ros/odom', Odometry, self.odomCallback)  # simulation
        self.go_pos_x_pub = rospy.Publisher('controller/x/setpoint', Float32, queue_size=1)  # simulation
        self.go_pos_y_pub = rospy.Publisher('controller/y/setpoint', Float32, queue_size=1)  # simulation
        self.go_pos_yaw_pub = rospy.Publisher('controller/yaw/setpoint', Float32, queue_size=1)  # simulation
        self.pose_control_input_6d_pub = rospy.Publisher('pose_control_input', PositionControlInput6D, queue_size=1)
        #############################################################

        self.timer10 = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

        ################ set for your environment ###################
        self.center_x = 0.0 # [m]
        self.center_y = 0.0 # [m]
        self.radius = 0.8   # [m]
        self.split_count_max = 36
        self.next_step_distance_thresh = 0.1
        self.next_step_angle_thresh = 0.05
        #############################################################

        self.center_xy = np.array((self.center_x, self.center_y))
        self.cur_xy = np.zeros(2)
        self.split_count = 0
        self.unit_vector = np.array((self.radius, 0.0))
        self.quaternion = Quaternion()
        self.target_xy = None
        self.target_yaw = None

    def timerCallback(self, event):
        self.rpy = tf.transformations.euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        self.target_yaw = 2.0 * np.pi * self.split_count / self.split_count_max
        self.target_xy = self.center_xy + self.radius * np.array((np.cos(self.target_yaw), np.sin(self.target_yaw)))

        self.target_yaw = self.target_yaw + np.pi / 2
        while self.target_yaw > np.pi:
            self.target_yaw = self.target_yaw - 2.0 * np.pi

        self.go_pos_x_msg.data = self.target_xy[0]
        self.go_pos_y_msg.data = self.target_xy[1]
        self.go_pos_yaw_msg.data = self.target_yaw
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        self.go_pos_y_pub.publish(self.go_pos_y_msg)
        self.go_pos_yaw_pub.publish(self.go_pos_yaw_msg)
        self.pose_control_input_6d_msg.x = self.target_xy[0]
        self.pose_control_input_6d_msg.y = self.target_xy[1]
        self.pose_control_input_6d_msg.z = 0.8
        self.pose_control_input_6d_msg.yaw = self.target_yaw
        self.pose_control_input_6d_pub.publish(self.pose_control_input_6d_msg)

        print('target=[{}, {}], target_theta={}'.format(self.target_xy[0], self.target_xy[1], self.target_yaw))
        print('curpos=[{}, {}], yaw={}'.format(self.cur_xy[0], self.cur_xy[1], self.rpy[2]))
        print('distance = {}, yaw_err={}'.format(np.linalg.norm(self.target_xy - self.cur_xy), self.target_yaw - self.rpy[2]))
        print('{}/{}'.format(self.split_count, self.split_count_max))

        if np.linalg.norm(self.cur_xy - self.target_xy) < self.next_step_distance_thresh and self.rpy[2] - self.target_yaw < self.next_step_angle_thresh:
            self.split_count = (self.split_count + 1) % self.split_count_max

    def mocapCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.orientation
        self.cur_xy[0] = self.odom.pose.position.x  # real robot
        self.cur_xy[1] = self.odom.pose.position.y  # real robot

    def odomCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.pose.orientation
        self.cur_xy[0] = self.odom.pose.pose.position.x  # simulation
        self.cur_xy[1] = self.odom.pose.pose.position.y  # simulation


if __name__ == '__main__':
    rospy.init_node('circle_run')
    node = CircleRun()
    rospy.spin()

