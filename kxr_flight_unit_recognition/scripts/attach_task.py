#!/usr/bin/env python

import rospy
import numpy as np
import tf
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

def isConverged(value, thresh):
    if abs(value) < thresh:
        return True
    else:
        return False

class AttachTask:
    def __init__(self):
        ################# real robot ####################
        self.odom = PoseStamped()  # real robot
        self.odom_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)  #real robot
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)  # real robot
        ##########################################################

        ##################### simulation ############################
        # self.odom = Odometry()
        # self.odom_sub = rospy.Subscriber('mujoco_sim_ros/odom', Odometry, self.odomCallback)  # simulation
        # self.go_pos_x_pub = rospy.Publisher('controller/x/setpoint', Float32, queue_size=1)  # simulation
        ############################################################

        ############ set for your environment ###################
        self.touching_count_max = 3  # [sec]
        self.finish_task_count_max = 200  # * 0.01[sec]
        self.x_step_distance = 0.2       # [m]
        self.x_wall_thresh = 0.7         # [m]
        self.touching_thresh = 0.04      # [m]
        self.wall_leave_distance = 0.6   # [m]
        self.moving_detect_thresh = 0.05        # [m]
        #########################################################

        ############## set for your robot #######################
        self.pitch_safety_limit = 1.1
        #########################################################

        self.go_pos_x_msg = Float32()
        self.twist = TwistStamped()
        self.twist_sub = rospy.Subscriber('odometry/twist', TwistStamped, self.twistCallback)
        self.timer100 = rospy.Timer(rospy.Duration(0.01), self.timer100Callback)
        self.timer1 = rospy.Timer(rospy.Duration(1.0), self.timer1Callback)
        self.do_demo_flag_sub = rospy.Subscriber('do_attach_demo', Bool, self.doDemoFlagCallback)
        self.do_demo_flag = False

        self.initial_position = None
        self.initial_x = None
        self.gripper_msg = Int16()
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)
        self.pre_x = 0
        self.cur_x = 0
        self.tmp_x = 0
        self.quaternion = Quaternion()
        self.rpy = None
        self.touching_count = 0
        self.touching_x = 0
        self.finish_task = False
        self.count = 0
        self.safety_flag = True


    def timer100Callback(self, event):
        self.count = self.count + 1
        self.rpy = tf.transformations.euler_from_quaternion([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        if(self.rpy[1] > self.pitch_safety_limit):
            self.safety_flag = False
            print(self.safety_flag)
        else:
            self.safety_flag = True

        # print('pitch = {}'.format(self.rpy[1]))

        if not self.do_demo_flag:
            pass
        else:
            if self.safety_flag:
                self.go_pos_x_msg.data = self.cur_x + self.x_step_distance
                self.go_pos_x_pub.publish(self.go_pos_x_msg)
            else:
                # todo: dynamic pitch gain tuning
                self.go_pos_x_msg.data = self.cur_x - self.x_step_distance
                self.go_pos_x_pub.publish(self.go_pos_x_msg)

            if self.touching_count > self.touching_count_max and self.cur_x > self.x_wall_thresh:
                self.gripper_msg.data = 3
                self.gripper_pub.publish(self.gripper_msg)
                self.go_pos_x_msg.data = self.cur_x
                self.go_pos_x_pub.publish(self.go_pos_x_msg)
                self.touching_x = self.cur_x
                self.finish_task = True
                self.do_demo_flag = False
                self.count = 0
                print('release')

        if self.finish_task and self.count > self.finish_task_count_max:
            self.go_pos_x_msg.data = self.touching_x - self.wall_leave_distance
            self.go_pos_x_pub.publish(self.go_pos_x_msg)
            self.finish_task = False
            print('finish task. go to {}'.format(self.go_pos_x_msg.data))
            self.initial_position = None

    def timer1Callback(self, event):
        if self.initial_position is None:
            print('not do demo')
        else:
            if self.cur_x - self.initial_x > self.moving_detect_thresh:
                self.pre_x = self.tmp_x
                self.tmp_x = self.cur_x
                if isConverged(self.pre_x - self.cur_x, self.touching_thresh):
                    self.touching_count = self.touching_count + 1
                else:
                    self.touching_count = 0
                print('(1 sec ago, now) = ({}, {})'.format(self.pre_x, self.cur_x))
                print('touching count = {}'.format(self.touching_count))
            else:
                print('not moving start')

    def mocapCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.orientation
        self.cur_x = self.odom.pose.position.x  # real robot

    def odomCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.pose.orientation
        self.cur_x = self.odom.pose.pose.position.x  # simulation

    def twistCallback(self, msg):
        self.twist = msg
        self.gyro_pitch = self.twist.twist.angular.y

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data
        self.initial_position = self.odom

        ################## real robot #####################################
        self.pre_x = self.initial_position.pose.position.x  # real robot
        self.initial_x = self.initial_position.pose.position.x
        ###################################################################

        ################## simulation #####################################
        # self.pre_x = self.initial_position.pose.pose.position.x  # simulation
        # self.initial_x = self.initial_position.pose.pose.position.x
        ###################################################################

        self.tmp_x = self.pre_x
        rospy.loginfo('do demo flag = {}'.format(msg.data))
        rospy.loginfo(self.initial_position)


if __name__ == '__main__':
    rospy.init_node('attach_task', anonymous=False)
    node = AttachTask()
    rospy.spin()
