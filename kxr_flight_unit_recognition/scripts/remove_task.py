#!/usr/bin/env python

import rospy
import numpy as np
import tf
from time import sleep
from smach import State, StateMachine
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Bool

def clamp(minimam, value, maximum):
    return min(max(minimam, value), maximum)

class RemoveTask:
    def __init__(self):
        ###################### real robot ###########################
        self.odom = PoseStamped()  # real robot
        self.odom_sub = rospy.Subscriber('birotor/mocap/pose', PoseStamped, self.mocapCallback)  #real robot
        self.go_pos_x_pub = rospy.Publisher('birotor/go_pos/x', Float32, queue_size=1)  # real robot
        self.go_pos_y_pub = rospy.Publisher('birotor/go_pos/y', Float32, queue_size=1)  # real robot
        ##########################################3##################

        ##################### simulation ############################
        # self.odom = Odometry()
        # self.odom_sub = rospy.Subscriber('mujoco_sim_ros/odom', Odometry, self.odomCallback)  # simulation
        # self.go_pos_x_pub = rospy.Publisher('controller/x/setpoint', Float32, queue_size=1)  # simulation
        # self.go_pos_y_pub = rospy.Publisher('controller/y/setpoint', Float32, queue_size=1)  # simulation
        #############################################################

        ################## set for your environment #################
        self.object_x = 0.9                     # [m] in world coords
        self.object_y = 0.0                     # [m] in world coords
        self.object_z = 0.9                     # [m] in world coords
        self.reaching_theta_thresh = 0.4        # [rad]
        self.x_step_distance = 0.2              # [m]
        self.grasp_distance_thresh = 0.1        # [m]
        self.touching_thresh = 0.1             # [m]
        self.recovering_remove_distance = 0.35  # [m]
        self.wall_leave_distance = 0.6          # [m]
        self.target_axis = np.array((1, 0))     # unit vector parallel to target moving trajectory
       #############################################################

        self.go_pos_x_msg = Float32()
        self.go_pos_y_msg = Float32()
        self.gripper_msg = Int16()
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)
        self.do_demo_flag_sub = rospy.Subscriber('do_remove_demo', Bool, self.doDemoFlagCallback)
        self.do_demo_flag = False
        self.demo_state_msg = Int16()
        self.demo_state_pub = rospy.Publisher('remove_demo_state', Int16, queue_size=1)

        self.pre_xy = np.zeros(2)
        self.tmp_xy = None
        self.cur_xy = np.zeros(2)
        self.target_xy = np.zeros(2)
        self.object_xy = np.array((self.object_x, self.object_y))
        self.trajectory = self.cur_xy - self.pre_xy

        self.theta = 0
        self.theta_count = 0
        self.touching_count = 0
        self.distance = 0
        self.recovered = False
        self.count = 0
        self.demo_state = 0
        ## demo state##
        ## 0: not do demo
        ## 1: waiting
        ## 2: approaching
        ## 3: contacting
        ## 4: recovering
        ## 5: removing

        self.timer100 = rospy.Timer(rospy.Duration(0.01), self.timer100Callback)
        self.timer1 = rospy.Timer(rospy.Duration(2.0), self.timer1Callback)

    def timer100Callback(self, event):
        self.count = self.count + 1
        self.demo_state_msg.data = self.demo_state
        self.demo_state_pub.publish(self.demo_state_msg)

        if not self.do_demo_flag:
            return

        self.updateTargetXY()

        if self.demo_state == 1:
            if np.linalg.norm(self.target_xy - self.cur_xy) < 0.1:
                if not self.recovered:
                    self.count = 0
                self.recovered = True
                if self.recovered and self.count >= 200:
                    print('recovered. reapproach')
                    self.demo_state = 2
                    self.recovered = False
                    return

        if self.demo_state == 2:
            self.distance = np.linalg.norm(self.cur_xy - self.object_xy)
            # if self.distance < self.grasp_distance_thresh and self.theta_count >= 3:
            if self.distance < self.grasp_distance_thresh:
                self.gripper_msg.data = 4
                self.gripper_pub.publish(self.gripper_msg)
                self.demo_state = 3
                self.touching_count = 0
                self.count = 0
                print('grasp')
                return

        if self.demo_state == 3:
            if self.count >= 600:
                if self.touching_count >= 2:
                    self.demo_state = 5
                    print('remove')
                    return
                else:
                    self.demo_state = 4
                    print('recover')
                    return

        if self.demo_state == 4:
            self.gripper_msg.data = 3
            self.gripper_pub.publish(self.gripper_msg)
            self.demo_state = 1
            self.recovered = False
            return

        if self.demo_state == 5:
            self.demo_state = 0
            self.do_demo_flag = False
            return

    def timer1Callback(self, event):
        if not self.do_demo_flag:
            print('not do demo')
            return

        self.update()

        self.updateTouchingState()

        print('cur = {}, target = {}'.format(self.cur_xy, self.target_xy))
        print('theta = {}, diff = {}, error = {}'.format(self.theta, np.linalg.norm(self.cur_xy- self.pre_xy), np.linalg.norm(self.target_xy - self.cur_xy)))
        print('theta count ={}, touching_count = {}'.format(self.theta_count, self.touching_count))
        print('')

    def updateTargetXY(self):
        if self.demo_state == 0:
            pass

        elif self.demo_state == 1:
            pass

        elif self.demo_state == 2:
            self.go_pos_x_msg.data = min(self.cur_xy[0] + self.x_step_distance, self.object_x)
            self.go_pos_y_msg.data = self.object_y
            self.go_pos_x_pub.publish(self.go_pos_x_msg)
            self.go_pos_y_pub.publish(self.go_pos_y_msg)

        elif self.demo_state == 3:
            self.go_pos_x_msg.data = self.cur_xy[0]
            self.go_pos_y_msg.data = self.cur_xy[1]
            self.go_pos_x_pub.publish(self.go_pos_x_msg)
            self.go_pos_y_pub.publish(self.go_pos_y_msg)

        elif self.demo_state == 4:
            self.go_pos_x_msg.data = self.cur_xy[0] - self.recovering_remove_distance
            self.go_pos_x_pub.publish(self.go_pos_x_msg)

        else:
            self.go_pos_x_msg.data = self.cur_xy[0] - self.wall_leave_distance
            self.go_pos_x_pub.publish(self.go_pos_x_msg)

        self.target_xy[0] = self.go_pos_x_msg.data
        self.target_xy[1] = self.go_pos_y_msg.data

    def updateTouchingState(self):
        if np.linalg.norm(self.pre_xy - self.cur_xy) < self.touching_thresh:
            self.touching_count = self.touching_count + 1
        else:
            self.touching_count = 0

    def update(self):
        self.pre_xy = np.copy(self.tmp_xy)
        self.tmp_xy = np.copy(self.cur_xy)
        self.trajectory = self.cur_xy - self.pre_xy
        self.theta = np.arccos(np.dot(self.target_axis, self.trajectory) / (np.linalg.norm(self.target_axis) * np.linalg.norm(self.trajectory)))

        if abs(self.theta) < self.reaching_theta_thresh:
            self.theta_count = self.theta_count + 1
        else:
            self.theta_count = 0

    def mocapCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.orientation
        self.cur_xy[0] = self.odom.pose.position.x
        self.cur_xy[1] = self.odom.pose.position.y  # real robot

    def odomCallback(self, msg):
        self.odom = msg
        self.quaternion = self.odom.pose.pose.orientation
        self.cur_xy[0] = self.odom.pose.pose.position.x  # simulation
        self.cur_xy[1] = self.odom.pose.pose.position.y

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data
        if msg.data:
            ######################### real robot ########################
            self.pre_xy[0] = self.odom.pose.position.x
            self.pre_xy[1] = self.odom.pose.position.y
            #############################################################

            ######################## simulation #########################
            # self.pre_xy[0] = self.odom.pose.pose.position.x
            # self.pre_xy[1] = self.odom.pose.pose.position.y
            #############################################################

            self.tmp_xy = np.copy(self.pre_xy)
            self.demo_state = 2


if __name__ == '__main__':
    rospy.init_node('remove_task', anonymous=False)
    node = RemoveTask()
    rospy.spin()
