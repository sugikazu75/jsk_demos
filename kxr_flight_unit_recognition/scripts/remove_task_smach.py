#! /usr/bin/env python

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
from kxr_rosserial_msgs.msg import PIDControllerState6D
from kxr_rosserial_msgs.msg import PIDControllerState

def clamp(minimam, value, maximum):
    return min(max(minimam, value), maximum)

class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes=['approach'])
        self.do_demo_flag_sub = rospy.Subscriber('do_remove_demo', Bool, self.doDemoFlagCallback)
        self.do_demo_flag = False
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.odom = None
        self.cur_xy = np.zeros(2)

    def execute(self, userdata):
        self.do_demo_flag = False
        while(1):
            sleep(1.0)
            print('waiting. current=[{}, {}]'.format(self.cur_xy[0], self.cur_xy[1]))
            if self.do_demo_flag:
                return 'approach'

    def doDemoFlagCallback(self, msg):
        self.do_demo_flag = msg.data

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y

class Approach(State):
    def __init__(self):
        State.__init__(self, outcomes=['contact'])
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.odom = None

        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)
        self.gripper_msg = Int16()

        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_x_msg = Float32()

        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()

        self.cur_xy = np.zeros(2)
        self.approach_count = 0

        ############# parameter ################
        self.approach_target_xy = np.array((0.95, 0.0))
        self.approach_step_distance = 0.2
        self.approach_target_thresh = 0.1
        self.approach_count_max = 3
        ########################################

    def execute(self, userdata):
        self.go_pos_y_msg.data = self.approach_target_xy[1]
        self.go_pos_y_pub.publish(self.go_pos_y_msg)
        self.gripper_msg.data = 3
        self.gripper_pub.publish(self.gripper_msg)
        while(self.odom is None):
            pass
        while(1):
            sleep(1)
            print('approaching. target=[{}, {}] current=[{}, {}], approach_count={}'.format(self.approach_target_xy[0], self.approach_target_xy[1], self.cur_xy[0], self.cur_xy[1], self.approach_count))

            self.go_pos_x_msg.data = min(self.cur_xy[0] + self.approach_step_distance, self.approach_target_xy[0])
            self.go_pos_x_pub.publish(self.go_pos_x_msg)
            if np.linalg.norm(self.cur_xy - self.approach_target_xy) < self.approach_target_thresh:
                self.approach_count = self.approach_count + 1
            else:
                self.approach_count = 0

            if self.approach_count > self.approach_count_max:
                return 'contact'


    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y


class Contact(State):
    def __init__(self):
        State.__init__(self, outcomes=['recover', 'remove'])
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.odom = None
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)
        self.gripper_msg = Int16()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_x_msg = Float32()
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()

        self.mocap_sub_count = 0
        self.mocap_subscribed = False
        self.contact_count = 0
        self.contact_fail_count = 0
        self.cur_xy = np.zeros(2)
        self.tmp_xy = np.zeros(2)
        self.pre_xy = np.zeros(2)

        ################ parameter #######################
        self.contact_fail_count_max = 3
        self.contact_count_max = 3
        self.contact_distance_thresh = 0.1
        ##################################################

    def execute(self, userdata):
        self.contact_count = 0
        self.contact_fail_count = 0
        self.gripper_msg.data = 4
        self.gripper_pub.publish(self.gripper_msg)

        while(self.odom is None):
            pass

        self.go_pos_x_msg.data = self.cur_xy[0]
        self.go_pos_y_msg.data = self.cur_xy[1]
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        self.go_pos_y_pub.publish(self.go_pos_y_msg)

        while(1):
            sleep(1.0)
            print('contacting. contact_count={}, contact_fail_count={}'.format(self.contact_count, self.contact_fail_count))

            if np.linalg.norm(self.pre_xy - self.cur_xy) < self.contact_distance_thresh:
                self.contact_count = self.contact_count + 1
                self.contact_fail_count = 0
            else:
                self.contact_count = 0
                self.contact_fail_count = self.contact_fail_count + 1

            if self.contact_count > self.contact_count_max:
                return 'remove'
            if self.contact_fail_count > self.contact_fail_count_max:
                return 'recover'

    def mocapCallback(self, msg):
        self.odom = msg
        self.mocap_sub_count = self.mocap_sub_count + 1
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y
        if self.mocap_sub_count % 100 == 0:
            self.mocap_subscribed = True
            self.pre_xy = np.copy(self.tmp_xy)
            self.tmp_xy = np.copy(self.cur_xy)
            self.mocap_sub_count = 0


class Recover(State):
    def __init__(self):
        State.__init__(self, outcomes=['approach'])
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)

        self.go_pos_x_msg = Float32()
        self.go_pos_y_msg = Float32()
        self.gripper_msg = Int16()
        self.odom = None

        self.cur_xy = np.zeros(2)

        ################### parameter ####################
        self.recover_target_xy = np.array((0.5, 0.0))
        self.recover_distance_thresh = 0.1
        ##################################################

    def execute(self, userdata):
        self.odom = None
        self.go_pos_x_msg.data = self.recover_target_xy[0]
        self.go_pos_y_msg.data = self.recover_target_xy[1]
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        self.go_pos_y_pub.publish(self.go_pos_y_msg)

        self.gripper_msg.data = 3
        self.gripper_pub.publish(self.gripper_msg)

        while(self.odom is None):
            pass

        while(1):
            sleep(1)
            print('recovering. target=[{}, {}], current=[{}, {}]'.format(self.recover_target_xy[0], self.recover_target_xy[1], self.cur_xy[0], self.cur_xy[1]))
            if np.linalg.norm(self.cur_xy - self.recover_target_xy) < self.recover_distance_thresh:
                return 'approach'


    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y


class Remove(State):
    def __init__(self):
        State.__init__(self, outcomes=['wait'])
        self.z_controller_state = None
        self.pid_controller_state_6d = None
        self.pid_controller_state_6d_sub = rospy.Subscriber('pid', PIDControllerState6D, self.pidCallback)
        self.odom = None
        self.pos_z = 0.0
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.z_offset_msg = Float32()
        self.z_offset_pub = rospy.Publisher('pid/setoffset/z', Float32, queue_size=1)
        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_yaw_msg = Float32()
        self.go_pos_yaw_pub = rospy.Publisher('go_pos/yaw', Float32, queue_size=1)

        self.cur_xy = np.zeros(2)
        self.initial_pos_z = 0.0
        self.initial_z_offset = 0.0

        ############## parameter ######################
        self.remove_target_xy = np.array((0.3, 0.0))
        self.remove_falling_detect_height = 0.2
        self.remove_z_offset_step = 0.4
        ###############################################

    def execute(self, userdata):
        self.odom = None
        self.z_controller_state = None
        self.pid_controller_state_6d = None
        print('remove. decreasing z acc until falling detection.')

        while(self.odom is None or self.z_controller_state is None):
            pass
        sleep(0.1)

        self.initial_pos_z = self.pos_z
        self.initial_z_offset = self.z_controller_state.offset
        self.z_offset_msg.data = self.initial_z_offset

        while(1):
            sleep(0.1)
            self.z_offset_msg.data = self.z_offset_msg.data - self.remove_z_offset_step
            self.z_offset_pub.publish(self.z_offset_msg)

            if self.initial_pos_z - self.pos_z > self.remove_falling_detect_height:
                self.z_offset_msg.data = self.initial_z_offset
                self.z_offset_pub.publish(self.z_offset_msg)

                print('go to [{}, {}]'.format(self.remove_target_xy[0], self.remove_target_xy[1]))
                self.go_pos_x_msg.data = self.remove_target_xy[0]
                self.go_pos_y_msg.data = self.remove_target_xy[1]
                self.go_pos_x_pub.publish(self.go_pos_x_msg)
                self.go_pos_y_pub.publish(self.go_pos_y_msg)

                return 'wait'

    def pidCallback(self, msg):
        self.pid_controller_state_6d = msg
        self.z_controller_state = msg.z

    def mocapCallback(self, msg):
        self.odom = msg
        self.pos_z = msg.pose.position.z
        self.cur_xy[0] = msg.pose.position.x
        self.cur_xy[1] = msg.pose.position.y


if __name__ == '__main__':
    node = rospy.init_node('remove_task')
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('WAIT', Wait(), transitions={'approach':'APPROACH'})
        StateMachine.add('APPROACH', Approach(), transitions={'contact':'CONTACT'})
        StateMachine.add('CONTACT', Contact(), transitions={'recover':'RECOVER', 'remove':'REMOVE'})
        StateMachine.add('RECOVER', Recover(), transitions={'approach':'APPROACH'})
        StateMachine.add('REMOVE', Remove(), transitions={'wait':'WAIT'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
