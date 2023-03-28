#!/usr/bin/env python

import rospy
import tf
import time
import numpy as np
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int16


def isConverged(value, thresh):
    if abs(value) < thresh:
        return True
    else:
        return False


# class forceTransitioner:
#     def __init__(self):
#         force_transition_sub = rospy.Subscriber('joy', Joy, self.forceTransitionCallback)
#         self.force_transition_flag = False

#     def forceTransitionCallback(self, msg):
#         if msg.axes[10] == -1.0 and msg.buttons[1] == 1:
#             self.force_transition_flag = True

#     def setForceTransitionFlag(flag):
#         self.force_transition_flag = flag

#     def getForceTransitionFlag():
#         return self.force_transition_flag


class mocapTransformer:
    def __init__(self):
        self.mocap_pose_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)

        self.br = tf.TransformBroadcaster()

    def mocapCallback(self, msg):
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), rospy.Time.now(), 'flight_unit', 'world')

class apriltagTransformer:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(1), self.timerCallback)

        ################# parameter ##########################
        self.tag_frame = '1'
        self.world_frame = 'world'
        ######################################################

    def timerCallback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(self.world_frame, self.tag_frame, rospy.Time(0))
            rospy.set_param('apriltag_pos', trans)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        force_transition_sub = rospy.Subscriber('joy', Joy, self.forceTransitionCallback)
        self.force_transition_flag = False

    def execute(self, userdata):
        self.force_transition_flag = False
        while True:
            if self.force_transition_flag:
                print('\033[31mforce transition\033[0m')
                return 'done'
            else:
                print('waiting')
                time.sleep(1.0)

    def forceTransitionCallback(self, msg):
        if msg.axes[10] == -1.0 and msg.buttons[1] == 1:
            self.force_transition_flag = True


class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        force_transition_sub = rospy.Subscriber('joy', Joy, self.forceTransitionCallback)
        self.force_transition_flag = False

        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_z_msg = Float32()
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)

        ################### variables  #####################
        self.odom = None
        self.count = 0
        self.pre_pos = np.zeros(3)
        self.cur_pos = np.zeros(3)
        self.tmp_pos = np.zeros(3)
        self.apriltag_pos = None
        ###################################################

        ################### parameter ####################
        self.approach_x_step = 0.2
        self.approach_y_step = 0.1
        self.approach_z_step = 0.1
        self.approach_xy_thresh = 0.1
        self.approach_offset = np.array((-0.2, -0.15, 0.0))
        ##################################################

    def execute(self, userdata):
        self.force_transition_flag = False

        while self.odom is None:
            pass

        while(True):
            time.sleep(0.2)
            if self.force_transition_flag:
                print('\033[31mforce transition\033[0m')
                return 'done'

            if not rospy.has_param('apriltag_pos'):
                print('\033[31mnot exist apriltag_pos in rosparam\033[0m')
                pass
            else:
                self.apriltag_pos = rospy.get_param('apriltag_pos')
                self.target_pos = np.array((self.apriltag_pos[0], self.apriltag_pos[1], self.apriltag_pos[2])) + self.approach_offset
                diff = self.target_pos - self.cur_pos
                print('apriltag_pos=[{}, {}, {}]\tcurrent_pos=[{}, {}, {}]\tdiff=[{}, {}, {}]'.format(self.apriltag_pos[0], self.apriltag_pos[1], self.apriltag_pos[2], self.cur_pos[0], self.cur_pos[1], self.cur_pos[2], diff[0], diff[1], diff[2]))
                self.go_pos_x_msg.data = min(self.cur_pos[0] + self.approach_x_step * np.sign(self.target_pos[0] - self.cur_pos[0]), self.apriltag_pos[0] + self.approach_x_step)
                self.go_pos_y_msg.data = self.target_pos[1]
                self.go_pos_z_msg.data = self.target_pos[2]

                self.go_pos_x_pub.publish(self.go_pos_x_msg)
                self.go_pos_y_pub.publish(self.go_pos_y_msg)
                self.go_pos_z_pub.publish(self.go_pos_z_msg)

                if np.linalg.norm(self.cur_pos[0:2] - self.target_pos[0:2]) < self.approach_xy_thresh:
                    time.sleep(1.0)
                    print('\033[31mapproached\033[0m')
                    return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_pos[0] = msg.pose.position.x
        self.cur_pos[1] = msg.pose.position.y
        self.cur_pos[2] = msg.pose.position.z

    def forceTransitionCallback(self, msg):
        if msg.axes[10] == -1.0 and msg.buttons[1] == 1:
            self.force_transition_flag = True

class DetectContact(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        force_transition_sub = rospy.Subscriber('joy', Joy, self.forceTransitionCallback)
        self.force_transition_flag = False

        ############### variables ##################
        self.odom = None
        self.cur_pos = np.zeros(3)
        self.pre_pos = np.zeros(3)
        self.tmp_pos = np.zeros(3)
        self.detect_contact_count = 0
        ############################################

        ############### parameter ##################
        self.detect_contact_thresh = 0.1
        self.detect_contact_max = 3
        ############################################

    def execute(self, userdata):
        self.force_transition_flag = False
        while(True):
            if self.force_transition_flag:
                print('\033[31mforce transition\033[0m')
                return 'done'
            else:
                time.sleep(1.0)
                self.pre_pos = np.copy(self.tmp_pos)
                self.tmp_pos = np.copy(self.cur_pos)
                if np.linalg.norm(self.pre_pos - self.cur_pos) < self.detect_contact_thresh:
                    self.detect_contact_count = self.detect_contact_count + 1
                else:
                    self.detect_contact_count = 0

                print('approach count = {}'.format(self.detect_contact_count))
                print('pre=[{}, {}]\tcur=[{}, {}]'.format(self.pre_pos[0], self.pre_pos[1], self.cur_pos[0], self.cur_pos[1]))

                if self.detect_contact_count > self.detect_contact_max:
                    print('\033[31mcontact detected\033[0m')
                    return 'done'

    def mocapCallback(self, msg):
        self.odom = msg
        self.cur_pos[0] = msg.pose.position.x
        self.cur_pos[1] = msg.pose.position.y

    def forceTransitionCallback(self, msg):
        if msg.axes[10] == -1.0 and msg.buttons[1] == 1:
            self.force_transition_flag = True

class Release(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.gripper_msg = Int16()
        self.gripper_pub = rospy.Publisher('/lisp_command', Int16, queue_size=1)

    def execute(self, userdata):
        self.gripper_msg.data = 3
        self.gripper_pub.publish(self.gripper_msg)
        print('\033[31mgripper open\033[0m')
        time.sleep(2.0)
        return 'done'

class Leave(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.odom = None
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)

        ################## parameter ########################
        self.leave_x_distance = 0.3
        #####################################################

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass
        self.go_pos_x_msg.data = self.odom.pose.position.x - self.leave_x_distance
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        print('\033[31mleave x = {}\033[0m'.format(self.go_pos_x_msg.data))
        return 'done'

    def mocapCallback(self, msg):
        self.odom = msg


if __name__ == '__main__':
    rospy.init_node('integrated_experiment_node')
    apriltag_transformer = apriltagTransformer()
    mocap_transformer = mocapTransformer()
    sm_top = smach.StateMachine(outcomes=['success'])
    with sm_top:
        smach.StateMachine.add('WAIT', Wait(), transitions={'done':'APPROACH'})
        smach.StateMachine.add('APPROACH', Approach(), transitions={'done':'DETECT_CONTACT'})
        smach.StateMachine.add('DETECT_CONTACT', DetectContact(), transitions={'done':'RELEASE'})
        smach.StateMachine.add('RELEASE', Release(), transitions={'done':'LEAVE'})
        smach.StateMachine.add('LEAVE', Leave(), transitions={'done':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
