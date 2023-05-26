#!/usr/bin/env python

import rospy
import smach
import smach_ros
from kxr_interface import kxrInterface
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray, PoseStamped, Wrench, Quaternion, Vector3
import tf.transformations as tft
from std_msgs.msg import UInt8, Empty
import std_srvs.srv
import tf
from sensor_msgs.msg import Joy
import time

def clamp(value, minimum, maximum):
    if minimum > maximum:
        tmp = minimum
        minimum = maximum
        maximum = tmp
    return max(minimum, min(value, maximum))

class mocapTransformer:
    def __init__(self):
        self.mocap_pose_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)

        self.br = tf.TransformBroadcaster()

    def mocapCallback(self, msg):
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w), rospy.Time.now(), 'flight_unit', 'world')

class objectTransformer:
    def __init__(self):
        self.object_mocap_sub = rospy.Subscriber("/object/mocap/pose", PoseStamped, self.objectMocapCallback)
        self.object_euler = Vector3()
        self.object_euler_pub = rospy.Publisher("/object/euler", Vector3, queue_size=1)
        self.cnt = 0

    def objectMocapCallback(self, msg):
        self.cnt = (self.cnt + 1) % 20
        if self.cnt == 0:
            euler = tf.transformations.euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
            self.object_euler.x = euler[0]
            self.object_euler.y = euler[1]
            self.object_euler.z = euler[2]
            self.object_euler_pub.publish(self.object_euler)


class apriltagTransformer:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(1), self.timerCallback)

        self.tag_frame = '1'
        self.world_frame = 'world'

    def timerCallback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(self.world_frame, self.tag_frame, rospy.Time(0))
            rospy.set_param('apriltag_pos', trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass


class BaseState(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot


class Start(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])
        self.task_start = False
        self.task_start_sub = rospy.Subscriber("/task_start", Empty, self.taskStartCallback)

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):
        self.task_start = False
        while not (self.task_start or self.robot.getForceSkip()):
            rospy.sleep(0.1)
            rospy.loginfo_once("wait to start task")

            if rospy.is_shutdown():
                return 'preempted'

        rospy.loginfo_once('task start!')
        time.sleep(1.0)
        self.robot.resetForceSkip();
        return 'succeeded'

class Pickup(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])
        self.pickup_done = False
        self.pickup_done_sub = rospy.Subscriber('/pickup_done', Empty, self.pickupCallback)

    def pickupCallback(self, msg):
        self.pickup_done = True

    def execute(self, userdata):
        while not (self.pickup_done or self.robot.getForceSkip()):
            rospy.sleep(0.2)
            rospy.loginfo_once("wait to complete pickup")

            if rospy.is_shutdown():
                return 'preempted'

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

class RotateYaw(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['target_yaw', 'yaw_converged_thresh', 'yaw_step'])
        self.target_yaw = None
        self.yaw_converged_thresh = None
        self.yaw_step = None

    def execute(self, userdata):
        self.target_yaw = userdata.target_yaw
        self.yaw_converged_thresh = userdata.yaw_converged_thresh
        self.yaw_step = userdata.yaw_step

        while not (self.isYawConverged() or self.robot.getForceSkip()):
            time.sleep(0.2)
            self.robot.goPosYaw(clamp(self.robot.getRpy()[2] + self.yaw_step, self.target_yaw, self.robot.getInitialRpy()[2]))

            if rospy.is_shutdown():
                return 'preempted'

        if self.isYawConverged():
            rospy.loginfo_once('yaw rotation is converged')

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

    def isYawConverged(self):
        return abs(self.robot.getRpy()[2] - self.target_yaw) < self.yaw_converged_thresh

class GroundApproach(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['ground_target_x', 'ground_step_x', 'ground_converged_thresh'])
        self.prev_pose = PoseStamped()
        self.ground_target_x = None
        self.ground_step_x = None
        self.ground_converged_thresh = None

    def execute(self, userdata):
        self.ground_target_x = userdata.ground_target_x
        self.ground_step_x = userdata.ground_step_x
        self.ground_converged_thresh = userdata.ground_converged_thresh

        while not (self.robot.getForceSkip() or self.isXConverged()):
            time.sleep(0.2)
            self.robot.goPosX(clamp(self.robot.getPose().pose.position.x + self.ground_step_x, self.robot.getInitialPose().pose.position.x, self.ground_target_x))

            if rospy.is_shutdown():
                return 'preempted'
            pass

        if self.isXConverged():
            rospy.loginfo_once('x movement is converged')

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

    def isXConverged(self):
        return abs(self.robot.getPose().pose.position.x - self.ground_target_x) < self.ground_converged_thresh

class ZIControl(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

    def execute(self, userdata):
        while not (self.robot.getPID().z.i_control_flag or self.robot.getForceSkip()):
            self.robot.setZIControl(True)

        if rospy.is_shutdown():
            return 'preempted'

        if self.robot.getPID().z.i_control_flag:
            rospy.loginfo_once('z i control is started')

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

class AerialManipulationApproach(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['aerial_manipulation_approach_step', 'aerial_manipulation_approach_offset', 'aerial_manipulation_approach_thresh'])
        self.approach_step = None
        self.approach_offset = None
        self.approach_thresh = None
        self.apriltag_pos = None
        self.approach_done = False

    def execute(self, userdata):
        self.approach_step = userdata.aerial_manipulation_approach_step
        self.approach_offset = userdata.aerial_manipulation_approach_offset
        self.approach_thresh = userdata.aerial_manipulation_approach_thresh
        self.apriltag_pos = None
        self.approach_done = False

        while not (self.robot.getForceSkip() or self.approach_done):
            time.sleep(0.2)
            self.robot.setArmFeedbackMode(True) # arm feedback control flag
            time.sleep(1.0)
            self.robot.callMotion(6) # arm feedback control

            if not rospy.has_param('apriltag_pos'):
                rospy.loginfo_once('not exist apriltag_pos in rosparam')
                pass

            else:
                rospy.loginfo_once('found apriltag')
                apriltag_pos = rospy.get_param('apriltag_pos')
                target_pos = np.array((apriltag_pos[0], apriltag_pos[1], apriltag_pos[2])) + self.approach_offset
                cur_pos = self.robot.getPosition()
                diff = target_pos - cur_pos
                # rospy.loginfo('apriltag_pos=[{}, {}, {}]\tcurrent_pos=[{}, {}, {}]\tdiff=[{}, {}, {}]'.format(apriltag_pos[0], apriltag_pos[1], apriltag_pos[2], cur_pos[0], cur_pos[1], cur_pos[2], diff[0], diff[1], diff[2]))

                self.robot.goPosX(min(cur_pos[0] + self.approach_step[0], apriltag_pos[0] + self.approach_step[0]))
                self.robot.goPosY(target_pos[1])
                self.robot.goPosZ(target_pos[2])

                if np.linalg.norm(cur_pos[0:2] - target_pos[0:2]) < self.approach_thresh:
                    rospy.loginfo('approached')
                    self.approach_done = True

            if rospy.is_shutdown():
                return 'preempted'

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

class AerialManipulationDetectContact(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['aerial_manipulation_detect_contact_thresh', 'aerial_manipulation_detect_contact_count'])
        self.pre_position = self.robot.getPosition()
        self.cur_position = self.robot.getPosition()
        self.detect_contact_thresh = None
        self.detect_contact_count = None
        self.done = False
        time.sleep(1.0)

    def execute(self, userdata):
        self.robot.setArmFeedbackMode(False)
        self.detect_contact_thresh = userdata.aerial_manipulation_detect_contact_thresh
        self.detect_contact_count = userdata.aerial_manipulation_detect_contact_count
        self.count = 0
        self.done = False

        while not (self.robot.getForceSkip() or self.done):
            time.sleep(1.0)
            tmp = self.robot.getPosition()
            self.pre_position = np.copy(self.cur_position)
            self.cur_position = np.copy(tmp)

            diff = np.linalg.norm(self.pre_position - self.cur_position)
            if diff < self.detect_contact_thresh:
                self.count = self.couunt + 1
            else:
                self.count = 0

            rospy.loginfo('approach count = {}, diff = {}'.format(self.count, diff))
            if self.count > self.detect_contact_count:
                rospy.loginfo('Detected Contact!')
                self.done = True

            if rospy.is_shutdown():
                return 'preempted'

        self.robot.callMotion(3) # release object

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

class AerialManipulationLeave(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['aerial_manipulation_leave_step'])
        self.done = False
        self.leave_step = None

    def execute(self, userdata):
        self.leave_step = userdata.aerial_manipulation_leave_step
        self.done = False
        time.sleep(0.2)

        while not (self.robot.getForceSkip() or self.done):
            self.robot.goPosX(self.robot.getPosition()[0] - self.leave_step)

            if rospy.is_shutdown():
                return 'preempted'

            self.done = True

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'

if __name__ == '__main__':
    rospy.init_node('task')
    mocap_transformer = mocapTransformer()
    apriltag_transformer = apriltagTransformer()
    object_transformer = objectTransformer()
    robot = kxrInterface()
    sm_top = smach.StateMachine(outcomes=['Succeeded', 'Preempted'])

    sm_top.userdata.target_yaw = 0.0
    sm_top.userdata.yaw_converged_thresh = 0.1
    sm_top.userdata.yaw_step = 0.2
    sm_top.userdata.ground_target_x = 0.0
    sm_top.userdata.ground_step_x = 0.1
    sm_top.userdata.ground_converged_thresh = 0.1

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Pickup',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('Pickup', Pickup(robot),
                               transitions={'succeeded':'RotateYaw',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('RotateYaw', RotateYaw(robot),
                               transitions={'succeeded':'GroundApproach',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('GroundApproach', GroundApproach(robot),
                               transitions={'succeeded':"ZIControl",
                                            'preempted':'Preempted'})

        smach.StateMachine.add('ZIControl', ZIControl(robot),
                               transitions={'succeeded':"AerialManipulation",
                                            'preempted':'Preempted'})

        sm_sub = smach.StateMachine(outcomes=['AerialManipulationSucceeded', 'AerialManipulationPreempted'])

        sm_sub.userdata.aerial_manipulation_approach_step = np.array((0.2, 0.1, 0.1))
        sm_sub.userdata.aerial_manipulation_approach_offset = np.array((-0.2, -0.15, 0.0))
        sm_sub.userdata.aerial_manipulation_approach_thresh = 0.1
        sm_sub.userdata.aerial_manipulation_detect_contact_thresh = 0.1
        sm_sub.userdata.aerial_manipulation_detect_contact_count = 3
        sm_sub.userdata.aerial_manipulation_leave_step = 0.3

        with sm_sub:
            smach.StateMachine.add('AerialManipulationStart', Start(robot),
                                   transitions={'succeeded':'AerialManipulationApproach',
                                                'preempted':'AerialManipulationPreempted'})

            smach.StateMachine.add('AerialManipulationApproach', AerialManipulationApproach(robot),
                                   transitions={'succeeded':'AerialManipulationDetectContact',
                                                'preempted':'AerialManipulationPreempted'})

            smach.StateMachine.add('AerialManipulationDetectContact', AerialManipulationDetectContact(robot),
                                   transitions={'succeeded':'AerialManipulationLeave',
                                                'preempted':'AerialManipulationPreempted'})

            smach.StateMachine.add('AerialManipulationLeave', AerialManipulationLeave(robot),
                                   transitions={'succeeded':'AerialManipulationSucceeded',
                                                'preempted':'AerialManipulationPreempted'})


        smach.StateMachine.add('AerialManipulation', sm_sub, transitions={'AerialManipulationSucceeded':'Succeeded',
                                                                          'AerialManipulationPreempted':'Preempted'})


        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()
