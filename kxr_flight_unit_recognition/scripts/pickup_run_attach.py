#!/usr/bin/env python

import rospy
import tf
import time
import smach
import smach_ros
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Empty
from sensor_msgs.msg import Joy

from kxr_interface import kxrInterface, BaseState, Start
from state_transitioner import WheeltoFoot, FoottoWheel

def clamp(value, minimum, maximum):
    if minimum > maximum:
        tmp = minimum
        minimum = maximum
        maximum = tmp
    return max(minimum, min(value, maximum))

class WalkAndPickDemo(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

    def execute(self, userdata):
        self.robot.callMotion(1)
        while not self.robot.forceSkip():
            if rospy.is_shutdown():
                return 'preempted'
            pass
        self.robot.callMotion(0)

        time.sleep(5.0)
        self.robot.resetForceSkip();
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
            if self.robot.getRpy()[2] > 0:
                self.robot.goPosYaw(clamp(self.robot.getRpy()[2] - self.yaw_step, self.target_yaw, self.robot.getInitialRpy()[2]))
                # rospy.loginfo('{}, {}'.format(self.robot.getRpy()[2], clamp(self.robot.getRpy()[2] - self.yaw_step, self.target_yaw, self.robot.getInitialRpy()[2])))
            else:
                self.robot.goPosYaw(clamp(self.robot.getRpy()[2] + self.yaw_step, self.target_yaw, self.robot.getInitialRpy()[2]))
                # rospy.loginfo('{} {}'.format(self.robot.getRpy()[2], clamp(self.robot.getRpy()[2] + self.yaw_step, self.target_yaw, self.robot.getInitialRpy()[2])))

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
                rospy.loginfo('apriltag_pos=[{}, {}, {}]\tcurrent_pos=[{}, {}, {}]\tdiff=[{}, {}, {}]'.format(apriltag_pos[0], apriltag_pos[1], apriltag_pos[2], cur_pos[0], cur_pos[1], cur_pos[2], diff[0], diff[1], diff[2]))

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
        self.robot.callMotion(0)
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
                self.count = self.count + 1
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

    robot = kxrInterface()

    sm_top = smach.StateMachine(outcomes=['Succeeded', 'Preempted'])

    sm_top.userdata.target_yaw = 0.0
    sm_top.userdata.yaw_converged_thresh = 0.1
    sm_top.userdata.yaw_step = 0.5
    sm_top.userdata.ground_target_x = 0.5
    sm_top.userdata.ground_step_x = 0.3
    sm_top.userdata.ground_converged_thresh = 0.1

    sm_top.userdata.foot_to_wheel_target_pitch = -0.3
    sm_top.userdata.foot_to_wheel_default_pitch_gain = np.array((20, 1, 4))
    sm_top.userdata.foot_to_wheel_transition_pitch_gain = np.array((40, 1, 4))
    sm_top.userdata.foot_to_wheel_target_z_output = 4.0
    sm_top.userdata.foot_to_wheel_target_control_mode = False

    sm_top.userdata.wheel_to_foot_target_pitch = -0.3
    sm_top.userdata.wheel_to_foot_default_pitch_gain = np.array((20, 1, 4))
    sm_top.userdata.wheel_to_foot_transition_pitch_gain = np.array((40, 1, 4))
    sm_top.userdata.wheel_to_foot_target_z_output = 7.0
    sm_top.userdata.wheel_to_foot_target_control_mode = False

    sm_top.userdata.wheel_to_foot_pitch_gain = np.array((40, 1, 4))


    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'AerialManipulation',
                                            'preempted':'Preempted'})

        # smach.StateMachine.add('Start', Start(robot),
        #                        transitions={'succeeded':'FoottoWheel',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('FoottoWheel', FoottoWheel(robot),
        #                        transitions={'succeeded':'RotateYawWait',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('RotateYawWait', Start(robot),
        #                        transitions={'succeeded':'RotateYaw',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('RotateYaw', RotateYaw(robot),
        #                        transitions={'succeeded':'GroundApproach',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('GroundApproach', GroundApproach(robot),
        #                        transitions={'succeeded':"WheeltoFoot",
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('WheeltoFoot', WheeltoFoot(robot),
        #                        transitions={'succeeded':'TakeoffWait',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('TakeoffWait', Start(robot),
        #                        transitions={'succeeded':'ZIControl',
        #                                     'preempted':'Preempted'})

        # smach.StateMachine.add('ZIControl', ZIControl(robot),
        #                        transitions={'succeeded':"AerialManipulation",
        #                                     'preempted':'Preempted'})

        sm_sub = smach.StateMachine(outcomes=['AerialManipulationSucceeded', 'AerialManipulationPreempted'])

        sm_sub.userdata.aerial_manipulation_approach_step = np.array((0.2, 0.1, 0.1))
        sm_sub.userdata.aerial_manipulation_approach_offset = np.array((-0.2, -0.15, 0.2))
        sm_sub.userdata.aerial_manipulation_approach_thresh = 0.1
        sm_sub.userdata.aerial_manipulation_detect_contact_thresh = 0.05
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
