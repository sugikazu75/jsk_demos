#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
import numpy as np
from kxr_interface import kxrInterface, BaseState, Start

class FoottoWheel(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['foot_to_wheel_target_pitch', 'foot_to_wheel_default_pitch_gain', 'foot_to_wheel_transition_pitch_gain', 'foot_to_wheel_target_z_output', 'foot_to_wheel_target_control_mode'])
        self.target_pitch = None
        self.default_pitch_gain = None
        self.transition_pitch_gain = None
        self.target_z_output = None
        self.target_control_mode = None

        self.done = False

    def execute(self, userdata):
        self.target_pitch = userdata.foot_to_wheel_target_pitch
        self.default_pitch_gain = userdata.foot_to_wheel_default_pitch_gain
        self.transition_pitch_gain = userdata.foot_to_wheel_transition_pitch_gain
        self.target_z_output = userdata.foot_to_wheel_target_z_output
        self.target_control_mode = userdata.foot_to_wheel_target_control_mode

        self.done = False

        self.robot.setZOffset(self.robot.getPID().z.offset - self.robot.getPID().z.output + self.target_z_output)
        self.robot.goPosPitch(self.target_pitch)
        time.sleep(3.0)
        self.robot.setDockingMode(False)
        time.sleep(0.5)
        self.robot.setPitchGain(self.transition_pitch_gain[0], self.transition_pitch_gain[1], self.transition_pitch_gain[2])
        time.sleep(0.5)

        rospy.loginfo("wait for pitch convergion. target={}".format(self.target_pitch))
        while not (self.robot.getForceSkip() or self.done):
            time.sleep(0.02)
            if abs(self.robot.getPID().pitch.pos_error) < 0.03:
                self.done = True

            if rospy.is_shutdown():
                return 'preempted'

        rospy.loginfo('set docking mode true')
        self.robot.setDockingMode(True)
        time.sleep(1.0)
        self.robot.setPitchGain(self.default_pitch_gain[0], self.default_pitch_gain[1], self.default_pitch_gain[2])

        ## mode
        # self.robot.setControlMode(True)
        rospy.loginfo('raise throttle')
        self.robot.setZIControl(True)
        time.sleep(7.0)
        target_z = 0.5
        self.robot.setZOffset((self.robot.getPID().z.setpoint - target_z) * self.robot.getPID().z.gain[0] + self.robot.getPID().z.offset)
        self.robot.goPosZ(target_z)
        ## mode

        rospy.loginfo('transform to wheel mode')
        self.robot.callMotion(5)
        self.robot.goPosPitch(0.0)

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'



class WheeltoFoot(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'], io_keys=['wheel_to_foot_target_pitch', 'wheel_to_foot_default_pitch_gain', 'wheel_to_foot_transition_pitch_gain', 'wheel_to_foot_target_z_output', 'wheel_to_foot_target_control_mode'])
        self.target_pitch = None
        self.default_pitch_gain = None
        self.transition_pitch_gain = None
        self.target_z_output = None
        self.target_control_mode = None

        self.done = False

    def execute(self, userdata):
        self.target_pitch = userdata.wheel_to_foot_target_pitch
        self.default_pitch_gain = userdata.wheel_to_foot_default_pitch_gain
        self.transition_pitch_gain = userdata.wheel_to_foot_transition_pitch_gain
        self.target_z_output = userdata.wheel_to_foot_target_z_output
        self.target_control_mode = userdata.wheel_to_foot_target_control_mode

        self.done = False

        rospy.loginfo('transform to foot mode')
        self.robot.callMotion(8)
        self.robot.setZOffset(self.robot.getPID().z.offset - self.robot.getPID().z.output + self.target_z_output)
        rospy.loginfo('set z offset')
        self.robot.setControlMode(False)
        rospy.loginfo('set control mode false')
        self.robot.goPosPitch(self.target_pitch)
        rospy.loginfo('set target pitch')
        self.robot.goPosX(self.robot.getPose().pose.position.x - 0.1)
        rospy.loginfo('set target x')
        self.robot.setPitchGain(self.transition_pitch_gain[0], self.transition_pitch_gain[1], self.transition_pitch_gain[2])
        time.sleep(10.0)

        rospy.loginfo('wait for foot is contacting. targte pitch={}'.format(self.target_pitch))
        while not (abs(self.robot.getPID().pitch.pos_error) < 0.05 or self.robot.getForceSkip()):
            pass
        self.robot.resetForceSkip()

        time.sleep(3.0)
        self.robot.goPosPitch(0.0)
        self.robot.goPosX(self.robot.getPose().pose.position.x)
        rospy.loginfo("decrease throttle")
        self.robot.setZOffset(self.robot.getPID().z.offset - self.robot.getPID().z.output + 4.0)
        time.sleep(3.0)
        self.robot.setDockingMode(False)
        time.sleep(0.5)

        self.robot.resetForceSkip()
        rospy.loginfo("wait for pitch convergion. target pitch={}".format(0.0))
        while not (self.robot.getForceSkip() or self.done):
            time.sleep(0.02)
            if abs(self.robot.getPID().pitch.pos_error) < 0.03:
                self.done = True

            if rospy.is_shutdown():
                return 'preempted'

        self.robot.setDockingMode(True)
        time.sleep(1.0)
        self.robot.setPitchGain(self.default_pitch_gain[0], self.default_pitch_gain[1], self.default_pitch_gain[2])

        time.sleep(1.0)
        self.robot.resetForceSkip()
        return 'succeeded'


if __name__ == '__main__':
    rospy.init_node('task')

    robot = kxrInterface()

    sm_top = smach.StateMachine(outcomes=['Succeeded', 'Preempted'])

    sm_top.userdata.foot_to_wheel_target_pitch = 0.0
    sm_top.userdata.foot_to_wheel_default_pitch_gain = np.array((20, 1, 4))
    sm_top.userdata.foot_to_wheel_transition_pitch_gain = np.array((40, 1, 4))
    sm_top.userdata.foot_to_wheel_target_z_output = 4.0
    sm_top.userdata.foot_to_wheel_target_control_mode = False


    sm_top.userdata.wheel_to_foot_target_pitch = 0.0
    sm_top.userdata.wheel_to_foot_default_pitch_gain = np.array((20, 1, 4))
    sm_top.userdata.wheel_to_foot_transition_pitch_gain = np.array((40, 1, 4))
    sm_top.userdata.wheel_to_foot_target_z_output = 7.0
    sm_top.userdata.wheel_to_foot_target_control_mode = False

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'FoottoWheel',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('FoottoWheel', FoottoWheel(robot),
                               transitions={'succeeded':'Wait',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('Wait', Start(robot),
                               transitions={'succeeded':'WheeltoFoot',
                                            'preempted':'Preempted'})

        smach.StateMachine.add('WheeltoFoot', WheeltoFoot(robot),
                               transitions={'succeeded':'Succeeded',
                                            'preempted':'Preempted'})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()
