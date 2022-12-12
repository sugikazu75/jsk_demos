#! /usr/bin/env python

import rospy
import time
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from kxr_rosserial_msgs.msg import FlightState
from kxr_rosserial_msgs.msg import PIDControllerState6D

'''
uint8 NONE=0
uint8 ARM_OFF=1
uint8 ARM_ON=2
uint8 TAKEOFF=3
uint8 HOVERING=4
uint8 LANDING=5
uint8 state
'''

state = ['NONE', 'ARM_OFF', 'ARM_ON', 'TAKEOFF', 'HOVERING', 'LANDING']

class WaitArmOn(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None

    def execute(self, userdata):
        print('wait for arm_on')
        while self.flight_state != 2:
            time.sleep(1.0)
            pass
        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


class WaitTakeoff(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None

    def execute(self, userdata):
        print('wait for takeoff')
        while self.flight_state != 3 and self.flight_state != 4:
            time.sleep(1.0)
            pass
        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


class ArmOff(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None
        self.flight_state_pub = rospy.Publisher('set_nav_state', UInt8, queue_size=1)
        self.flight_state_msg = UInt8()

    def execute(self, userdata):
        while self.flight_state is None:
            pass
        while self.flight_state != 1:  # not armoff
            time.sleep(2.0)
            self.flight_state_msg.data = 1  # armoff
            self.flight_state_pub.publish(self.flight_state_msg)
            print('current state = {}'.format(state[self.flight_state]))
            print('publish armoff command')

        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


class ArmOn(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None
        self.flight_state_pub = rospy.Publisher('set_nav_state', UInt8, queue_size=1)
        self.flight_state_msg = UInt8()

    def execute(self, userdata):
        while self.flight_state is None:
            pass
        while self.flight_state == 0 or self.flight_state == 1:  # none or armoff
            time.sleep(2.0)
            self.flight_state_msg.data = 2  # armon
            self.flight_state_pub.publish(self.flight_state_msg)
            print('current state = {}'.format(state[self.flight_state]))
            print('publish armon command')

        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


class Takeoff(State):
    def __init__(self):
        State.__init__(self, outcomes=['done', 'fail'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None
        self.flight_state_pub = rospy.Publisher('set_nav_state', UInt8, queue_size=1)
        self.flight_state_msg = UInt8()

    def execute(self, userdata):
        while self.flight_state is None:
            pass
        if self.flight_state == 0 or self.flight_state == 1:  # none or armoff. only armon->takeoff is possible
            return 'fail'

        while self.flight_state == 2:  # armon
            time.sleep(3.0)
            self.flight_state_msg.data = 3  # takeoff
            self.flight_state_pub.publish(self.flight_state_msg)
            print('current state = {}'.format(state[self.flight_state]))
            print('publish takeoff command')

        ##### when hovering test #####
        time.sleep(10)
        ##############################

        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


class Landing(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.FlightStateCallback)
        self.flight_state = None
        self.flight_state_pub = rospy.Publisher('set_nav_state', UInt8, queue_size=1)
        self.flight_state_msg = UInt8()

    def execute(self, userdata):
        while self.flight_state is None:
            pass
        while self.flight_state == 3 or self.flight_state == 4:  # takeoff or hoveirng
            time.sleep(3.0)
            self.flight_state_msg.data = 5  # landing
            self.flight_state_pub.publish(self.flight_state_msg)
            print('current state = {}'.format(state[self.flight_state]))
            print('publish landing command')

        ##### when hovering test #####
        time.sleep(10)
        ##############################

        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state

class zIControlEnable(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.pid_sub = rospy.Subscriber('pid', PIDControllerState6D, self.pidCallback)
        self.z_i_control_flag = None
        self.z_i_control_msg = Bool()
        self.z_i_control_pub = rospy.Publisher('pid/i_control/z', Bool, queue_size=1)

    def execute(self, userdata):
        self.z_i_control_flag = None
        while self.z_i_control_flag is None:
            pass
        while not self.z_i_control_flag:
            self.z_i_control_msg.data = True
            self.z_i_control_pub.publish(self.z_i_control_msg)
            time.sleep(2.0)
        return 'done'

    def pidCallback(self, msg):
        self.z_i_control_flag = msg.z.i_control_flag

class zIControlUnable(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
        self.pid_sub = rospy.Subscriber('pid', PIDControllerState6D, self.pidCallback)
        self.z_i_control_flag = None
        self.z_i_control_msg = Bool()
        self.z_i_control_pub = rospy.Publisher('pid/i_control/z', Bool, queue_size=1)

    def execute(self, userdata):
        while self.i_control_flag is None:
            pass
        while self.i_control_flag:
            self.z_i_control_msg.data = False
            self.z_i_control_pub.publish(self.z_i_control_msg)
            time.sleep(2.0)
        return 'done'

    def pidCallback(self, msg):
        self.z_i_control_flag = msg.z.i_control_flag


if __name__ == '__main__':
    node = rospy.init_node('navigation_node')
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('ARM_ON', ArmOn(), transitions={'done':'TAKEOFF'})
        StateMachine.add('TAKEOFF', Takeoff(), transitions={'done':'LANDING', 'fail':'ARM_ON'})
        StateMachine.add('LANDING', Landing(), transitions={'done':'ARM_OFF'})
        StateMachine.add('ARM_OFF', ArmOff(), transitions={'done':'success'})

    sis = smach_ros.IntrospectionServer('smach_server' , sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


