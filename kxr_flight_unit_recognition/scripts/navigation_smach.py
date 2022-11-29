#! /usr/bin/env python

import rospy
import time
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import UInt8
from kxr_rosserial_msgs.msg import FlightState

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

class Arm_off(State):
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


class Arm_on(State):
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
            time.sleep(2.0)
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
            time.sleep(2.0)
            self.flight_state_msg.data = 5  # landing
            self.flight_state_pub.publish(self.flight_state_msg)
            print('current state = {}'.format(state[self.flight_state]))
            print('publish landing command')

        return 'done'

    def FlightStateCallback(self, msg):
        self.flight_state = msg.state


if __name__ == '__main__':
    rospy.init_node('navigation')
    armoff = Arm_off()
    armon = Arm_on()
    takeoff = Takeoff()
    landing = Landing()

    rospy.spin()

