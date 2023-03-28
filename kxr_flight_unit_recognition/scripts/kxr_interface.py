import rospy
from geometry_msgs.msg import PoseStamped
from kxr_rosserial_msgs.msg import PIDControllerState6D, FlightState
from sensor_msgs.msg import Joy

class kxrInterface(object):
    def __init__(self):
        self.odom = PoseStamped()
        self.pid = PIDControllerState6D()
        self.flight_state = FlightState()
        self.joy = Joy()

        self.odom_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.odomCallback)
        self.pid_sub = rospy.Subscriber('pid', PIDControllerState6D, self.pidCallback)
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.flightStateCallback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)

    def odomCallback(self, msg):
        self.odom = msg

    def pidCallback(self, msg):
        self.pid = msg

    def flightStateCallback(self, msg):
        self.flight_state = msg

    def joyCallback(self, msg):
        self.joy = msg
        if self.joy.axes[10] == 1.0 and self.joy.buttons[1] == 1:
            self.force_skip_flag = 1

    def getOdom(self):
        return self.odom

    def getPID(self):
        return self.pid

    def getFlightState(self):
        return self.flight_state

    def getJoy(self):
        return self.joy

