import rospy
from geometry_msgs.msg import PoseStamped
from kxr_rosserial_msgs.msg import PIDControllerState6D, FlightState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, Int16
import tf
import numpy as np

class kxrInterface(object):
    def __init__(self):
        self.pose = PoseStamped()
        self.initial_pose = PoseStamped()
        self.pose_initialized = False
        self.rpy = [0, 0, 0]
        self.initial_rpy = [0, 0, 0]
        self.pid = PIDControllerState6D()
        self.flight_state = FlightState()
        self.joy = Joy()

        self.pose_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.poseCallback)
        self.pid_sub = rospy.Subscriber('pid', PIDControllerState6D, self.pidCallback)
        self.flight_state_sub = rospy.Subscriber('flight_state', FlightState, self.flightStateCallback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)

        self.go_pos_x_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_msg = Float32()
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_z_msg = Float32()
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)
        self.go_pos_yaw_msg = Float32()
        self.go_pos_yaw_pub = rospy.Publisher('go_pos/yaw', Float32, queue_size=1)

        self.z_i_control_msg = Bool()
        self.z_i_control_pub = rospy.Publisher('pid/i_control/z', Bool, queue_size=1)

        self.arm_feedback_msg = Bool()
        self.arm_feedback_pub = rospy.Publisher("arm_feedback", Bool, queue_size=1)
        self.call_motion_msg = Int16()
        self.call_motion_pub = rospy.Publisher("call_motion", Int16, queue_size=1)

        self.force_skip_flag = False

    def poseCallback(self, msg):
        self.pose = msg
        self.rpy = tf.transformations.euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))
        if not self.pose_initialized:
            self.initial_pose = self.pose
            self.initial_rpy = tf.transformations.euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w))

    def pidCallback(self, msg):
        self.pid = msg

    def flightStateCallback(self, msg):
        self.flight_state = msg

    def joyCallback(self, msg):
        self.joy = msg
        if self.joy.axes[10] == 1.0 and self.joy.buttons[1] == 1: # arrow_up and cross
            rospy.loginfo('Force skip')
            self.force_skip_flag = True

    def getPose(self):
        return self.pose

    def getPosition(self):
        return np.array((self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z))

    def getInitialPose(self):
        return self.initial_pose

    def getRpy(self):
        return self.rpy

    def getInitialRpy(self):
        return self.initial_rpy

    def getPID(self):
        return self.pid

    def getFlightState(self):
        return self.flight_state

    def getJoy(self):
        return self.joy

    def getForceSkip(self):
        return self.force_skip_flag

    def resetForceSkip(self):
        self.foece_skip_flag = False

    def goPosX(self, target):
        self.go_pos_x_msg.data = target
        self.go_pos_x_pub.publish(self.go_pos_x_msg)

    def goPosY(self, target):
        self.go_pos_y_msg.data = target
        self.go_pos_y_pub.publish(self.go_pos_y_msg)

    def goPosZ(self, target):
        self.go_pos_z_msg.data = target
        self.go_pos_z_pub.publish(self.go_pos_z_msg)

    def goPosYaw(self, target):
        self.go_pos_yaw_msg.data = target
        self.go_pos_yaw_pub.publish(self.go_pos_yaw_msg)

    def setZIControl(self, flag):
        self.z_i_control_msg.data = flag
        self.z_i_control_pub.publish(self.z_i_control_msg)

    def serArmFeedbackMode(self, flag):
        self.arm_feedback_msg.data = flag
        self.arm_feedback_pub.publish(self.arm_feedback_msg)

    def callMotion(self, num):
        self.call_motion_msg.data = num
        self.call_motion_pub.publish(self.call_motion_msg)
