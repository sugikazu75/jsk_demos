import rospy
import smach
import tf
import numpy as np
import time

from geometry_msgs.msg import PoseStamped, Vector3
from kxr_rosserial_msgs.msg import PIDControllerState6D, FlightState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, Int16, Empty

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
        self.robot.resetForceSkip()
        rospy.loginfo("wait to start task")
        while not (self.task_start or self.robot.getForceSkip()):
            rospy.sleep(0.1)

            if rospy.is_shutdown():
                return 'preempted'

        rospy.loginfo_once('task start!')
        time.sleep(1.0)
        self.robot.resetForceSkip();
        return 'succeeded'

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

        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)
        self.go_pos_roll_pub = rospy.Publisher('go_pos/roll', Float32, queue_size=1)
        self.go_pos_pitch_pub = rospy.Publisher('go_pos/pitch', Float32, queue_size=1)
        self.go_pos_yaw_pub = rospy.Publisher('go_pos/yaw', Float32, queue_size=1)

        self.z_i_control_pub = rospy.Publisher('pid/i_control/z', Bool, queue_size=1)
        self.z_offset_pub = rospy.Publisher('pid/setoffset/z',  Float32, queue_size=1)

        self.pitch_gain_pub = rospy.Publisher('pid/setgain/pitch', Vector3, queue_size=1)

        self.set_control_mode_pub = rospy.Publisher('set_control_mode', Bool, queue_size=1)

        self.set_motion_flag_pub = rospy.Publisher("motion_flag", Bool, queue_size=1)
        self.call_motion_pub = rospy.Publisher("call_motion", Int16, queue_size=1)
        self.docking_pub = rospy.Publisher("docking_mode", Bool, queue_size=1)

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
        '''
        NONE = 0
        ARM_OFF = 1
        ARM_ON = 2
        TAKEOFF = 3
        HOVERING = 4
        LANDING = 5
        '''
        self.flight_state = msg.state

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
        self.force_skip_flag = False

    def goPosX(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_x_pub.publish(msg)

    def goPosY(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_y_pub.publish(msg)

    def goPosZ(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_z_pub.publish(msg)

    def goPosRoll(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_roll_pub.publish(msg)

    def goPosPitch(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_pitch_pub.publish(msg)

    def goPosYaw(self, target):
        msg = Float32()
        msg.data = target
        self.go_pos_yaw_pub.publish(msg)

    def setZIControl(self, flag):
        msg = Bool()
        msg.data = flag
        self.z_i_control_pub.publish(msg)

    def setZOffset(self, offset):
        msg = Float32()
        msg.data = offset
        self.z_offset_pub.publish(msg)

    def setPitchGain(self, p, i, d):
        msg = Vector3()
        msg.x = p
        msg.y = i
        msg.z = d
        self.pitch_gain_pub.publish(msg)

    def setControlMode(self, flag):
        msg = Bool()
        msg.data = flag
        self.set_control_mode_pub.publish(msg)

    def setMotionFlag(self, flag):
        msg= Bool()
        msg.data = flag
        self.set_motion_flag_pub.publish(msg)

    def callMotion(self, num):
        msg = Int16()
        msg.data = num
        self.call_motion_pub.publish(msg)

    def setDockingMode(self, flag):
        msg = Bool()
        msg.data = flag
        self.docking_pub.publish(msg)

