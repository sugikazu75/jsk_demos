#!/usr/bin/env python

import sys
import time
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

rospy.init_node("state_transition_node")

z_offset_pub = rospy.Publisher("/pid/setoffset/z", Float32, queue_size=1)
z_i_control_pub = rospy.Publisher("/pid/i_control/z", Bool, queue_size=1)
pitch_gain_pub = rospy.Publisher("/pid/setgain/pitch", Vector3, queue_size=1)

target_state = rospy.get_param("~state", 0)

z_offset_msg = Float32()
z_i_control_msg = Bool()
pitch_gain_msg = Vector3()

if target_state == 0:     # lock state
    print("lock state")
    z_offset_msg.data = 6.0
    z_i_control_msg.data = False
    pitch_gain_msg.x = 20.0
    pitch_gain_msg.y = 1.0
    pitch_gain_msg.z = 4.0

elif target_state == 1:   # unlock state
    print("unlock state")
    z_offset_msg.data = 2.5
    z_i_control_msg.data = False
    pitch_gain_msg.x = 40.0
    pitch_gain_msg.y = 1.0
    pitch_gain_msg.z = 4.0

time.sleep(0.6)

z_offset_pub.publish(z_offset_msg)
z_i_control_pub.publish(z_i_control_msg)
pitch_gain_pub.publish(pitch_gain_msg)
