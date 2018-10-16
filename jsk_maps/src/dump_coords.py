#!/usr/bin/env python

from math import asin
from math import atan2

import numpy as np
import rospy
import tf
import tf2_ros


def quaternion2rpy(q):
    """
    Roll-pitch-yaw angles of a quaternion.

    Parameters
    ----------
    quat : (4,) array
        Quaternion in `[w x y z]` format.

    Returns
    -------
    rpy : (3,) array
        Array of yaw-pitch-roll angles, in [rad].
    """
    roll = atan2(
        2 * q[2] * q[3] + 2 * q[0] * q[1],
        q[3] ** 2 - q[2] ** 2 - q[1] ** 2 + q[0] ** 2)
    pitch = -asin(
        2 * q[1] * q[3] - 2 * q[0] * q[2])
    yaw = atan2(
        2 * q[1] * q[2] + 2 * q[0] * q[3],
        q[1] ** 2 + q[0] ** 2 - q[3] ** 2 - q[2] ** 2)
    rpy = np.array([yaw, pitch, roll])
    return rpy, np.pi - rpy


rospy.init_node("dump_coords")

listener = tf.TransformListener()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        (trans, q) = listener.lookupTransform(
            '/eng8/wrs',
            '/interactive_marker', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException,
            tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,):
        continue
    rpy = quaternion2rpy(q)[0]
    print("     `(:rpy #f({} 0.0 0.0)".
          format(rpy[2]))
    print('        :pos ,(float-vector {} {} 0)'.
          format(trans[0] * 1000.0, trans[1] * 1000))
    print('        :name "/eng8/wrs/")')
    rate.sleep()
