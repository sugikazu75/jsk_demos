#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from smach import State, StateMachine
import smach_ros
import time
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float32

class boundingBoxTransformPublisher:
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()

        self.bounding_boxes = BoundingBoxArray()
        self.bounding_box_sub = rospy.Subscriber('HSI_color_filter/boxes', BoundingBoxArray, self.boundingBoxCallback)

    def boundingBoxCallback(self, msg):
        self.bounding_boxes = msg
        if len(self.bounding_boxes.boxes) > 0:
            if self.bounding_boxes.boxes[0].dimensions.x > 0.1 and self.bounding_boxes.boxes[0].dimensions.y > 0.1:  # filter by size
                self.broadcaster.sendTransform((self.bounding_boxes.boxes[0].pose.position.x,
                                                self.bounding_boxes.boxes[0].pose.position.y,
                                                self.bounding_boxes.boxes[0].pose.position.z),
                                               (self.bounding_boxes.boxes[0].pose.orientation.x,
                                                self.bounding_boxes.boxes[0].pose.orientation.y,
                                                self.bounding_boxes.boxes[0].pose.orientation.z,
                                                self.bounding_boxes.boxes[0].pose.orientation.w),
                                               rospy.Time.now(),
                                               'box',
                                               self.bounding_boxes.header.frame_id)


class findBoundingBox(State):
    def __init__(self):
        State.__init__(self, outcomes=['found_on_ground', 'found_in_air', 'not_found'])
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)
        self.searching_count = 0
        self.found_count = 0
        self.average_num = 20
        self.trans_w_all = np.zeros((self.average_num, 3))
        self.trans_w = None
        self.trans_b_all = np.zeros((self.average_num, 3))
        self.trans_b = None
        self.ground_height_thresh = 0.5

    def execute(self, userdata):
        self.count = 0
        while(True):
            self.count = self.count + 1
            try:
                (trans_w, rot_w) = self.listener.lookupTransform('world', 'box', rospy.Time(0))
                (trans_b, rot_b) = self.listener.lookupTransform('camera_center', 'box', rospy.Time(0))
                self.trans_w_all[self.found_count] = trans_w
                self.trans_b_all[self.fount_count] = trans_b
                self.found_count = self.found_count + 1
                if self.found_count > self.average_num:
                    self.trans_w = np.average(self.trans_w_all, axis=0)
                    self.trans_b = np.average(self.trans_b_all, axis=0)
                    rospy.set_param("target_pos_w", self.trans_w)
                    rospy.set_param("target_pos_b", self.trans_b)
                    if self.trans_w[2] < self.ground_height_thresh:
                        return 'found_on_ground'
                    else:
                        return 'found_in_air'

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.found_count = 0
                if self.count > 100:
                    return 'not_found'

            self.rate.sleep()


class approachBoundingBox(State):
    def __init__(self):
        State.__init__(self, outcomes=['done', 'search'])
        self.mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.mocapCallback)
        self.odom = None
        self.target_pos_w = rospy.get_param('target_pos_w')
        self.target_pos_b = rospy.get_param('target_pos_b')
        self.distance_thresh = 0.5

        self.go_pos_x_msg = Float32()
        self.go_pos_y_msg = Float32()
        self.go_pos_z_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)
        self.go_pos_z_pub = rospy.Publisher('go_pos/z', Float32, queue_size=1)

    def execute(self, userdata):
        self.odom = None
        while self.odom is None:
            pass
        if self.target_pos_b[2] > self.distance_thresh:
            self.go_pos_x_msg.data = (self.odom.pose.position.x + self.target_pos_w[0]) / 2
            self.go_pos_y_msg.data = (self.odom.pose.position.y + self.target_pos_w[1]) / 2
            self.go_pos_z_msg.data = (self.odom.pose.position.z + self.target_pos_w[2]) / 2
            self.go_pos_x_pub.publish(self.go_pos_x_msg)
            self.go_pos_y_pub.publish(self.go_pos_y_msg)
            self.go_pos_z_pub.publish(self.go_pos_z_msg)
            return 'search'
        else:
            go_pos_x(self.target_pos[0])
            go_pos_y(self.target_pos[1])
            go_pos_z(self.target_pos[2])
            return 'done'

        # theta = - atan(x / z)

    def mocapCallback(self, msg):
        self.odom = msg

class gridSearch(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'not_found'])
        self.listener = tf.TransformListener()
        self.go_pos_x_msg = Float32()
        self.go_pos_y_msg = Float32()
        self.go_pos_x_pub = rospy.Publisher('go_pos/x', Float32, queue_size=1)
        self.go_pos_y_pub = rospy.Publisher('go_pos/y', Float32, queue_size=1)

        ############################
        self.safety_x_max = 0.9
        self.safety_x_min = -0.6
        self.safety_y_max = 0.5
        self.safety_y_min = -0.5
        self.x_step = 0.2
        self.y_step = 0.2
        self.x_direction = 1
        self.y_direction = 1
        self.direction_index = 0
        self.converged_thresh = 0.1
        ############################

        self.target_point = np.array((0.0, 0.0))
        self.current_point = np.zeros(2)

        self.odom = PoseStamped()
        self.odom_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.odomCallback)

    def clamp(self, value, min_, max_):
        return min(max(min_, value), max_)

    def calcNextPoint(self):
        # move to x
        if self.direction_index == 0:
            self.target_point[0] = self.clamp(self.target_point[0] + self.x_direction * self.x_step, self.safety_x_min, self.safety_x_max)

            # corner cases
            if self.target_point[0] == self.safety_x_max:
                self.x_direction = -1
                self.direction_index = 1

            if self.target_point[0] == self.safety_x_min:
                self.x_direction = 1
                self.direction_index = 1

        # move to y
        elif self.direction_index == 1:
            self.target_point[1] = self.clamp(self.target_point[1] + self.y_direction * self.y_step, self.safety_y_min, self.safety_y_max)
            self.direction_index = 0  # x move mode

            # corner cases
            if self.target_point[1] == self.safety_y_max:
                self.y_direction = -1
            if self.target_point[1] == self.safety_y_min:
                self.y_direction = 1

    def publishNextPoint(self):
        self.calcNextPoint()
        self.go_pos_x_msg.data = self.target_point[0]
        self.go_pos_y_msg.data = self.target_point[1]
        self.go_pos_x_pub.publish(self.go_pos_x_msg)
        self.go_pos_y_pub.publish(self.go_pos_y_msg)
        print('publish=[{}, {}]'.format(self.target_point[0], self.target_point[1]))

    def isPointConverged(self):
        return np.linalg.norm(self.target_point - self.current_point) < self.converged_thresh

    def execute(self, userdata):
        while self.odom is None:
            pass

        self.publishNextPoint()
        while not self.isPointConverged():
            pass
        print("search box")
        try:
            (trans_w, rot_w) = self.listener.lookupTransform('world', 'box', rospy.Time(0))
            return 'found'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return 'not_found'

    def odomCallback(self, msg):
        self.odom = msg
        self.current_point[0] = msg.pose.position.x
        self.current_point[1] = msg.pose.position.y


if __name__ == '__main__':
    rospy.init_node('bounding_box_transform_publisher')
    node = boundingBoxTransformPublisher()
    rospy.spin()
