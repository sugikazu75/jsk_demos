#!/usr/bin/env python3

import os
import sys

import mediapipe as mp
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment


# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    from cv_bridge import CvBridge
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(ws_python3_paths[0])
    else:
        from cv_bridge import CvBridge


class HandPoseEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(HandPoseEstimation, self).__init__()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.INDEX2FINGERNAME = {
            self.mp_hands.HandLandmark.WRIST: "wrist",
            self.mp_hands.HandLandmark.THUMB_CMC: "thumb_mcp",
            self.mp_hands.HandLandmark.THUMB_MCP: "thumb_pip",
            self.mp_hands.HandLandmark.THUMB_IP: "thumb_dip",
            self.mp_hands.HandLandmark.THUMB_TIP: "thumb_tip",
            self.mp_hands.HandLandmark.INDEX_FINGER_MCP: "index_mcp",
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP: "index_pip",
            self.mp_hands.HandLandmark.INDEX_FINGER_DIP: "index_dip",
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP: "index_tip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP: "middle_mcp",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP: "middle_pip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP: "middle_dip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP: "middle_tip",
            self.mp_hands.HandLandmark.RING_FINGER_MCP: "ring_mcp",
            self.mp_hands.HandLandmark.RING_FINGER_PIP: "ring_pip",
            self.mp_hands.HandLandmark.RING_FINGER_DIP: "ring_dip",
            self.mp_hands.HandLandmark.RING_FINGER_TIP: "ring_tip",
            self.mp_hands.HandLandmark.PINKY_MCP: "little_mcp",
            self.mp_hands.HandLandmark.PINKY_PIP: "little_pip",
            self.mp_hands.HandLandmark.PINKY_DIP: "little_dip",
            self.mp_hands.HandLandmark.PINKY_TIP: "little_tip",
        }

        FINGERNAMES = [
            "wrist",
            "thumb_mcp",
            "thumb_pip",
            "thumb_dip",
            "thumb_tip",
            "index_mcp",
            "index_pip",
            "index_dip",
            "index_tip",
            "middle_mcp",
            "middle_pip",
            "middle_dip",
            "middle_tip",
            "ring_mcp",
            "ring_pip",
            "ring_dip",
            "ring_tip",
            "little_mcp",
            "little_pip",
            "little_dip",
            "little_tip",
        ]
        connections = []
        for i in range(5):
            connections.append((FINGERNAMES[0], FINGERNAMES[i * 4 + 1]))
            connections.append((FINGERNAMES[i * 4 + 1], FINGERNAMES[i * 4 + 2]))
            connections.append((FINGERNAMES[i * 4 + 2], FINGERNAMES[i * 4 + 3]))
            connections.append((FINGERNAMES[i * 4 + 3], FINGERNAMES[i * 4 + 4]))
        self.connections = connections

        self.hand_pose_estimator = self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        self.bridge = CvBridge()
        self.pub_img = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pose_pub = self.advertise('~output/pose',
                                       PeoplePoseArray, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        bridge = self.bridge

        mp_drawing = self.mp_drawing
        mp_hands = self.mp_hands

        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.hand_pose_estimator.process(image)

        people_pose_msg = PeoplePoseArray(header=img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)
        _PRESENCE_THRESHOLD = 0.5
        _VISIBILITY_THRESHOLD = 0.5
        if results.multi_hand_landmarks:
            hand_list = []
            for hand_landmarks in results.multi_hand_landmarks:
                pose_msg = PeoplePose()
                image_rows, image_cols, _ = image.shape
                hand = {}
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    if ((landmark.HasField('visibility') and
                         landmark.visibility < _VISIBILITY_THRESHOLD) or
                        (landmark.HasField('presence') and
                         landmark.presence < _PRESENCE_THRESHOLD)):
                        continue
                    landmark_px = self.mp_drawing._normalized_to_pixel_coordinates(
                        landmark.x, landmark.y,
                        image_cols, image_rows)
                    if landmark_px:
                        pose_msg.scores.append(landmark.visibility)
                        pose_msg.limb_names.append(self.INDEX2FINGERNAME[idx])
                        pose_msg.poses.append(
                            Pose(position=Point(x=landmark_px[0],
                                                y=landmark_px[1],
                                                z=0)))
                        hand[self.INDEX2FINGERNAME[idx]] = np.array([landmark_px[0],
                                                                     landmark_px[1],
                                                                     0.0])
                people_pose_msg.poses.append(pose_msg)
                hand_list.append(hand)

            for hand in hand_list:
                skeleton_msg = HumanSkeleton(header=img_msg.header)
                for a, b in self.connections:
                    if not (a in hand and b in hand):
                        continue
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*hand[a]),
                        end_point=Point(*hand[b]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)
                skeleton_msgs.skeletons.append(skeleton_msg)
        self.pose_pub.publish(people_pose_msg)
        self.skeleton_pub.publish(skeleton_msgs)

        if self.pub_img.get_num_connections() > 0:
            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            out_img_msg = bridge.cv2_to_imgmsg(
                image, encoding='bgr8')
            out_img_msg.header = img_msg.header
            self.pub_img.publish(out_img_msg)


if __name__ == '__main__':
    rospy.init_node('hand_pose_estimation')
    node = HandPoseEstimation()
    rospy.spin()
