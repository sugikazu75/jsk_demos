#!/usr/bin/env python

from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
from cameramodels import PinholeCameraModel
import rospy
from sensor_msgs.msg import CameraInfo
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import PeoplePose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment
import message_filters
from scipy import linalg


try:
    from sklearn.utils.linear_assignment_ import linear_assignment
except:
    from scipy.optimize import linear_sum_assignment as linear_assignment


def DLT(P1, P2, point1, point2):
    A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:],]
    A = np.array(A).reshape((4,4))
    B = np.dot(A.transpose(), A)
    U, s, Vh = linalg.svd(B, full_matrices=False)
    return Vh[3, 0:3] / Vh[3, 3]


class StereoKeypointMatching(ConnectionBasedTransport):

    def __init__(self):
        super(StereoKeypointMatching, self).__init__()

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

        self.pose_pub = self.advertise(
            '~output/pose', PeoplePoseArray, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 30)
        sub_left_camera_info = message_filters.Subscriber(
            '~left/camera_info',
            CameraInfo, queue_size=1)
        sub_right_camera_info = message_filters.Subscriber(
            '~right/camera_info',
            CameraInfo, queue_size=1)
        sub_right_poses = message_filters.Subscriber(
            '~right/pose',
            PeoplePoseArray, queue_size=1)
        sub_left_poses = message_filters.Subscriber(
            '~left/pose',
            PeoplePoseArray, queue_size=1)
        self.subs = [
            sub_left_camera_info,
            sub_right_camera_info,
            sub_left_poses,
            sub_right_poses,
        ]
        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, left_camera_info, right_camera_info,
                 left_hand, right_hand):
        left_camera = PinholeCameraModel.from_camera_info(
            left_camera_info)
        right_camera = PinholeCameraModel.from_camera_info(
            right_camera_info)

        limbs = []
        for pose in left_hand.poses:
            limbs.extend(pose.limb_names)
        for pose in right_hand.poses:
            limbs.extend(pose.limb_names)
        limbs = list(set(limbs))

        left_poses = []
        for pose in left_hand.poses:
            limb_to_pose = {}
            for name, pose in zip(pose.limb_names, pose.poses):
                limb_to_pose[name] = np.array(
                    [pose.position.x, pose.position.y])
            left_poses.append(limb_to_pose)
        right_poses = []
        for pose in right_hand.poses:
            limb_to_pose = {}
            for name, pose in zip(pose.limb_names, pose.poses):
                limb_to_pose[name] = np.array(
                    [pose.position.x, pose.position.y])
            right_poses.append(limb_to_pose)

        dists = np.zeros((len(left_hand.poses), len(right_hand.poses)),
                         dtype=np.float64)
        for i, left_pose in enumerate(left_poses):
            for j, right_pose in enumerate(right_poses):
                dist = 0
                for name in limbs:
                    if not (name in left_pose and name in right_pose):
                        continue
                    left_xy = left_pose[name]
                    right_xy = right_pose[name]
                    point = DLT(left_camera.P, right_camera.P, left_xy, right_xy)
                    x, y = left_camera.project3d_to_pixel(point)
                    dist += np.sqrt((left_xy[0] - x) ** 2 + (left_xy[1] - y) ** 2)
                    x, y = right_camera.project3d_to_pixel(point)
                    dist += np.sqrt((right_xy[0] - x) ** 2 + (right_xy[1] - y) ** 2)
                dists[i, j] = dist

        left_indices, right_indices = linear_assignment(-dists)

        out_msg = PeoplePoseArray(header=left_camera_info.header)
        hand_list = []
        for left_index, right_index in zip(left_indices, right_indices):
            left_pose = left_poses[left_index]
            right_pose = right_poses[right_index]
            pose_msg = PeoplePose()
            hand = {}
            for name in limbs:
                if not (name in left_pose and name in right_pose):
                    continue
                left_xy = left_pose[name]
                right_xy = right_pose[name]
                x, y, z = DLT(left_camera.P, right_camera.P, left_xy, right_xy)
                u, v = right_camera.project3d_to_pixel((x, y, z))
                if not (0 <= u < right_camera.width and 0 <= v < right_camera.height):
                    continue
                if not (0 <= u < left_camera.width and 0 <= v < left_camera.height):
                    continue
                pose_msg.limb_names.append(name)
                pose_msg.poses.append(
                    Pose(position=Point(x=x, y=y, z=z),
                         orientation=Quaternion(w=1.0)))
                hand[name] = np.array([x, y, z])
            hand_list.append(hand)
            out_msg.poses.append(pose_msg)
        self.pose_pub.publish(out_msg)

        if self.skeleton_pub.get_num_connections() > 0:
            skeleton_msgs = HumanSkeletonArray(header=left_camera_info.header)
            for hand in hand_list:
                skeleton_msg = HumanSkeleton(header=left_camera_info.header)
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
            self.skeleton_pub.publish(skeleton_msgs)


if __name__ == '__main__':
    rospy.init_node('stereo_keypoint_matching')
    node = StereoKeypointMatching()
    rospy.spin()
