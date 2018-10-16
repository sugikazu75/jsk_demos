#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import time

import cv2
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class ImagesSaver(object):

    SAVE_DIRECTORY = '~/.ros/images_saver'

    def __init__(self):
        super(ImagesSaver, self).__init__()
        self._service = rospy.Service("images_saver", Empty, self.save_images)
        self.bridge = CvBridge()

        self.subscribe()
        if not os.path.exists(os.path.expanduser(self.SAVE_DIRECTORY)):
            os.mkdir(os.path.expanduser(self.SAVE_DIRECTORY))

    def subscribe(self):
        self._sub1 = message_filters.Subscriber("~input/src1", Image)
        self._sub2 = message_filters.Subscriber("~input/src2", Image)
        self._sub3 = message_filters.Subscriber("~input/src3", Image)

        use_async = rospy.get_param("~approximate_sync", False)
        queue_size = rospy.get_param("~queue_size", 100)
        subs = [self._sub1, self._sub2, self._sub3]
        if use_async:
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(subs, queue_size)
        sync.registerCallback(self._save_buffer)

    def _save_buffer(self, img_msg1, img_msg2, img_msg3):
        self.img1 = self.bridge.imgmsg_to_cv2(img_msg1, "bgr8")
        self.img2 = self.bridge.imgmsg_to_cv2(img_msg2, "bgr8")
        self.img3 = self.bridge.imgmsg_to_cv2(img_msg3, "bgr8")

    def save_images(self, req):
        time_stamp = time.strftime("%Y-%m-%d-%H-%M-%S")
        cv2.imwrite(
            os.path.join(
                os.path.expanduser(self.SAVE_DIRECTORY),
                "{}_img1.jpg".format(time_stamp)), self.img1)
        cv2.imwrite(
            os.path.join(
                os.path.expanduser(self.SAVE_DIRECTORY),
                "{}_img2.jpg".format(time_stamp)), self.img2)
        cv2.imwrite(
            os.path.join(
                os.path.expanduser(self.SAVE_DIRECTORY),
                "{}_img3.jpg".format(time_stamp)), self.img3)
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("images_saver")
    i = ImagesSaver()
    rospy.spin()
