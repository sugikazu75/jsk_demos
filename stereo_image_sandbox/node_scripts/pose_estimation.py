#!/usr/bin/env python3

import importlib.util
import os
import sys

from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image


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



def mod(a, b):
    """find a % b"""
    floored = np.floor_divide(a, b)
    return np.subtract(a, np.multiply(floored, b))

def sigmoid(x):
    """apply sigmoid actiation to numpy array"""
    return 1/ (1 + np.exp(-x))

def sigmoid_and_argmax2d(inputs, threshold, interpreter, width, height):
    """return y,x coordinates from heatmap"""
    #v1 is 9x9x17 heatmap
    # v1 = interpreter.get_tensor(output_details[0]['index'])[0]
    v1 = interpreter.get_tensor(inputs[0]['index'])[0]
    height = v1.shape[0]
    width = v1.shape[1]
    depth = v1.shape[2]
    reshaped = np.reshape(v1, [height * width, depth])
    reshaped = sigmoid(reshaped)
    #apply threshold
    reshaped = (reshaped > threshold) * reshaped
    coords = np.argmax(reshaped, axis=0)
    yCoords = np.round(np.expand_dims(np.divide(coords, width), 1))
    xCoords = np.expand_dims(mod(coords, width), 1)
    return np.concatenate([yCoords, xCoords], 1)

def get_offset_point(y, x, offsets, keypoint, num_key_points):
    """get offset vector from coordinate"""
    y_off = offsets[y,x, keypoint]
    x_off = offsets[y,x, keypoint+num_key_points]
    return np.array([y_off, x_off])


def get_offsets(output_details, coords, interpreter, num_key_points=17):
    """get offset vectors from all coordinates"""
    offsets = interpreter.get_tensor(output_details[1]['index'])[0]
    offset_vectors = np.array([]).reshape(-1,2)
    for i in range(len(coords)):
        heatmap_y = int(coords[i][0])
        heatmap_x = int(coords[i][1])
        #make sure indices aren't out of range
        if heatmap_y >8:
            heatmap_y = heatmap_y -1
        if heatmap_x > 8:
            heatmap_x = heatmap_x -1
        offset_vectors = np.vstack((offset_vectors, get_offset_point(heatmap_y, heatmap_x, offsets, i, num_key_points)))
    return offset_vectors

def draw_lines(keypoints, image, bad_pts):
    """connect important body part keypoints with lines"""
    #color = (255, 0, 0)
    color = (0, 255, 0)
    thickness = 2
    #refernce for keypoint indexing: https://www.tensorflow.org/lite/models/pose_estimation/overview
    body_map = [[5,6], [5,7], [7,9], [5,11], [6,8], [8,10], [6,12], [11,12], [11,13], [13,15], [12,14], [14,16]]
    for map_pair in body_map:
        #print(f'Map pair {map_pair}')
        if map_pair[0] in bad_pts or map_pair[1] in bad_pts:
            continue
        start_pos = (int(keypoints[map_pair[0]][1]), int(keypoints[map_pair[0]][0]))
        end_pos = (int(keypoints[map_pair[1]][1]), int(keypoints[map_pair[1]][0]))
        image = cv2.line(image, start_pos, end_pos, color, thickness)
    return image


class PoseEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(PoseEstimation, self).__init__()

        pkg = importlib.util.find_spec('tensorflow')
        if pkg is None:
            from tflite_runtime.interpreter import Interpreter
        else:
            from tensorflow.lite.python.interpreter import Interpreter

        self.min_conf_threshold = rospy.get_param('~threshold', 0.5)
        model_path = rospy.get_param('~model_path')
        interpreter = Interpreter(model_path=model_path)
        interpreter.allocate_tensors()

        self.input_details = interpreter.get_input_details()
        self.output_details = interpreter.get_output_details()
        self.interpreter = interpreter
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        #set stride to 32 based on model size

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        self.bridge = CvBridge()
        self.pub_img = self.advertise(
            '~output/viz', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        bridge = self.bridge

        min_conf_threshold = self.min_conf_threshold
        output_stride = 32
        input_mean = 127.5
        input_std = 127.5
        interpreter = self.interpreter
        input_details = self.input_details
        output_details = self.output_details
        width, height = self.width, self.height

        frame = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        input_height, input_width, _ = frame.shape
        frame_rgb = frame.copy()
        frame_rgb = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        scale_width = width / input_width
        scale_height = height / input_height
        input_data = np.expand_dims(frame_resized, axis=0)

        frame_resized = cv2.cvtColor(frame_resized, cv2.COLOR_RGB2BGR)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        #get y,x positions from heatmap
        coords = sigmoid_and_argmax2d(output_details, min_conf_threshold,
                                      interpreter, width, height)
        #keep track of keypoints that don't meet threshold
        drop_pts = list(np.unique(np.where(coords ==0)[0]))
        #get offets from postions
        offset_vectors = get_offsets(output_details, coords, interpreter)
        #use stide to get coordinates in image coordinates
        keypoint_positions = coords * output_stride + offset_vectors
        keypoint_positions[:, 1] /= scale_width
        keypoint_positions[:, 0] /= scale_height

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(keypoint_positions)):
            #don't draw low confidence points
            if i in drop_pts:
                continue
            # Center coordinates
            x = int(keypoint_positions[i][1])
            y = int(keypoint_positions[i][0])
            center_coordinates = (x, y)
            radius = 2
            color = (0, 255, 0)
            thickness = 2
            cv2.circle(frame, center_coordinates, radius, color, thickness)

        frame_resized = draw_lines(keypoint_positions, frame, drop_pts)

        out_img_msg = bridge.cv2_to_imgmsg(
            frame, encoding='bgr8')
        out_img_msg.header = img_msg.header
        self.pub_img.publish(out_img_msg)


if __name__ == '__main__':
    rospy.init_node('pose_estimation')
    node = PoseEstimation()
    rospy.spin()
