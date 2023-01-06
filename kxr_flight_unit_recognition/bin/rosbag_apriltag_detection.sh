#!/bin/bash

echo "record apriltag detection data"
rosrun kxr_flight_unit_control rosbag_image_data.sh /stereo/left_camera/tag_detections /stereo/left_camera/tag_detections_image/compressed /stereo/right_camera/image_raw/compressed /stereo/right_camera/image_rect_color/compressed /stereo/right_camera/tag_detections /stereo/right_camera/tag_detections_image/compressed ${@:1}
