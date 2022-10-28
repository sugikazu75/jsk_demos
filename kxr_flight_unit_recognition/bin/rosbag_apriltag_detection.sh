#!/bin/bash

echo "record apriltag detection data"
rosrun kxr_flight_unit_control rosbag_birotor_all_data.sh /stereo/image_raw/compressed /stereo/left_camera/image_raw/compressed /stereo/left_camera/image_rect_color/compressed /stereo/left_camera/tag_detections /stereo/left_camera/tag_detections_image/compressed /stereo/right_camera/image_raw/compressed /stereo/right_camera/image_rect_color/compressed /stereo/right_camera/tag_detections /stereo/right_camera/tag_detections_image/compressed ${@:1}
