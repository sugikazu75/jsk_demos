#!/bin/bash

echo "record image data"
rosrun kxr_flight_unit_control rosbag_all_data.sh /stereo/image_raw/compressed /stereo/left_camera/image_raw/compressed /stereo/left_camera/image_rect_color/compressed /stereo/right_camera/image_raw/compressed /stereo/right_camera/image_rect_color/compressed ${@:1}
