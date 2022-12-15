#!/bin/bash

echo "record hsi color filter data"
rosrun kxr_flight_unit_recognition rosbag_image_data.sh /HSI_color_filter/boxes /HSI_color_filter/cluster_decomposer/centroid_pose_array /HSI_color_filter/cluster_decomposer/mask
