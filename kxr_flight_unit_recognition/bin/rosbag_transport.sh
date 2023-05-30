#!/bin/bash

echo "record object pose"
rosrun kxr_flight_unit_recognition rosbag_apriltag_detection.sh /object/mocap/pose ${@:1}

