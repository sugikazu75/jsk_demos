# stereo image sandbox

```
mkdir -p ~/ros/stereo_ws/src
wget https://raw.githubusercontent.com/iory/jsk_demos/stereo/stereo_image_sandbox/rosinstall -O ~/ros/stereo_ws/src/.rosinstall
cd ~/ros/stereo_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros/stereo_ws/src
wstool up
cd ~/ros/stereo_ws
rosdep update
rosdep install --from-paths -i -y -r .
catkin b stereo_image_sandbox -j4
```
