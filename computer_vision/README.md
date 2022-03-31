# kth-rpl-hackathon-2022 computer vision
Repository hosting the code for the 2022 Robotics, Perception and Learning hackathon

## Requirements
- Open3D: version used was 0.9.0, it supports python 2.7, 3.5, 3.6, 3.7
- ROS  (to install ROS please check the other folder where the state_machine ROS package resisdes)

## Installation
1. Open a terminal and install open3d, matplotlib, pyyaml and rospkg
```
pip install open3d==0.9.0 matplotlib pyyaml rospkg
```
2. Give permissions for <b>get_box_geo.py</b> as an executable
```
chmod +x src/get_box_geo.py
./get_box_geo.py
```

## Purpose of the script
The script creates 5 topics which are essential for the main state machine to operate, these topics are:
- <i>"bbox/geometry"</i>: publishes messages of the type <b>OrientedBoundingBox</b> from <b>moveit_msgs.msg</b> which correspondes to the box to be folded
- 4 topics corresponding to the corners of the cloth, namely: <i>"bbox/corner0"</i>, <i>"bbox/corner1"</i>, <i>"bbox/corner2"</i>, <i>"bbox/corner3"</i> all of the type <b>PointStamped</b> from <b>geometry_msgs.msg</b>