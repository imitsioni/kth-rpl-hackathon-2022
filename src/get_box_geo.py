#!/usr/bin/env python

from geometry_utils import *
from moveit_msgs.msg import OrientedBoundingBox # pip install moveit. TBC with header
import ctypes
import struct



class box_geometry:
    def __init__(self) -> None:
        rospy.init_node('box_geometry')
        self.rate = rospy.Rate(0.5)  # publish message at 0.5 Hz, still not good enough
        # self.sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback, queue_size=1)
        self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher("bbox/geometry", OrientedBoundingBox, queue_size=1)
        # rospy.spin()
        self.rate.sleep()  

    def callback(self, data: PointCloud2) -> None:
        o3d_pcl = convertCloudFromRosToOpen3d(data)
        bbox = OrientedBoundingBox()
        geometry, center, top_surface_corners = get_width_length_height(o3d_pcl, max_plane_idx=2, view=False)
        rospy.loginfo(geometry)
        bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z = center[0], center[1], center[2]
        bbox.extents.x, bbox.extents.y, bbox.extents.z = geometry[0], geometry[1], geometry[2]
        self.pub.publish(bbox)



if __name__ == '__main__':
    box_geometry()
    rospy.spin()