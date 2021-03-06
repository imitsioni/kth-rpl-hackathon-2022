#!/usr/bin/env python

from geometry_utils import *
from moveit_msgs.msg import OrientedBoundingBox  # pip install moveit. TBC with header
from geometry_msgs.msg import Point, PointStamped
import ctypes
import struct
import copy
import numpy as np


class box_geometry:

    def __init__(self) -> None:
        rospy.init_node('box_geometry')
        self.rate = rospy.Rate(
            0.5)  # publish message at 0.5 Hz, still not good enough
        # self.sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback, queue_size=1)

        self.sub = rospy.Subscriber("/camera_kinect/depth/points",
                                    PointCloud2,
                                    self.callback,
                                    queue_size=1)
        self.geometry_pub = rospy.Publisher("bbox/geometry",
                                            OrientedBoundingBox,
                                            queue_size=1)
        """
        self.pub_corner0 = rospy.Publisher("bbox/corner0", Point, queue_size=1)
        self.pub_corner1 = rospy.Publisher("bbox/corner1", Point, queue_size=1)
        self.pub_corner2 = rospy.Publisher("bbox/corner2", Point, queue_size=1)
        self.pub_corner3 = rospy.Publisher("bbox/corner3", Point, queue_size=1)
        
        

        self.pub_corner0 = rospy.Publisher("bbox/corner0", PointStamped, queue_size=1)
        self.pub_corner1 = rospy.Publisher("bbox/corner1", PointStamped, queue_size=1)
        self.pub_corner2 = rospy.Publisher("bbox/corner2", PointStamped, queue_size=1)
        self.pub_corner3 = rospy.Publisher("bbox/corner3", PointStamped, queue_size=1)
        """
        self.pub_corner0_world = rospy.Publisher("bbox_world/corner0",
                                                 PointStamped,
                                                 queue_size=1)
        self.pub_corner1_world = rospy.Publisher("bbox_world/corner1",
                                                 PointStamped,
                                                 queue_size=1)
        self.pub_corner2_world = rospy.Publisher("bbox_world/corner2",
                                                 PointStamped,
                                                 queue_size=1)
        self.pub_corner3_world = rospy.Publisher("bbox_world/corner3",
                                                 PointStamped,
                                                 queue_size=1)

        # buffer
        self.buffer_surface = []
        self.buffer_box_geometry = []
        self.buffer_center = []

        # rospy.spin()
        # self.listener = tf.TransformListener()
        self.rate.sleep()

    def transform_to_world(self, pos: np.array, stamp=0):
        """
        # find the transform between 
        self.listener.waitForTransform('/world','/camera_depth_optical_frame', stamp, rospy.Duration(1) )
        t = self.listener.getLatestCommonTime('/world', '/camera_depth_optical_frame')
        (trans,rotm) = self.listener.lookupTransform('/world', '/camera_depth_optical_frame', t)
        print(trans,rotm)
        """
        trans = [0.2184907002920966, 0.10004434430211705, 0.2282298046491542]
        # rotm = [-0.6358874578790518, 0.6328879887381128, -0.3349407207381798, 0.2879490375301368]
        # rotm = quaternion_matrix(rotm)
        # rotm = rotm[0:3, 0:3]
        rotm = np.array([[-0.02546499, -0.61199935, 0.79044818],
                         [-0.99778278, -0.03307629, -0.05775356],
                         [0.06149023, -0.79016628, -0.60980013]])

        new_pos = trans + rotm.dot(pos)
        # return Point(new_pos[0], new_pos[1], new_pos[2])
        return new_pos

    def convert_to_point(self, pos: np.array) -> Point:
        return Point(pos[0], pos[1], pos[2])

    def callback(self, data: PointCloud2) -> None:
        o3d_pcl = convertCloudFromRosToOpen3d(data)
        bbox = OrientedBoundingBox()
        geometry, center, top_surface_corners = get_width_length_height(
            o3d_pcl, max_plane_idx=2, view=False)

        self.buffer_box_geometry.append(geometry)
        self.buffer_center.append(center)
        self.buffer_surface.append(top_surface_corners)

        # keep the buffer size <= 20
        if len(self.buffer_box_geometry) == 21:
            self.buffer_box_geometry.pop(0)
            self.buffer_center.pop(0)
            self.buffer_surface.pop(0)

        assert len(self.buffer_box_geometry) == len(self.buffer_center) == len(
            self.buffer_surface)

        # only publish the topic when the buffer saves 20 timestamps
        if len(self.buffer_box_geometry) == 20:
            buffer_box_geometry_numpy = np.array(self.buffer_box_geometry)
            buffer_center_numpy = np.array(self.buffer_center)
            buffer_surface_numpy = np.array(self.buffer_surface)
            geometry = np.median(buffer_box_geometry_numpy, axis=0)
            center = np.median(buffer_center_numpy, axis=0)
            top_surface_corners = np.median(buffer_surface_numpy, axis=0)

            # bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z = center[0], center[1], center[2]
            center_world = self.transform_to_world(
                center)  # transform the box center to the world frame
            bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z = center_world[
                0], center_world[1], center_world[2]
            bbox.extents.x, bbox.extents.y, bbox.extents.z = geometry[
                0] + 0.01, geometry[1] + 0.01, geometry[2]

            # cor0 = Point(top_surface_corners[0,0], top_surface_corners[0,1], top_surface_corners[0,2])
            # cor1 = Point(top_surface_corners[1,0], top_surface_corners[1,1], top_surface_corners[1,2])
            # cor2 = Point(top_surface_corners[2,0], top_surface_corners[2,1], top_surface_corners[2,2])
            # cor3 = Point(top_surface_corners[3,0], top_surface_corners[3,1], top_surface_corners[3,2])

            cor_world = np.array([
                self.transform_to_world(top_surface_corners[x])
                for x in range(top_surface_corners.shape[0])
            ])
            # y_increasing_ind = np.argsort(cor_world, axis=0) # increasing order
            y_increasing = np.argsort(cor_world[:, 1])
            cor_world = cor_world[y_increasing]
            cor_world[:, 2] = bbox.pose.position.z
            threshold = 0.01
            if cor_world[1, 1] - cor_world[0, 1] > threshold:
                cor0_world = cor_world[0]
                cor3_world = cor_world[3]
                cor1_world = cor_world[2] if (
                    cor_world[1, 0] < cor_world[2, 0]) else cor_world[1]
                cor2_world = cor_world[2] if (
                    cor_world[1, 0] > cor_world[2, 0]) else cor_world[1]
            else:
                cor2_world = cor_world[0] if (
                    cor_world[0, 0] < cor_world[1, 0]) else cor_world[1]
                cor0_world = cor_world[0] if (
                    cor_world[0, 0] > cor_world[1, 0]) else cor_world[1]
                cor1_world = cor_world[3] if (
                    cor_world[2, 0] < cor_world[3, 0]) else cor_world[2]
                cor3_world = cor_world[3] if (
                    cor_world[2, 0] > cor_world[3, 0]) else cor_world[2]

            world_frame = copy.copy(data.header)
            world_frame.frame_id = 'world'
            stamp = data.header.stamp

            cor0_world = PointStamped(world_frame,
                                      self.convert_to_point(cor0_world))
            cor1_world = PointStamped(world_frame,
                                      self.convert_to_point(cor1_world))
            cor2_world = PointStamped(world_frame,
                                      self.convert_to_point(cor2_world))
            cor3_world = PointStamped(world_frame,
                                      self.convert_to_point(cor3_world))

            self.geometry_pub.publish(bbox)
            # self.pub_corner0.publish(PointStamped(data.header, cor0))
            # self.pub_corner1.publish(PointStamped(data.header, cor1))
            # self.pub_corner2.publish(PointStamped(data.header, cor2))
            # self.pub_corner3.publish(PointStamped(data.header, cor3))

            self.pub_corner0_world.publish(cor0_world)
            self.pub_corner1_world.publish(cor1_world)
            self.pub_corner2_world.publish(cor2_world)
            self.pub_corner3_world.publish(cor3_world)

            rospy.loginfo([bbox.extents.x, bbox.extents.y, bbox.extents.z])
            # rospy.loginfo(top_surface_corners)


if __name__ == '__main__':
    box_geometry()
    rospy.spin()
