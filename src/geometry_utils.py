import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import copy
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *

def read_pointcloud(path: str, view: bool = False) -> o3d.geometry.PointCloud:
    '''
    # read point cloud from .pcd or .ply file
    :param path: path to the file
    :param view: Add visualization
    :return: o3d.geometry.PointCloud

    # can use np.asarray(pcd.points) to get points
    '''
    pcd = o3d.io.read_point_cloud(path)
    if view:
        o3d.visualization.draw_geometries([pcd])
    return pcd

def init_pointcloud(points: np.array, colors: np.array = None, view: bool = False) -> o3d.geometry.PointCloud:
    """
    init pointcloud from np.array
    :param points: input point cloud [N,3]
    :param colors: colors of the point cloud [N,3]
    :param view: Add visualization
    :return: o3d.geometry.PointCloud
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    if view:
        o3d.visualization.draw_geometries([pcd])
    return pcd

def downsample(pcd: o3d.geometry.PointCloud, size: float = 0.01, view: bool = False) -> o3d.geometry.PointCloud:
    '''
    downsample point cloud
    :param pcd: input point cloud
    :param size: voxel size used to downsample the point cloud
    :param view: Add visualization
    :return:  o3d.geometry.PointCloud
    '''
    downpcd = pcd.voxel_down_sample(voxel_size=size)
    if view:
        o3d.visualization.draw_geometries([downpcd])
    return downpcd


def cluster(pcd: o3d.geometry.PointCloud, eps: float = 0.02, min_points: int = 10, view: bool = False) -> np.array:
    '''
    Cluster PointCloud using the DBSCAN algorithm and Visualization
    :param pcd: input point cloud
    :param eps: Density parameter that is used to find neighbouring points.
    :param min_points: Minimum number of points to form a cluster.
    :param view: Add visualization
    :return: np.array
    '''

    labels = np.array(pcd.cluster_dbscan(eps, min_points))
    if view:
        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        newdownpcd = o3d.geometry.PointCloud()
        newdownpcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
        newdownpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([newdownpcd])
    return labels


def get_largest_cluster(pcd: o3d.geometry.PointCloud, labels: np.array, view: bool = False) -> o3d.geometry.PointCloud:
    """
    select the cluster with largest number of points --> indicate the table and bbox area
    :param pcd: input point cloud
    :param labels: labels obtained from cluster
    :param view: Add visualization
    :return: o3d.geometry.PointCloud
    """
    count = [(labels == i).sum() for i in np.unique(labels)]
    max_labels = np.unique(labels)[np.argmax(count)]
    wanted_index = np.where(labels == max_labels)[0]
    wanted_pcd = pcd.select_down_sample(wanted_index)
    if view:
        o3d.visualization.draw_geometries([wanted_pcd])
    return wanted_pcd


def find_plane(pcd: o3d.geometry.PointCloud, max_plane_idx: int = 2, distance_threshold: float = 0.01,
               ransac_n: int = 3, num_iterations: int = 1000, d_threshold: float = 0.01, view: bool = False) -> [dict, dict]:
    """
    find plane given the point cloud
    :param pcd: input point cloud
    :param max_plane_idx: want to find maximun number of plane
    :param distance_threshold: Max distance a point can be from the plane model, and still be considered an inlier.
    :param ransac_n: Number of initial points to be considered inliers in each iteration.
    :param num_iterations: Number of iterations.
    :param d_threshold: Density parameter that is used to find neighbouring points.
    :param view: Add visualization
    :return: pointcloud in two planes: dict{o3d.geometry.PointCloud, o3d.geometry.PointCloud
             coefficient of two planes: dict{ np.array, np.array} )
    """
    segment_models = {}
    segments = {}
    rest = copy.copy(pcd)
    for i in range(max_plane_idx):
        segment_models[i], inliers_temp = rest.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
        segments[i] = rest.select_down_sample(inliers_temp)
        # ----------refine----------------------------------------
        labels_seg = cluster(pcd = segments[i], eps = d_threshold * 10)
        candidates = [len(np.where(labels_seg == j)[0]) for j in np.unique(labels_seg)]
        best_candidate = int(np.unique(labels_seg)[np.where(candidates == np.max(candidates))[0]])
        #print("the best candidate is: ", best_candidate)
        rest = rest.select_down_sample(inliers_temp, invert=True) + segments[i].select_down_sample(list(np.where(labels_seg != best_candidate)[0]))
        segments[i] = segments[i].select_down_sample(list(np.where(labels_seg == best_candidate)[0]))
        # -----------------------------------------------------------
        # rest = rest.select_down_sample(inliers_temp, invert=True)
        #print("pass", i, '/', max_plane_idx, "done.")
    if view:
        for i in range(max_plane_idx):
            colors_seg = plt.get_cmap('tab20')(i)
            segments[i].paint_uniform_color(list(colors_seg[:3]))
        o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)] + [rest])
    return segments, segment_models


def get_plane_distance(select_bbox: o3d.geometry.OrientedBoundingBox, plane: np.array) -> float:
    '''
    # https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
    Function to find distance from the center of bbox to the plane
    :param select_bbox: input bbox
    :param plane: coefficient of a plane
    :return: float
    '''
    point = select_bbox.get_center()
    d = abs((plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]))
    e = (math.sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]))
    distance = d / e
    return distance


def get_width_length(selected_bbox: o3d.geometry.OrientedBoundingBox) -> [float, float]:
    """
    find the width and length of the bbox
    :param selected_bbox: input bbox
    :return: float, float
    """
    aa_obb = copy.copy(selected_bbox)
    original_rotation = aa_obb.R
    aa_obb = aa_obb.rotate(np.linalg.inv(original_rotation))

    max_bound = aa_obb.get_max_bound()
    min_bound = aa_obb.get_min_bound()
    size = max_bound - min_bound
    width = size[0]
    length = size[1]
    return width, length


def get_width_length_height(pcd, max_plane_idx=2, view=False) -> [np.array, np.array]:
    """
    given the pointcloud, find the box and return the geometry of the box and the top center of the box plane
    :param pcd: input pointcloud
    :param max_plane_idx: maximun number of plane want to find
    :param view: Add visualization
    :return: [width, length, height]: np.array, center: np.array
    """
    downpcd = downsample(pcd=pcd)
    labels = cluster(pcd=downpcd)
    wanted_pcd = get_largest_cluster(pcd=downpcd, labels=labels, view=view)
    segments, segment_models = find_plane(pcd=wanted_pcd, max_plane_idx=max_plane_idx, view=view)
    ## select the table index and box index
    point_length = np.array([len(segments[i].points) for i in range(max_plane_idx)])
    table_index = np.argsort(-point_length)[0]  # largest indices
    table_plane = segment_models[table_index]
    box_index = np.argsort(-point_length)[1]  # second largest indices
    box_points = segments[box_index]
    ## obtain geometry information from box points
    box_bbox = box_points.get_oriented_bounding_box()
    center = box_points.get_center()
    width, length = get_width_length(selected_bbox=box_bbox)
    height = get_plane_distance(select_bbox=box_bbox, plane=table_plane)
    if view:
        test_pcd = o3d.geometry.PointCloud()
        test_pcd.points = o3d.utility.Vector3dVector(np.asarray(box_bbox.get_box_points()))
        colorpoint = np.zeros_like(np.asarray(box_bbox.get_box_points()))
        colorpoint[:] = np.array([1, 0, 0])
        test_pcd.colors = o3d.utility.Vector3dVector(colorpoint)

        test2_pcd = o3d.geometry.PointCloud()
        test2_pcd.points = o3d.utility.Vector3dVector(box_bbox.get_center().reshape(1, -1))
        color2point = np.zeros_like(box_bbox.get_center().reshape(1, -1))
        color2point[:] = np.array([0, 1, 0])
        test2_pcd.colors = o3d.utility.Vector3dVector(color2point)
        o3d.visualization.draw_geometries([box_points] + [test_pcd] + [test2_pcd])
    return np.array([width, length, height]), center


### below refer to https://github.com/kaku756/icp_calib/blob/b6f9af98019990f6111836dcbad4e97e5b69f692/scripts/lib_cloud_conversion_between_Open3D_and_ROS.py


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud: o3d.geometry.OrientedBoundingBox, frame_id="odom") -> PointCloud2:
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud: PointCloud2) -> o3d.geometry.PointCloud:
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud