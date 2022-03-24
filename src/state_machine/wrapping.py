#!/usr/bin/env python

import copy

from sensor_msgs.msg import CameraInfo, Image
# from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import moveit_commander
import geometry_msgs.msg
# import message_filters
# import cv2
import rospy
import tf2_ros
# import tf2_geometry_msgs
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped, Vector3, Quaternion
# from sensor_msgs.msg import JointState
import baxter_interface
from baxter_interface import CHECK_VERSION
import time
# from copy import deepcopy
import copy as copy_module
# import math
from tf.transformations import quaternion_matrix

# import os, sys
# from datetime import datetime
import pickle
# import struct
# import pypcd
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import PointCloud
from datetime import datetime

from baxter_core_msgs.msg import EndpointState
from moveit_msgs.msg import Constraints, OrientationConstraint
from apriltag_ros.msg import AprilTagDetectionArray

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

from shape_msgs.msg import SolidPrimitive
from math import sqrt, sin, cos

# import Pyro4
# import Pyro4.util
# import zmq

from utils import get_pose_stamped_str, copy_pose_stamped, plan_cartesian_group, plan_group



class BaxterWrapping:
    def __init__(self):


        # tf
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # self.world_camera_transform = self.tfBuffer.lookup_transform("world", #source frame
        #                               "camera_rgb_optical_frame",
        #                               rospy.Time(0), #get the tf at first available time
        #                               rospy.Duration(10.0)) #wait for 1 second
        # self.camera_world_transform = self.tfBuffer.lookup_transform("camera_rgb_optical_frame", #source frame
        #                               "world",
        #                               rospy.Time(0), #get the tf at first available time
        #                               rospy.Duration(10.0)) #wait for 1 second

        # Baxter gripper interface
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        # init_state = rs.state().enabled

        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self.head = baxter_interface.Head()
        self.left_gripper.calibrate()
        self.right_gripper.calibrate()

        # moveit planning scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.sleep(2)

        self.init_obstacles()


        # We define two groups: One for each arm of Baxter
        # depending on the position of the grasping point we should have some high level decision making which arm to use

        # moveit interface left arm
        self.left_group = moveit_commander.MoveGroupCommander('left_arm')
        self.left_group.set_planner_id("RRTConnectkConfigDefault")
        self.left_group.set_planning_time(10)
        self.left_group.set_num_planning_attempts(5)
        self.left_group.set_start_state_to_current_state()
        self.left_group.set_goal_orientation_tolerance(0.2)

        # moveit interface right arm
        self.right_group = moveit_commander.MoveGroupCommander('right_arm')
        self.right_group.set_planner_id("RRTConnectkConfigDefault")
        self.right_group.set_planning_time(2)
        self.right_group.set_num_planning_attempts(5)
        self.right_group.set_start_state_to_current_state()
        self.right_group.set_goal_orientation_tolerance(0.2)

        ## Orientation constraints
        # c = Constraints()
        # EE_orientation_constraint = OrientationConstraint()
        # EE_orientation_constraint.header.frame_id = "world"
        # EE_orientation_constraint.link_name = "left_gripper"
        # EE_orientation_constraint.orientation.x = 0.
        # EE_orientation_constraint.orientation.y = 1.
        # EE_orientation_constraint.orientation.z = 0.
        # EE_orientation_constraint.orientation.w = 0.0
        # EE_orientation_constraint.absolute_x_axis_tolerance = 0.2
        # EE_orientation_constraint.absolute_y_axis_tolerance = 0.2
        # EE_orientation_constraint.absolute_z_axis_tolerance = 3
        # EE_orientation_constraint.weight = 1
        # c.orientation_constraints = [EE_orientation_constraint]
        #
        # self.left_group.set_path_constraints(c)
        # cr = copy.deepcopy(c)
        # cr.orientation_constraints[0].link_name = "right_gripper"
        # self.right_group.set_path_constraints(cr)
        ########### variables

        # Optoforce sensor variables
        self.opto_calibrated_right = False            # This variable is needed to remove the initial offset of the sensor
        self.opto_offsets_right = np.zeros(3)         # Offset for y,z axis
        self.opto_thresholds_right = np.asarray([5, 5])
        self.opto_forces_right = np.zeros(3)


        self.opto_calibrated_left = False  # This variable is needed to remove the initial offset of the sensor
        self.opto_offsets_left = np.zeros(3)  # Offset for y,z axis
        self.opto_thresholds_left = np.asarray([5, 5])
        self.opto_forces_left = np.zeros(3)


        # Boolean for force sensor
        self.interrupt_movement_right = False
        self.interrupt_movement_left = False

        self.arm = 'right'

        # Grasping points
        self.grasping_points = [np.zeros(3) for i in range(4)]
        self.detected_corner = [False for i in range(4)]
        self.active_corner = -1000



        ########## Predefined poses for testing
        self.test_start_pose = PoseStamped()
        self.test_start_pose.header.frame_id = "world"
        self.test_start_pose.pose.position.x = 0.628
        self.test_start_pose.pose.position.y = -0.621
        self.test_start_pose.pose.position.z = -0.041  # self.FOLDING_Z #fixed Z!!!!!!!!!!!!!!!!

        self.test_start_pose.pose.orientation.x = 0
        self.test_start_pose.pose.orientation.y = 1
        self.test_start_pose.pose.orientation.z = 0
        self.test_start_pose.pose.orientation.w = 0

        self.test_end_pose = copy_module.deepcopy(self.test_start_pose)
        self.test_end_pose.pose.position.y = self.test_end_pose.pose.position.y + 0.2

        print("Created test poses --------------------------")
        print("Start:" + get_pose_stamped_str(self.test_start_pose))
        print("End:" + get_pose_stamped_str(self.test_end_pose))
        print("-------------------------")
        #########

        # tf
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.world_camera_transform = self.tfBuffer.lookup_transform("world",  # source frame
                                                                     "camera_link",
                                                                     rospy.Time(0),
                                                                     # get the tf at first available time
                                                                     rospy.Duration(10.0))  # wait for 1 second
        self.camera_world_transform = self.tfBuffer.lookup_transform("camera_link",  # source frame
                                                                     "world",
                                                                     rospy.Time(0),
                                                                     # get the tf at first available time
                                                                     rospy.Duration(10.0))  # wait for 1 second

        self.right_gripper_world_transform = self.tfBuffer.lookup_transform("right_gripper",  # source frame
                                                                     "world",
                                                                     rospy.Time(0),
                                                                     # get the tf at first available time
                                                                     rospy.Duration(10.0))  # wait for 1 second

        ##### SUBSCRIPTIONS
        self.left_ee = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.left_ee_callback)
        self.right_ee = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.right_ee_callback)
        self.opto_right = rospy.Subscriber('/optoforce_fingertip_wrench', WrenchStamped, self.force_callback_right)
        self.opto_left = rospy.Subscriber('/optoforce_fingertip_wrench2', WrenchStamped, self.force_callback_left)
        self.tags_detected = rospy.Subscriber("/tag_detections",AprilTagDetectionArray, self.update_grasping_points)

        # Services
        self.ik_srv = rospy.ServiceProxy('/compute_ik',
                                         GetPositionIK)
        rospy.loginfo("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")

        self.patch_size_pc = 10

    def init_obstacles(self):
        # Box estimation should go here, maybe we can start with using this hardcoded stuff (THIS COULD BE USED TO DEFINE OBSTACLES)
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = self.robot.get_planning_frame()
        # box_pose.pose.position.x = 0.6
        # box_pose.pose.position.y = -0.35
        # box_pose.pose.position.z = 0
        # box_pose.pose.orientation.w = 1.0
        # box_name = "box"
        # self.scene.add_box(box_name, box_pose, size=(0.5, 0.05, 0.3))

        # Add Table Box
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.position.x = 0.8
        table_pose.pose.position.y = 0.
        table_pose.pose.position.z = -0.3
        table_pose.pose.orientation.w = 1.0
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(1, 1.22, 0.1))

        # Add bounding box around table
        left_wall = geometry_msgs.msg.PoseStamped()
        left_wall.header.frame_id = self.robot.get_planning_frame()
        left_wall.pose.position.x = 0.8
        left_wall.pose.position.y = 0.7
        left_wall.pose.position.z = 0.
        left_wall.pose.orientation.w = 1.0
        self.scene.add_box("left_wall", left_wall, size=(1.5, 0.05, 2.))

        right_wall = geometry_msgs.msg.PoseStamped()
        right_wall.header.frame_id = self.robot.get_planning_frame()
        right_wall.pose.position.x = 0.8
        right_wall.pose.position.y = -0.7
        right_wall.pose.position.z = 0.
        right_wall.pose.orientation.w = 1.0
        self.scene.add_box("right_wall", right_wall, size=(1.5, 0.05, 2.))

        front_wall = geometry_msgs.msg.PoseStamped()
        front_wall.header.frame_id = self.robot.get_planning_frame()
        front_wall.pose.position.x = 1.3
        front_wall.pose.position.y = 0.
        front_wall.pose.position.z = 0.
        front_wall.pose.orientation.w = 1.0
        self.scene.add_box("front_wall", front_wall, size=(0.05, 1.5, 2.))

    #### CALLBACKS

    def left_ee_callback(self, msg):
        return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 0])

    def right_ee_callback(self, msg):
        return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1])

    def force_callback_right(self, msg):
        """
            This function is a callback of the fingertip sensor. Hence it sets the interrupt boolean
        Args:
            msg (WrenchStamped): wrench perceived from the optoforce sensors
        """
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z

        if not self.opto_calibrated_right:
            self.opto_offsets_right[0] = force_x
            self.opto_offsets_right[1] = force_y
            self.opto_offsets_right[2] = force_z
            self.opto_calibrated_right = True
            print("Calibration: offset_y = " + str(self.opto_offsets_right[1]) + ", offset_z =  " + str(self.opto_offsets_right[2]))

        if np.abs(force_y-self.opto_offsets_right[1]) > self.opto_thresholds_right[0] or np.abs(force_z-self.opto_offsets_right[2]) > self.opto_thresholds_right[1]:
            self.interrupt_movement_right = True
            # print("STOP movement")
        else:
            self.interrupt_movement_right = False

        self.opto_forces_right = np.array([force_x-self.opto_offsets_right[0], force_y-self.opto_offsets_right[1], force_z-self.opto_offsets_right[2]])


    def force_callback_left(self, msg):
        """
            This function is a callback of the fingertip sensor. Hence it sets the interrupt boolean
        Args:
            msg (WrenchStamped): wrench perceived from the optoforce sensors
        """
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z

        if not self.opto_calibrated_left:
            self.opto_offsets_left[0] = force_x
            self.opto_offsets_left[1] = force_y
            self.opto_offsets_left[2] = force_z
            self.opto_calibrated_left = True
            print("Calibration: offset_y = " + str(self.opto_offsets_left[1]) + ", offset_z =  " + str(self.opto_offsets_left[2]))

        if np.abs(force_y-self.opto_offsets_left[1]) > self.opto_thresholds_left[0] or np.abs(force_z-self.opto_offsets_left[2]) > self.opto_thresholds_left[1]:
            self.interrupt_movement_left = True
            # print("STOP movement")
        else:
            self.interrupt_movement_left = False

        self.opto_forces_left = np.array([force_x-self.opto_offsets_left[0], force_y-self.opto_offsets_left[1], force_z-self.opto_offsets_left[2]])

    # Service Calls
    def get_ik(self, pose_stamped,
               ik_timeout=1.0,
               ik_attempts=1,
               avoid_collisions=False):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """

        cur_state = self.robot.get_current_state()

        req = GetPositionIKRequest()
        req.ik_request.group_name = self.arm + "_arm"
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp

    def update_grasping_points(self, msg):
        tag_ids = [5, 9, 4, 3]
        threshold = -0.16
        self.detected_corner = [False for i in range(4)]
        for tag in msg.detections:
            if tag.id[0] in tag_ids:
                self.grasping_points[tag_ids.index(tag.id[0])] = self.transform_to_world(np.array([tag.pose.pose.pose.position.x,
                                                         tag.pose.pose.pose.position.y,
                                                         tag.pose.pose.pose.position.z]))
                if self.grasping_points[tag_ids.index(tag.id[0])][2] > threshold:
                    self.detected_corner[tag_ids.index(tag.id[0])] = True

    def propose_corner(self):
        proposed = False
        while not proposed:
            if (True in self.detected_corner):
                for i, corner in enumerate(self.detected_corner):
                    if corner:
                        self.active_corner = i
                        break
                print("Corner: ", self.active_corner)
                proposed = True
            else:
                print("Corner not proposed yet")

    def calculate_release_orientation(self, movedir):
        """
            calculate the global pose of the endeffector to release the cloth
        Args:
            movedir: np.array([x,y,z]) place direction in global frame
        """
        link_name = self.arm + "_gripper"
        world_gripper_trafo = self.tfBuffer.lookup_transform("world",  # source frame
                                       link_name,
                                       rospy.Time(0),
                                       # get the tf at first available time
                                       rospy.Duration(1.0))
        quat = world_gripper_trafo.transform.rotation
        print("Quaternion World to gripper")
        print([quat.x, quat.y, quat.z, quat.w])

        R1 = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        R1 = R1[0:3, 0:3]

        # Gripper z axis in world frame
        z_G_W = R1.dot(np.array([0, 0, 1]))
        z_G_W /= np.linalg.norm(z_G_W)

        print("Gripper Z in world frame:")
        print(z_G_W)


        # desired rotation axis
        rot_axis = np.cross(z_G_W, movedir)
        rot_axis /= np.linalg.norm(rot_axis)
        print("rot_axis: ")
        print(rot_axis)
        # angle by which we'd like to rotate
        alpha = 20 * np.pi / 180

        quat = [rot_axis[0] * sin(alpha / 2), rot_axis[1] * sin(alpha / 2), rot_axis[2] * sin(alpha / 2), cos(alpha / 2)]
        R2 = quaternion_matrix(quat)[0:3, 0:3]

        R12 = np.matmul(R1.T, R2.T)
        quat12 = self.rotation_matrix_quaternion(R12)

        des_orient = Quaternion()
        des_orient.x = quat12[0]
        des_orient.y = quat12[1]
        des_orient.z = quat12[2]
        des_orient.w = quat12[3]

        return des_orient

    def rotation_matrix_quaternion(self, R):
        w = sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        x = (R[2, 1] - R[1, 2]) / (4.0 * w)
        y = (R[0, 2] - R[2, 0]) / (4.0 * w)
        z = (R[1, 0] - R[0, 1]) / (4.0 * w)
        return np.array([x, y, z, w])

    #
    # def check_marker_visibility(self, id):
    #     return self.detected_corner[id]

    def transform_to_world(self, pos):
        quat = self.world_camera_transform.transform.rotation
        trans = self.world_camera_transform.transform.translation
        rotm = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        rotm = rotm[0:3, 0:3]

        return np.array([trans.x, trans.y, trans.z]) + rotm.dot(pos)
    #### ACTIONS

    def calibrate_sensor(self):
        """ Functions to ask the sensor to be calibrated

        """
        self.opto_calibrated_right = False
        self.opto_calibrated_left = False

    def close_right_gripper(self):
        self.right_gripper.close()

    def open_right_gripper(self):
        self.right_gripper.open()

    def close_left_gripper(self):
        self.left_gripper.close()

    def open_left_gripper(self):
        self.left_gripper.open()

    def close_gripper(self):
        """ Close the gripper

        """
        key = self.arm
        if key == "right":
            self.close_right_gripper()
        if key == "left":
            self.close_left_gripper()

    def open_gripper(self):
        """ Open the gripper

        """
        key = self.arm
        if key == "right":
            self.open_right_gripper()
        if key == "left":
            self.open_left_gripper()

    def cancel_action(self):
        prev_len = len(self.list_actions)
        item = self.list_actions.pop(-1)

        print("Removing last item from action list: prev_len = " + str(prev_len) + ", new_len = " + str(
            len(self.list_actions)))
        print("Item with img1 = " + item[0] + " and img2 = " + item[1])

        pkl_file = open(self.path_imgs + "/" + self.name_pkl_action, 'wb')
        pickle.dump(self.list_actions, pkl_file)
        pkl_file.close()

    def initialize(self):
        if self.arm == 'right':
            self.rabp_go_base_pose()
        if self.arm == 'left':
            self.labp_go_base_pose()

    def grasp(self, id):
        """ Function that moves the correct arm to the grasping position

        Args:
            id (int): id of corner we want to grasp

        Returns:
            success (bool): if action was successful

        """
        # select arm
        pos = PoseStamped()
        pos.pose.position.x = self.grasping_points[id][0]
        pos.pose.position.y = self.grasping_points[id][1]
        pos.pose.position.z = self.grasping_points[id][2] + 0.04
        pos.pose.orientation.x = 0.
        pos.pose.orientation.y = 1.
        pos.pose.orientation.z = 0.
        pos.pose.orientation.w = 0.0
        print(pos)

        self.arm_selection(pos.pose.position.y)
        print(self.arm)

        # plan and execute trajectory
        self.move_to_point(pos)

        success = True
        return success

    def pull(self):
        """ Function that moves the robot arm to pull the cloth
            up to a specific force threshold

        Returns:
            success (bool): if action was successful

        """
        # Move up
        self.calibrate_sensor()
        time.sleep(1)

        # Define movement
        direction = np.array([0., 0., 1.])
        distance = 0.01

        count = 0
        print(count)
        print(self.opto_forces_right)
        print(self.interrupt_movement_right)
        # Keep moving while force threshold not surpassed
        pull = True
        while pull:           # TODO: define better safe stopping condition
            self.move_along_line(direction, distance)
            print(self.opto_forces_right)
            print(self.interrupt_movement_right)
            count += 1
            print(count)
            pull = not (self.interrupt_movement_right or self.interrupt_movement_left) and count < 10

        # TODO: code success
        success = True
        return success

    def place(self, A, B, l_edge):
        """

        Args:
            A (Vector3):
            B (Vector3):
            l_edge (float):

        Returns:
            success (bool): if action was successful

        """
        group = self.get_group()

        # Get EE position
        EE_pose = group.get_current_pose()
        # plan movements
        directions, distances = self.get_waypoints(A, B, EE_pose, l_edge)

        # move towards the closest corner
        self.move_along_line(directions[0], distances[0])

        # move towards the final point
        self.move_along_line(directions[1], distances[1])

        # Get EE position
        EE_pose = group.get_current_pose()
        EE_pose.pose.orientation = self.calculate_release_orientation(directions[0])
        group.set_pose_target(EE_pose.pose)
        group.go(wait=True)
        group.clear_pose_targets()
        time.sleep(1)

        group.stop()
        time.sleep(1)

        success = True
        return success

    def get_waypoints(self, A, B, EE_pose, l_edge):
        """ Functions to compute two waypoints needed to complete the folding

                Args:
                    A (Vector3): position corner A
                    B (Vector3): position corner B
                    e (PoseStamped): pose of EE
                    l_edge (float): length of the edge

                Returns:
                    directions (list[numpy(float)]): list of directions for both waypoints
                    distances (list[float]): list of distances for both waypoints

                """
        directions = []
        distances = []

        ### Compute final point
        a = np.array([A.x, A.y, A.z])
        b = np.array([B.x, B.y, B.z])
        e = np.array([EE_pose.pose.position.x, EE_pose.pose.position.y, EE_pose.pose.position.z])

        # Compute first movement EE toward closest edge (assumed to be A)
        a1 = copy.deepcopy(a)
        a1[2] += 0.08               # Offset on z
        a1_e = a1 - e
        # directions.append(a1_e/np.linalg.norm(a1_e))
        # distances.append(np.linalg.norm(a1_e))

        # compute total translation
        f = copy.deepcopy(a)
        f[2] -= l_edge
        l = np.linalg.norm(e - f) - l_edge
        a_b = b - a
        a_b = a_b / np.linalg.norm(a_b) * l

        # Sum of movements
        e_b = a1_e + a_b

        # Move towards b and go down
        directions.append(e_b / np.linalg.norm(e_b))
        distances.append(np.linalg.norm(e_b))

        directions.append(np.array([0., 0., -1.]))
        distances.append(0.03)



        print("Directions: ", directions)
        print("Distances: ", distances)
        return directions, distances


    def arm_selection(self, grasp_y_position):    # TODO: to be tested
        """ Functions that decide which arm to use for the ongoing execution
            and set the class variable self.arm

        Args:
            grasp_y_position (float): y coordinates grasping point

        """
        if grasp_y_position > 0.01:  # TODO: check this is an ok value
            key = 'left'
        else:
            key = 'right'

        # move head depending on the arm that will move
        print("Selected arm: ", key)
        # if key == "left":
        #     self.head.set_pan(0.5)
        #
        # if key == "right":
        #     self.head.set_pan(-0.5)

        self.arm = key

    def get_group(self):
        if self.arm == 'right':
            return self.right_group
        elif self.arm == 'left':
            return self.left_group

    def test_traj(self):
        print("Test trajectory")
        # calculate waypoint
        # waypoint_list=[]
        # waypoint_list.append(self.test_start_pose)
        # waypoint_list.append(self.test_end_pose)
        # self.folding_sequence(self.right_group,"right",waypoint_list)

        waypoint_list = []
        waypoint_list.append(self.test_start_pose.pose)
        waypoint_list.append(self.test_end_pose.pose)
        group = self.right_group

        # COmpute cartesian path also takes input argument bool collisionavoidance, can be used later
        (plan, fraction) = self.right_group.compute_cartesian_path(
            waypoint_list, 0.001, 0.0, avoid_collisions=True)
        group.execute(plan, wait=True)

        group.stop()
        time.sleep(1)

        # self.wrap_start_pose = self.test_start_pose
        # self.wrap_end_pose = self.test_end_pose

        # self.execute_wrapping()



    def move_along_line(self, direction, dist=0.01):
        """ Input: gnp.array, arm_key = "left" or "right"
        This input corresponds to the constant direction in which the robot should move in cartesian space
        This function will be used to stretch the cloth"""

        # For now hardcode direction
        # direction = Vector3()
        # direction.x = 0.
        # direction.y = 0.
        # direction.z = 1.

        group = self.get_group()

        current_pose = group.get_current_pose()
        waypoints = []

        n_waypoints = 2

        normalizer = sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
        move_direction = direction / normalizer


        for i in range(n_waypoints):
            new_pose = copy.deepcopy(current_pose.pose)
            new_pose.position.x += move_direction[0] * dist * (i + 1) / n_waypoints
            new_pose.position.y += move_direction[1] * dist * (i + 1) / n_waypoints
            new_pose.position.z += move_direction[2] * dist * (i + 1) / n_waypoints
            waypoints.append(new_pose)

        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.001, 4.0, avoid_collisions=True)

        # OPTIONAL TODO: Check timing of trajectory
        # n_timed_traj = len(plan.joint_trajectory.points)
        #
        # desired_duration = 10.0
        # dt = int((desired_duration / n_timed_traj) * 1e9)  # nsecs
        #
        # plan.joint_trajectory.points[0].time_from_start.secs = 0
        # plan.joint_trajectory.points[0].time_from_start.nsecs = 0
        # print(str(n_timed_traj) + "timed trajectory points!")
        # print(str(dt * 1e-9) + " s per point")
        # for i in range(1, n_timed_traj):
        #     plan.joint_trajectory.points[i].time_from_start.nsecs = plan.joint_trajectory.points[i-1].time_from_start.nsecs + dt
        #
        #     if plan.joint_trajectory.points[i].time_from_start.nsecs >= int(1e9):
        #         plan.joint_trajectory.points[i].time_from_start.secs += 1
        #         plan.joint_trajectory.points[i].time_from_start.nsecs -= 1e9
        # print(plan)
        group.execute(plan, wait=True)


    def move_to_point(self, point):
        """ Input: geometry_msgs/PoseStamped
                This input corresponds to the constant direction in which the robot should move in cartesian space
                This function will be used to stretch the cloth"""

        # Select which arm to use
        self.arm_selection(point.pose.position.y)
        group = self.get_group()


        # Done with right group, must be choosen a new one
        init_pose = group.get_current_pose()
        # point.pose.orientation = init_pose.pose.orientation
        print(init_pose.pose.orientation)
        group.set_pose_target(point.pose)
        group.go(wait=True)
        group.clear_pose_targets()
        time.sleep(1)

        group.stop()
        time.sleep(1)

    def ik_control(self, cartesian_point):
        """
        This function does IK control, hence it controls the joint angles to reach a cartesian point
        Args:
            cartesian_point (PoseStamped()):
        """
        ik_response = self.get_ik(cartesian_point)
        k = self.arm
        group = self.get_group()

        joint_names = [k + "_e0", k + "_e1", k + "_s0", k + "_s1", k + "_w0", k + "_w1", k + "_w2"]
        for name in joint_names:
            group.set_joint_value_target(name, ik_response.solution.joint_state.position[ik_response.solution.joint_state.name.index(name)])
        group.go(wait=True)

    def rabp_go_base_pose(self):
        print("-- right arm base pose --")
        self.right_group.set_joint_value_target("right_e0", -0.2661456666981193);
        self.right_group.set_joint_value_target("right_e1", 1.5274613695369008);
        self.right_group.set_joint_value_target("right_s0", 0.6807039746241523);
        self.right_group.set_joint_value_target("right_s1", -0.9345777950191884);
        self.right_group.set_joint_value_target("right_w0", 0.1902136176977913);
        self.right_group.set_joint_value_target("right_w1", 0.9901845985800346);
        self.right_group.set_joint_value_target("right_w2", -0.5648884251388037);
        plan = self.right_group.go(wait=True)


    def labp_go_base_pose(self):
        print("-- left arm base pose --")
        self.left_group.set_joint_value_target("left_e0", 0.2941408160770281);
        self.left_group.set_joint_value_target("left_e1", 1.584218658688661);
        self.left_group.set_joint_value_target("left_s0", -0.5982525072753113);
        self.left_group.set_joint_value_target("left_s1", -0.9368787662010164);
        self.left_group.set_joint_value_target("left_w0", -0.22281070944035636);
        self.left_group.set_joint_value_target("left_w1", 0.9541360500647273);
        self.left_group.set_joint_value_target("left_w2", 0.4881893857445329);
        plan = self.left_group.go(wait=True)

    def get_num_actions(self):
        print("Number of action items: " + str(len(self.list_actions)))
        print("Number of no-action items: " + str(len(self.list_no_actions)))


def print_commands():
    print("---------------------------------------")
    print("com - display commands")
    print("rabp - right arm base pose")
    print("labp - left arm base pose")
    print("crg - close right gripper")
    print("clg - close left gripper")
    print("org - open right gripper")
    print("olg - open left gripper")
    print("t - testing trajectory")
    print("p - to point trajectory")
    print("e - execute wrapping")
    print("ca - cancel action")
    print("grasp - grasp action")
    print("pull - pull action")
    print("place - place action")
    print("movedir - move along line")
    # print("exe - EXPLORE!!!!")
    print("---------------------------------------")


def main():
    # remap again
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)

    rospy.init_node('baxter_wrapping', anonymous=False)

    # init class obj
    bwrap = BaxterWrapping()

    list_explore_actions = []

    try:

        print_commands()

        while not (rospy.is_shutdown()):
            # rospy.spin()

            r_str = raw_input()
            if (r_str == '\x03'):
                break

            if (r_str == 'rabp'):
                bwrap.rabp_go_base_pose()
                # p = PoseStamped()
                # p.pose.position.x = 0.65
                # p.pose.position.y = -0.5
                # p.pose.position.z = 0.03
                # p.pose.orientation.x = 0.
                # p.pose.orientation.y = 1.
                # p.pose.orientation.z = 0.
                # p.pose.orientation.w = 0.0
                # bwrap.move_to_point(p)

            elif (r_str == 'labp'):
                bwrap.labp_go_base_pose()
                # p = PoseStamped()
                # p.pose.position.x = 0.65
                # p.pose.position.y = 0.5
                # p.pose.position.z = 0.03
                # p.pose.orientation.x = 0.
                # p.pose.orientation.y = 1.
                # p.pose.orientation.z = 0.
                # p.pose.orientation.w = 0.0
                # bwrap.move_to_point(p)

            elif (r_str == 'crg'):
                bwrap.close_right_gripper()

            elif (r_str == 'org'):
                bwrap.open_right_gripper()

            elif (r_str == 'clg'):
                bwrap.close_left_gripper()

            elif (r_str == 'olg'):
                bwrap.open_left_gripper()

            elif (r_str == 'com'):
                print_commands()

            elif (r_str == 'e'):
                bwrap.start_execute_wrapping()

            elif (r_str == "ca"):
                bwrap.cancel_action()

            elif (r_str == 't'):
                bwrap.test_traj()
                # bwrap.point_cloud_test()
                # print(dir(bwrap.head))
                # bwrap.head.set_pan(0.0)

            elif (r_str == 'p'):
                bwrap.move_to_point(0)

            elif (r_str == "grasp1"):
                bwrap.grasp(0)

            elif (r_str == "grasp2"):
                bwrap.grasp(1)

            elif (r_str == "grasp3"):
                bwrap.grasp(2)

            elif (r_str == "grasp4"):
                bwrap.grasp(3)

            elif (r_str == "pull"):
                bwrap.pull()

            elif (r_str == "place1"):
                pA = Vector3()
                pA.x = 0.825
                pA.y = -0.105
                pA.z = -0.12
                pD = Vector3()
                pD.x = 0.58
                pD.y = 0.
                pD.z = -0.14
                bwrap.place(pA, pD, 0.08)

            elif (r_str == "place2"):
                pA = Vector3()
                pA.x = 0.73
                pA.y = 0.11
                pA.z = -0.14
                pD = Vector3()
                pD.x = 0.72
                pD.y = -0.17
                pD.z = -0.14
                bwrap.place(pA, pD, 0.08)

            elif (r_str == "place3"):
                pA = Vector3()
                pA.x = 0.58
                pA.y = 0.
                pA.z = -0.14
                pD = Vector3()
                pD.x = 0.825
                pD.y = -0.05
                pD.z = -0.14
                bwrap.place(pA, pD, 0.08)

            elif (r_str == "place4"):
                pA = Vector3()
                pA.x = 0.72
                pA.y = -0.17
                pA.z = -0.14
                pD = Vector3()
                pD.x = 0.73
                pD.y = 0.11
                pD.z = -0.14
                bwrap.place(pA, pD, 0.08)

            elif (r_str == "movedir"):
                direction = np.array([0.,0.,1])
                bwrap.move_along_line(direction, 0.03)

            elif (r_str == "movepr"):
                p = PoseStamped()

                p.pose.position.x = 0.65
                p.pose.position.y = -0.5
                p.pose.position.z = 0.03
                # p.pose.orientation.x = 0.
                # p.pose.orientation.y = 1.
                # p.pose.orientation.z = 0.
                # p.pose.orientation.w = 0.0
                bwrap.move_to_point(p)
            elif (r_str == "movepl"):
                p = PoseStamped()
                p.pose.position.x = 0.75
                p.pose.position.y = 0.2
                p.pose.position.z = 0.
                p.pose.orientation.x = 0.
                p.pose.orientation.y = 1.
                p.pose.orientation.z = 0.
                p.pose.orientation.w = 0.0
                bwrap.move_to_point(p)
            elif (r_str == "call_ik"):
                start_time = rospy.get_time()
                p = PoseStamped()
                p.pose.position.x = 0.75
                p.pose.position.y = -0.2
                p.pose.position.z = 0.
                p.pose.orientation.x = 0.
                p.pose.orientation.y = 1.
                p.pose.orientation.z = 0.
                p.pose.orientation.w = 0.0
                current_time = rospy.get_time()
                while (current_time - start_time) < 30.:
                    t = (current_time - start_time)
                    p_ = copy.deepcopy(p)
                    p_.pose.position.y += 0.1 * sin(2 * np.pi * t / 10.)
                    bwrap.ik_control(p_)
                    # rospy.sleep(0.01)
                    current_time = rospy.get_time()

                print("Called IK Servcie!!!")

            elif (r_str == "cur_orient"):
                group = bwrap.get_group()

                # Get EE position
                EE_pose = group.get_current_pose()
                o = bwrap.calculate_release_orientation(np.array([0, 1, 0]))
                EE_pose.pose.orientation = o
                print(o)
                group.set_pose_target(EE_pose.pose)
                group.go(wait=True)
                group.clear_pose_targets()
                time.sleep(1)

                group.stop()
                time.sleep(1)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
