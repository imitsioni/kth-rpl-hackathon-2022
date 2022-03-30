#!/usr/bin/env python

from fcntl import F_SEAL_SEAL
from xmlrpc.client import boolean
import rospy
import smach
import smach_ros
import time
import numpy as np
import os

## Planning imports
from wrapping import BaxterWrapping
import moveit_commander
import time
from geometry_msgs.msg import Vector3, Point, PointStamped
from moveit_msgs.msg import OrientedBoundingBox
import rospy
from language_server import LanguageServer

## Planning imports


# Defines start node to get all info
class Start(smach.State):

    def __init__(self,
                 outcomes=['sucess'],
                 input_keys=['foo_counter_in'],
                 output_keys=['foo_counter_out']) -> None:
        """Initialization node, just defaults to a sub state folding machine.

        Args:
            outcomes (list, optional): possible outcomes. 
            Defaults to ['sucess'].
            input_keys (list, optional): input keys to interact. 
            Defaults to ['foo_counter_in'].
            output_keys (list, optional): output keys to be interacted by other
             nodes. Defaults to ['foo_counter_out'].
        """
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state Identify Corner')
        time.sleep(2)
        return 'sucess'


class IdentifyCorner(smach.State):

    def __init__(self,
                 baxter: object,
                 lang_server: object,
                 outcomes=['sucess'],
                 input_keys=['baxter'],
                 output_keys=['foo_counter_out']) -> None:
        """Node on the state machine to identify a corner

        Args:
            baxter (object): Baxter object created on wrapping.py
            lang_server (object): LanguageServer object from language_server.py
            outcomes (list, optional): possible outcomes. 
            Defaults to ['sucess'].
            input_keys (list, optional): input keys to interact. 
            Defaults to ['baxter'].
            output_keys (list, optional): output keys to be interacted by other
             nodes. 
            Defaults to ['foo_counter_out'].
        """

        self.baxter = baxter
        self.lang_server = lang_server
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=['baxter'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Identify Corner')

        #TODO: ask which corner to grasp

        # Initialize position
        self.baxter.initialize()

        time.sleep(2)
        return 'sucess'


# define state Bar
class GraspCorner(smach.State):

    def __init__(self, baxter: object, corner_id: int,
                 lang_server: object) -> None:
        """Stretch node, is the last node of the sub state machine sequence

        Args:
            baxter (object): Baxter object created on wrapping.py
            lang_server (object): LanguageServer object from language_server.py
            corner_id (int): integer for the corner_id from (0,1,2,3)
        """

        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=['baxter', 'corner_id'])
        self.corner_id = corner_id
        self.baxter = baxter
        self.lang_server = lang_server

    def execute(self, userdata):
        # Ask human to start grasping
        #self.lang_server.ask_for_start_grasping()
        self.lang_server.plese_propose_corner()
        self.baxter.propose_corner()
        self.corner_id = self.baxter.active_corner

        # Grasp corner
        rospy.loginfo('Executing state Grasp Corner')
        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

        self.baxter.grasp(self.corner_id)  # int

        # Human will tell "CLOSE"
        self.lang_server.ask_for_close_gripper()

        self.baxter.close_gripper()
        time.sleep(2)

        return 'sucess'


# define state Bar
class Stretch(smach.State):

    def __init__(self, baxter: object, lang_server: object) -> None:
        """Stretch node, is the last node of the sub state machine sequence

        Args:
            baxter (object): Baxter object created on wrapping.py
            lang_server (object): LanguageServer object from language_server.py
            final_flag (bool): Boolean for to signal if it is the final fold
        """

        smach.State.__init__(self, outcomes=['sucess'], input_keys=['baxter'])
        self.baxter = baxter
        self.lang_server = lang_server

    def execute(self, userdata):
        rospy.loginfo('Executing state Stretch')

        # Stretch
        self.baxter.pull()
        time.sleep(0.5)

        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

        return 'sucess'


class Fold(smach.State):

    def __init__(self, counter: int, baxter: object, edge: float,
                 final_corners: PointStamped, closest_corners: PointStamped,
                 corner_id: int, lang_server: object) -> None:
        """Creates the Fold node which is connected to a Goal node. Performs,
        the folding of the cloth.
        Args:
            counter (int): Counter of the corner loop of the state machine
            baxter (object): Baxter object created on wrapping.py
            edge (float): float describing the size of the edge of the box
            final_corners (PointStamped): ROS message to give coordinates about
            the final corner to fold
            closest_corners (PointStamped): ROS message giving the cordinates 
            of the closest corner
            corner_id (int): integer for the corner_id from (0,1,2,3)
            lang_server (object): LanguageServer object from language_server.py
        """

        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=[
                                 'corner_id', 'edge', 'baxter',
                                 'closest_corners', 'final_corners'
                             ])

        self.baxter = baxter
        self.corner_id = 0
        self.edge = edge
        self.closest_corners = closest_corners
        self.final_corners = final_corners
        self.lang_server = lang_server

    def execute(self, userdata):
        rospy.loginfo('Executing state Fold')
        time.sleep(0.5)
        # Place
        corner = self.baxter.active_corner
        edge = self.edge

        self.baxter.place(self.closest_corners[corner],
                          self.final_corners[corner],
                          edge)  # Vector3, Vector3, float

        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        return 'sucess'


class Goal(smach.State):

    def __init__(self, baxter: object, lang_server: object,
                 final_flag: bool) -> None:
        """Goal node, is the last node of the sub state machine sequence

        Args:
            baxter (object): Baxter object created on wrapping.py
            lang_server (object): LanguageServer object from language_server.py
            final_flag (bool): Boolean for to signal if it is the final fold
        """
        smach.State.__init__(self, outcomes=['sucess'], input_keys=['baxter'])
        self.baxter = baxter
        self.lang_server = lang_server
        self.final_flag = final_flag

    def execute(self, userdata):
        rospy.loginfo('Executing state GOAL')
        # Open gripper
        self.baxter.open_gripper()
        # Added upwards movement
        self.baxter.move_along_line(np.array([0., 0., 1]), 0.04)

        self.baxter.initialize()
        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

        if not self.final_flag:
            # Ask human whether to keep grasping
            self.lang_server.ask_for_corner()

        if self.final_flag:
            if self.lang_server.patrick:
                self.lang_server.thank_you_patrick()
            if self.lang_server.chris:
                self.lang_server.thank_you_chris()

            self.lang_server.thanking_for_participating()
            self.land_server.stop = True

        return 'sucess'


def sub_state_machine_fold_corner(counter: int, final_flag: bool,
                                  baxter: object, edge: float,
                                  final_corners: PointStamped,
                                  closest_corners: PointStamped,
                                  corner_id: int, lang_server: object) -> None:
    """Creates a sub state machine for each of the foldings, since it is a 
    repeated task depending on the number of corners.
    Args:
        counter (int): Counter of the corner loop of the state machine
        final_flag (bool): Boolean for to signal if it is the final fold
        baxter (object): Baxter object created on wrapping.py
        edge (float): float describing the size of the edge of the box
        final_corners (PointStamped): ROS message to give coordinates about
        the final corner to fold
        closest_corners (PointStamped): ROS message giving the cordinates of 
        the closest corner.
        corner_id (int): integer for the corner_id from (0,1,2,3)
        lang_server (object): LanguageServer object from language_server.py
    """

    smach.StateMachine.add(
        'IdentifyCorner_' + str(counter),
        IdentifyCorner(baxter=baxter, lang_server=lang_server),
        transitions={'sucess': 'GraspCorner_' + str(counter)})

    smach.StateMachine.add('GraspCorner_' + str(counter),
                           GraspCorner(baxter=baxter,
                                       corner_id=corner_id,
                                       lang_server=lang_server),
                           transitions={
                               'sucess': 'Stretch_' + str(counter),
                           })

    smach.StateMachine.add('Stretch_' + str(counter),
                           Stretch(baxter=baxter, lang_server=lang_server),
                           transitions={
                               'sucess': 'Fold_' + str(counter),
                           })

    smach.StateMachine.add('Fold_' + str(counter),
                           Fold(baxter,
                                corner_id,
                                edge,
                                closest_corners,
                                final_corners,
                                lang_server=lang_server),
                           transitions={'sucess': 'Goal_' + str(counter)})

    if not final_flag:
        smach.StateMachine.add(
            'Goal_' + str(counter),
            Goal(baxter, lang_server=lang_server),
            transitions={'sucess': 'IdentifyCorner_' + str(counter + 1)})
    else:
        smach.StateMachine.add('Goal_' + str(counter),
                               Goal(baxter,
                                    lang_server=lang_server,
                                    final_flag=final_flag),
                               transitions={'sucess': 'folded'})


def convert_to_vec(msg):
    return Vector3(msg.point.x, msg.point.y, msg.point.z)


def main():
    rospy.init_node('folding_state_machine')
    lang_server = LanguageServer(pid=os.getpid(), chris=False, patrick=False)

    # remap again
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    time.sleep(5)
    # init class obj
    baxter = BaxterWrapping()
    time.sleep(5)

    ############### INITIALIZATION ##############
    # Initialize right and left arm
    baxter.rabp_go_base_pose()
    baxter.labp_go_base_pose()
    baxter.open_left_gripper()
    baxter.open_right_gripper()

    corner0_msg = rospy.wait_for_message('/bbox_world/corner0', PointStamped)
    A = convert_to_vec(corner0_msg)

    corner1_msg = rospy.wait_for_message('/bbox_world/corner1', PointStamped)
    B = convert_to_vec(corner1_msg)

    corner2_msg = rospy.wait_for_message('/bbox_world/corner2', PointStamped)
    C = convert_to_vec(corner2_msg)

    corner3_msg = rospy.wait_for_message('/bbox_world/corner3', PointStamped)
    D = convert_to_vec(corner3_msg)

    closest_corners = [A, B, C, D]
    final_corners = [D, C, B, A]

    edge_msg = rospy.wait_for_message('/bbox/geometry', OrientedBoundingBox)
    # edge = 0.08
    edge = edge_msg.extents.z
    print(edge)
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['final_outcome'])
    sm_top.userdata.sm_counter = 0
    sm_top.userdata.corner_id = 0
    sm_top.userdata.baxter = baxter
    sm_top.userdata.final_corners = final_corners
    sm_top.userdata.closest_corners = closest_corners
    sm_top.userdata.edge = edge

    # sm.userdata.corner_positon

    # Open the container
    counter_fold = 0
    max_goal_counter = 3
    with sm_top:
        # Add states to the container

        smach.StateMachine.add('Start',
                               Start(),
                               transitions={'sucess': 'FinalGoal'})

        sm_sub = smach.StateMachine(outcomes=['folded'])
        sm_sub.userdata.corner_id = 0
        sm_sub.userdata.edge = edge

        with sm_sub:
            sm_sub.userdata.baxter = baxter
            sm_sub.userdata.final_corners = final_corners
            sm_sub.userdata.closest_corners = closest_corners
            sm_sub.userdata.edge = edge
            sm_sub.userdata.corner_id = 0
            sub_state_machine_fold_corner(counter=0,
                                          baxter=baxter,
                                          edge=edge,
                                          final_corners=final_corners,
                                          corner_id=0,
                                          closest_corners=closest_corners,
                                          lang_server=lang_server)
            sm_sub.userdata.corner_id = 1
            sub_state_machine_fold_corner(counter=1,
                                          baxter=baxter,
                                          edge=edge,
                                          final_corners=final_corners,
                                          corner_id=1,
                                          closest_corners=closest_corners,
                                          lang_server=lang_server)
            sm_sub.userdata.corner_id = 2
            sub_state_machine_fold_corner(counter=2,
                                          baxter=baxter,
                                          edge=edge,
                                          final_corners=final_corners,
                                          corner_id=2,
                                          closest_corners=closest_corners,
                                          lang_server=lang_server)
            sm_sub.userdata.corner_id = 3
            sub_state_machine_fold_corner(counter=3,
                                          final_flag=True,
                                          baxter=baxter,
                                          edge=edge,
                                          final_corners=final_corners,
                                          corner_id=3,
                                          closest_corners=closest_corners,
                                          lang_server=lang_server)

        smach.StateMachine.add('FinalGoal',
                               sm_sub,
                               transitions={'folded': 'final_outcome'})

    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    #sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()


if __name__ == '__main__':
    main()
