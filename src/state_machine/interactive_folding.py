#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import numpy as np

## Planning imports
from wrapping import BaxterWrapping
import moveit_commander
import time
from geometry_msgs.msg import Vector3
import rospy

## Planning imports



# Defines start node to get all info
class Start(smach.State):

    def __init__(self,
                 outcomes=['sucess'],
                 input_keys=['foo_counter_in'],
                 output_keys=['foo_counter_out']):
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Identify Corner')
        time.sleep(2)
        return 'sucess'


class IdentifyCorner(smach.State):

    def __init__(self,
                 outcomes=['sucess'],
                 input_keys=['baxter'],
                 output_keys=['foo_counter_out'],baxter=None):
        self.baxter=baxter
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=['baxter'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Identify Corner')

        #TODO: ask which corner to grasp

        # Initialize position
        self.baxter.initialize()

        time.sleep(2)
        return 'sucess'


# define state Bar
class GraspCorner(smach.State):

    def __init__(self,baxter=None,corner_id=None):
        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=['baxter','corner_id'])

        self.baxter = baxter
        self.corner_id = corner_id

    def execute(self, userdata):
        # Ask human to start grasping
        start_grasp = 'n'
        while start_grasp != 'y':
            start_grasp = raw_input("Start Grasping? [y]/[n]")

        # Verify that the corner is visible
        # rospy.loginfo('Checking Corner Visibility')
        # corner = self.corner_id
        # visible = self.baxter.check_marker_visibility(corner)
        #
        # while not visible:
        #     rospy.loginfo('Corner NOT Visible')
        #     rospy.sleep(1)
        #     visible = self.baxter.check_marker_visibility(corner)

        # Grasp corner
        rospy.loginfo('Executing state Grasp Corner')
        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        self.baxter.propose_corner()
        self.corner_id = self.baxter.active_corner

        self.baxter.grasp(self.corner_id)  # int

        # Human will tell "CLOSE"
        close = 'n'
        while close != 'y':
            close = raw_input("Close gripper? [y]/[n]")

        self.baxter.close_gripper()
        time.sleep(2)

        return 'sucess'


# define state Bar
class Stretch(smach.State):

    def __init__(self,baxter):
        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=['baxter'])
        self.baxter = baxter

    def execute(self, userdata):
        rospy.loginfo('Executing state Stretch')

        # Stretch
        self.baxter.pull()
        time.sleep(0.5)

        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

        return 'sucess'


class Fold(smach.State):

    def __init__(self,baxter,corner_id,edge,closest_corners,final_corners):
        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=['corner_id','edge','baxter','closest_corners','final_corners'])

        self.baxter=baxter
        self.corner_id=corner_id
        self.edge=edge
        self.closest_corners=closest_corners
        self.final_corners=final_corners

    def execute(self, userdata):
        rospy.loginfo('Executing state Fold')
        time.sleep(0.5)
        # Place
        corner = self.corner_id
        edge = self.edge

        self.baxter.place(self.closest_corners[corner], self.final_corners[corner], edge)  # Vector3, Vector3, float

        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        return 'sucess'


class Goal(smach.State):

    def __init__(self, baxter):
        smach.State.__init__(self,
                             outcomes=['sucess'],
                             input_keys=['baxter'])
        self.baxter = baxter

    def execute(self, userdata):
        rospy.loginfo('Executing state GOAL')
        # Open gripper
        self.baxter.open_gripper()
        time.sleep(1)
        # Added upwards movement
        self.baxter.move_along_line(np.array([0., 0., 1]), 0.04)

        self.baxter.initialize()
        # rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

        # Ask human whether to keep grasping
        keep_grasping = 'n'
        while keep_grasping != 'y':
            keep_grasping = raw_input("Grasp Another Corner? [y]/[n]")

        return 'sucess'


def sub_state_machine_fold_corner(counter=0, final_flag=False,baxter=None,edge=None,final_corners=None,closest_corners=None,corner_id=None):
    smach.StateMachine.add(
        'IdentifyCorner_' + str(counter),
        IdentifyCorner(baxter=baxter),
        transitions={'sucess': 'GraspCorner_' + str(counter)})

    smach.StateMachine.add('GraspCorner_' + str(counter),
                           GraspCorner(baxter=baxter,corner_id=corner_id),
                           transitions={
                               'sucess': 'Stretch_' + str(counter),
                           })

    smach.StateMachine.add('Stretch_' + str(counter),
                           Stretch(baxter=baxter),
                           transitions={
                               'sucess': 'Fold_' + str(counter),
                           })

    smach.StateMachine.add('Fold_' + str(counter),
                           Fold(baxter, corner_id, edge, closest_corners, final_corners),
                           transitions={'sucess': 'Goal_' + str(counter)})

    if not final_flag:
        smach.StateMachine.add(
            'Goal_' + str(counter),
            Goal(baxter),
            transitions={'sucess': 'IdentifyCorner_' + str(counter + 1)})
    else:
        smach.StateMachine.add('Goal_' + str(counter),
                               Goal(baxter),
                               transitions={'sucess': 'folded'})


def main():
    rospy.init_node('folding_state_machine')

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

    # Get Box estimations from vision
    A = Vector3()
    A.x = 0.872
    A.y = -0.05
    A.z = -0.134

    B = Vector3()
    B.x = 0.73
    B.y = 0.09
    B.z = -0.14

    C = Vector3()
    C.x = 0.58
    C.y = 0.
    C.z = -0.14

    D = Vector3()
    D.x = 0.6
    D.y = -0.001
    D.z = -0.14

    closest_corners = [A, B, C, D]
    final_corners = [C, D, B, A]
    edge = 0.08

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
            sub_state_machine_fold_corner(counter=0,baxter=baxter,edge=edge,final_corners=final_corners,corner_id=0,closest_corners=closest_corners)
            sm_sub.userdata.corner_id = 1
            sub_state_machine_fold_corner(counter=1,baxter=baxter,edge=edge,final_corners=final_corners,corner_id=1,closest_corners=closest_corners)
            sm_sub.userdata.corner_id = 2
            sub_state_machine_fold_corner(counter=2,baxter=baxter,edge=edge,final_corners=final_corners,corner_id=2,closest_corners=closest_corners)
            sm_sub.userdata.corner_id = 3
            sub_state_machine_fold_corner(counter=3, final_flag=True,baxter=baxter,edge=edge,final_corners=final_corners,corner_id=3,closest_corners=closest_corners)

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