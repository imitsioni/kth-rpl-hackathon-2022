#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time


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
        return 'outcome1'


class IdentifyCorner(smach.State):

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
        return 'outcome1'


# define state Bar
class GraspCorner(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sucess', 'fail'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp Corner')
        rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        return 'outcome1'


# define state Bar
class Stretch(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sucess', 'fail'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        return 'outcome1'


class Fold(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sucess', 'fail'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
        return 'outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0
    sm.uderdata.corner_id = 0
    # sm.userdata.corner_positon

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IndentifyCorner',
                               Foo(outcomes=['outcome1', 'outcome2']),
                               transitions={
                                   'outcome1': 'BAR',
                                   'outcome2': 'FOO2'
                               },
                               remapping={
                                   'foo_counter_in': 'sm_counter',
                                   'foo_counter_out': 'corner_id'
                               })

        smach.StateMachine.add('FOO2',
                               Foo(outcomes=['outcome1']),
                               transitions={
                                   'outcome1': 'BAR',
                               })

        smach.StateMachine.add('BAR',
                               Bar(),
                               transitions={'outcome1': 'FOO'},
                               remapping={'bar_counter_in': 'sm_counter'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()