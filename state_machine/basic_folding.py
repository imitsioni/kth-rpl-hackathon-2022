#!/usr/bin/env python

from typing import List
import roslib
import rospy
import smach
import smach_ros
import time

corner_id = {
    0: "bottom_left",
    1: "bottom_right",
    2: "top_left",
    3: "top_right"
}

corner_id_topic = "/corner_id"
position_corner_topic = "/position_corner"


# define state Fooe
class IdentifyCorner(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sucess'],
                             output_keys=['identify_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Identify Corner')
        rospy.wait_for_message(corner_id_topic, Int)
        rospy.Subscriber(corner_id_topic,
                         List,
                         userdata.identify_out,
                         queue_size=1)
        rospy.loginfo('Identified Corner ID:%f' % userdata.identify_out)
        return 'sucess'


# define state Bar
class Bar(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
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

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO',
                               Foo(),
                               transitions={
                                   'outcome1': 'BAR',
                                   'outcome2': 'outcome4'
                               },
                               remapping={
                                   'foo_counter_in': 'sm_counter',
                                   'foo_counter_out': 'sm_counter'
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