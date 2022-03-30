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
		time.sleep(2)
		return 'sucess'


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
		time.sleep(2)
		return 'sucess'


# define state Bar
class GraspCorner(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Grasp Corner')
		# rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
		time.sleep(2)
		return 'sucess'


# define state Bar
class Stretch(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Stretch')
		time.sleep(2)

		# rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

		return 'sucess'


class Fold(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Fold')
		time.sleep(2)

		# rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
		return 'sucess'


class Goal(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state GOAL')
		# rospy.loginfo('Counter = %f' % userdata.bar_counter_in)
		return 'sucess'


def main():
	rospy.init_node('smach_example_state_machine')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['Goal'])
	sm.userdata.sm_counter = 0
	sm.userdata.corner_id = 0
	# sm.userdata.corner_positon

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('Start',
		                       IdentifyCorner(),
		                       transitions={'sucess': 'GraspCorner'})

		smach.StateMachine.add('IndentifyCorner',
		                       IdentifyCorner(),
		                       transitions={'sucess': 'GraspCorner'})

		smach.StateMachine.add('GraspCorner',
		                       GraspCorner(),
		                       transitions={
		                           'sucess': 'Stretch',
		                       })

		smach.StateMachine.add('Stretch',
		                       Stretch(),
		                       transitions={
		                           'sucess': 'Fold',
		                       })

		smach.StateMachine.add('Fold', Fold(), transitions={'sucess': 'Goal'})
		#    remapping={'bar_counter_in': 'sm_counter'})

		smach.StateMachine.add('Goal', Goal(), transitions={
		    'sucess': 'Start',
		})

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
