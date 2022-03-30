#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros


# define state Foo
class Foo(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
		self.counter = 0

	def execute(self, userdata):
		rospy.loginfo('Executing state FOO')
		if self.counter < 3:
			self.counter += 1
			return 'outcome1'
		else:
			return 'outcome2'


# define state Bar
class Bar(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAR')
		return 'outcome1'


# define state Bas
class Bas(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome3'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAS')
		return 'outcome3'


def main():
	rospy.init_node('smach_example_state_machine')

	# Create the top level SMACH state machine
	sm_top = smach.StateMachine(outcomes=['BAS'])

	# Open the container
	with sm_top:

		smach.StateMachine.add('BAS', Bas(), transitions={'outcome3': 'SUB'})

		# Create the sub SMACH state machine
		sm_sub = smach.StateMachine(outcomes=['outcome4'])

		# Open the container
		with sm_sub:

			# Add states to the container
			smach.StateMachine.add('FOO',
			                       Foo(),
			                       transitions={
			                           'outcome1': 'BAR',
			                           'outcome2': 'outcome4'
			                       })
			smach.StateMachine.add('BAR', Bar(), transitions={'outcome1': 'FOO'})

		smach.StateMachine.add('SUB', sm_sub, transitions={'outcome4': 'BAS'})
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
	sis.start()

	# Execute SMACH plan
	outcome = sm_top.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()
