#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

from language_server import LanguageServer


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
		time.sleep(1)
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
		time.sleep(1)
		return 'sucess'


# define state Bar
class Stretch(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Stretch')
		time.sleep(1)

		# rospy.loginfo('Counter = %f' % userdata.bar_counter_in)

		return 'sucess'


class Fold(smach.State):

	def __init__(self):
		smach.State.__init__(self,
		                     outcomes=['sucess'],
		                     input_keys=['bar_counter_in'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Fold')
		time.sleep(1)

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


def sub_state_machine_fold_corner(counter=0, final_flag=False):
	smach.StateMachine.add('IdentifyCorner_' + str(counter),
	                       IdentifyCorner(),
	                       transitions={'sucess': 'GraspCorner_' + str(counter)})

	smach.StateMachine.add('GraspCorner_' + str(counter),
	                       GraspCorner(),
	                       transitions={
	                           'sucess': 'Stretch_' + str(counter),
	                       })

	smach.StateMachine.add('Stretch_' + str(counter),
	                       Stretch(),
	                       transitions={
	                           'sucess': 'Fold_' + str(counter),
	                       })

	smach.StateMachine.add('Fold_' + str(counter),
	                       Fold(),
	                       transitions={'sucess': 'Goal_' + str(counter)})

	if not final_flag:
		smach.StateMachine.add(
		    'Goal_' + str(counter),
		    Goal(),
		    transitions={'sucess': 'IdentifyCorner_' + str(counter + 1)})
	else:
		smach.StateMachine.add('Goal_' + str(counter),
		                       Goal(),
		                       transitions={'sucess': 'folded'})


def main():
	rospy.init_node('folding_state_machine')

	# Create a SMACH state machine
	sm_top = smach.StateMachine(outcomes=['final_outcome'])
	sm_top.userdata.sm_counter = 0
	sm_top.userdata.corner_id = 0
	# sm.userdata.corner_positon
	lang_server = LanguageServer()

	# Open the container
	counter_fold = 0
	max_goal_counter = 3
	with sm_top:
		# Add states to the container

		smach.StateMachine.add('Start',
		                       Start(),
		                       transitions={'sucess': 'FinalGoal'})

		sm_sub = smach.StateMachine(outcomes=['folded'])

		with sm_sub:
			sub_state_machine_fold_corner(0)
			sub_state_machine_fold_corner(1)
			sub_state_machine_fold_corner(2)
			sub_state_machine_fold_corner(counter=3, final_flag=True)

		smach.StateMachine.add('FinalGoal',
		                       sm_sub,
		                       transitions={'folded': 'final_outcome'})

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
