#!/usr/bin/env python

import time
import copy as copy_module

################ Util functions #####################


def get_pose_stamped_str(ps):
	return "[" + str(ps.pose.position.x) + ", " + str(
	    ps.pose.position.y) + ", " + str(ps.pose.position.z) + "]"


def copy_pose_stamped(msg_in):
	return copy_module.deepcopy(msg_in)


def plan_group(group, dest_pose, sleep_time, printed_str=""):

	curr_pose = group.get_current_pose()
	print(printed_str + " ---- Goal:" + get_pose_stamped_str(dest_pose) +
	      ", From:" + get_pose_stamped_str(curr_pose))
	group.set_start_state_to_current_state()
	group.set_pose_target(dest_pose)
	success = group.go(wait=True)

	group.stop()
	time.sleep(sleep_time)
	return success


def plan_cartesian_group(group, waypoint_list, sleep_time, printed_str=""):
	(plan, fraction) = group.compute_cartesian_path(waypoint_list, 0.01, 0.0)
	success = group.execute(plan, wait=True)

	group.stop()
	time.sleep(sleep_time)
	return success


################ End util functions #####################
