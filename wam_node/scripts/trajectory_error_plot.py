#!/usr/bin/env python

# WAMPy. Control a Barrett WAM Robot using Python.
# Copyright (C) 2018 Kevin Lam
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rosbag
import matplotlib.pyplot as plt
import rospy
import numpy as np


def main():

	# http://docs.ros.org/api/rosbag/html/python/rosbag.bag.Bag-class.html#read_messages
	bag = rosbag.Bag('../bagfiles/latest_wam_run.bag')

	bag_joint_cmds = bag.read_messages(topics='/wam/jnt_pos_cmd')
	bag_joint_states = bag.read_messages(topics='/wam/joint_states')

	cmd_times = np.empty((0,1))
	cmd_joints = np.empty((0,7))

	count = 0

	for topic, msg, t in bag_joint_cmds:
		cmd_times = np.append(cmd_times, t.to_sec())
		cmd_joints = np.vstack((cmd_joints, msg.joints))

	state_times = np.empty((0,1))
	state_joints = np.empty((0,7))

	for topic, msg, t in bag_joint_states:
		state_times = np.append(state_times, t.to_sec())
		state_joints = np.vstack((state_joints, msg.position))

	initial_cmd_time = cmd_times[0]

	print initial_cmd_time

	matching_timestamps = []


if __name__ == '__main__':
	main()