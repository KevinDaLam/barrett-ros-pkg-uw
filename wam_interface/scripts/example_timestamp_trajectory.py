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

import rospy
import numpy as np

file_name = 'files/test_trajectory.csv'

from wam_interface.wam_interface import WAMInterface

def main():

	# node = rospy.init_node("wam_example")

	node = rospy.init_node("wam_example")

	wam_home = [0, -1.94, 0, 2.79, 0, 0, 0]
	wam = WAMInterface(wam_home)

	time_trajectory = np.genfromtxt(file_name, delimiter=',')
	freq_trajectory, freq_vel_lims = wam.joint.convert_time_to_frequency(time_trajectory, frequency=250)
	np.set_printoptions(suppress=True)

	wam.joint.move_from_current_location(wam_home, 5)

	wam.joint.move_from_current_location(freq_trajectory[0], 5)
	
	wam.joint.send_trajectory(freq_trajectory, freq_vel_lims, frequency=250)

	wam.joint.move_from_current_location(wam_home, 5)

if __name__ == '__main__':
	main()
	