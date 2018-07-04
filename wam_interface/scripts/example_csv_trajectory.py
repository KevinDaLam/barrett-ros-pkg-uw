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

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import numpy as np

from wam_interface.np_trajectory import *
from wam_interface.wam_joint import Joint

def main():

	wam_home = [0, -1.8, 0, 1, 1.57, 1.57, 1.57]
	experiment_point = [2, -1, 1, 2, 1.57, 1.57, 1.57]

	test_array, vel_lims = Joint.create_trajectory(wam_home, experiment_point, duration_of_trajectory=2, frequency_of_trajectory=250)
	np.savetxt('files/test_trajectory.csv', test_array, delimiter=',')

	read_trajectory = create_joint_trajectory_from_csv('files/test_trajectory.csv', frequency_of_trajectory=250)


if __name__ == "__main__":
	main()