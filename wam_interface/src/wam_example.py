#!/usr/bin/env python

# WAMPy. Control a Barrett WAM Robot using Python.
# Copyright (C) 2018 Kevin Lam, Benjamin Blumer
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

from wam_interface.wam_interface import WAMInterface

if __name__ == "__main__":

    node = rospy.init_node("wam_example")

    wam_home =  [0, -1.94, 0, 2.79, 0, 0, 0]

    wam = WAMInterface(wam_home)

    experiment_home_point = [0, -1.94, 0, 2.79, 0, 0, 0]

    experiment_pickup_point = [1, -1.8, 0, 1, 0, 00, 0]

    experiment_dropoff_point = [1, -1.8, 0, 2, 0, 0, 0]

    wam.joint.move_from_current_location(wam_home, 2)
    wam.joint.move_from_current_location(experiment_home_point, 2)
    wam.joint.move_from_current_location(experiment_pickup_point, 2)

    wam.hand.close()

    wam.joint.move_from_current_location(experiment_dropoff_point, 2)

    wam.hand.open()

    wam.joint.move_from_current_location(experiment_home_point, 2)

    wam.hand.close_spread()

    wam.joint.move_from_current_location(wam_home, 2)