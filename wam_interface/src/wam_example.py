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

from wam_joint import *

if __name__ == "__main__":

    node = rospy.init_node("motion_control")

    # # Following this are some example locations.
    # #To get these locations, in a terminal, run:
    # # "roscore"
    # # then in a separate terminal
    # # "rosrun wam_node wam_node"
    # # Follow the onscreen instructions.
    # # Now it's ready to accept commands.
    # # To make it so that you can move it by hand, in a separate terminal:
    # # "rosservice call /wam/hold_joint_pos false"
    # # Now move it to wherever you're interested in (e.g. a part pickup or a part drop off)
    # # and you can get the joint coordinates by typing
    # # "rostopic echo /wam/joint_states"
    ## (or by running the function get_wam_joint_coordinates)
    # # This will keep spewing out the current joint coordinates.
    # # Kill it when you're done by hitting CTRL + C.

    experiment_home_point = [0, -1.8, 0, 1]

    experiment_pickup_point = [1, -1.8, 0, 1]

    experiment_dropoff_point = [1, -1.8, 0, 2]

    wam_home =  [1, 0, 1.5, 2]

    move_wam_from_current_location(wam_home, 2)

    move_wam_from_current_location(experiment_home_point, 2)
    move_wam_from_current_location(experiment_pickup_point, 2)
    close_wam_hand()
    move_wam_from_current_location(experiment_dropoff_point, 2)
    open_wam_hand()
    move_wam_from_current_location(experiment_home_point, 2)
    close_wam_hand_spread() 
    move_wam_from_current_location(wam_home, 2)

