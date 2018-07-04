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

# http://docs.ros.org/api/rosbag/html/python/rosbag.bag.Bag-class.html#read_messages
bag = rosbag.Bag('../bagfiles/latest_wam_run.bag')

# Returns tuple of (str, genpy.Message, genpy.Time)
bag_
bag_joint_states = bag.read_messages(topics='/wam/joint_states')