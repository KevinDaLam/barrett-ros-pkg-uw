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

import wam_cart
import wam_joint
import wam_bhand

class WAMInterface(object):

  def __init__(self, wam_joint_home=[0, -1.94, 0, 2.79, 0, 0, 0]):
    self.joint = wam_joint.Joint(wam_joint_home)
    self.cartesian = wam_cart.Cartesian()
    self.hand = wam_bhand.BHand()