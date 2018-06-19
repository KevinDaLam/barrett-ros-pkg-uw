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

class BHand(object):

    def __init__(self):
        pass

    def close():
        """
        Uses a service call to close the WAM hand (all three fingers on the same
        side making a fist).

        """
        close_hand_srv = rospy.ServiceProxy('/bhand/close_grasp', Empty)
        try:
            resp1 = close_hand_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Wait until the hand is closed before doing anything else.
        rospy.sleep(2)

    def close_spread():
        """
        Uses a service call to close the WAM hand (move all three fingers to the
         same side).

        """
        close_hand_srv = rospy.ServiceProxy('/bhand/close_spread', Empty)
        try:
            resp1 = close_hand_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def open():
        """
          Uses a service call to open the WAM hand (all three fingers on the same
        side, stretched out).
        """

        close_hand_srv = rospy.ServiceProxy('/bhand/open_grasp', Empty)
        try:
            resp1 = close_hand_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Wait until the hand is opened before doing anything else.
        rospy.sleep(2)