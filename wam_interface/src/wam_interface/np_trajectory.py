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

import numpy as np

def validate_trajectory(trajectory, type)

  return True

def create_joint_trajectory_from_csv(file_name):

  trajectory = np.genfromtxt(file_name, delimiter=',')

  if validate_trajectory(trajectory):
    return trajectory
  else:
    raise ReferenceError("Invalid File Format for Trajectory")


def create_cart_trajectory_from_csv(file_name):

  trajectory = np.genfromtxt(file_name, delimiter=',')


def generate_trajectory_from_equation(equation, start_t, end_t, duration_of_trajectory, frequency_of_trajectory):
  pass
