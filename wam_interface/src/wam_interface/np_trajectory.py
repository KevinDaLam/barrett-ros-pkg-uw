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

NUM_DOF = 7
NUM_AXES = 3
VELOCITY_LIMIT = 0.5

def validate_trajectory(trajectory, frequency_of_trajectory, traj_type):

  if traj_type == 'joint':
    if trajectory.shape[1] != NUM_DOF:
      return False
  elif traj_type == 'cart':
    if trajectory.shape[1] != NUM_AXES:
      return False

  vel_lims = np.diff(trajectory, axis=0)
  vel_lims = np.append(vel_lims, [[x for x in vel_lims[-1,:]]], axis = 0)
  vel_lims = vel_lims * frequency_of_trajectory
  vel_lims = np.absolute(vel_lims)

  if vel_lims.all() > 0.5:
    raise ValueError("One or more of the values in the specified velocities"
                     "Exceed 0.5 meter / second. The robot won't like this."
                     "Adjust the trajectory so that each point can be "
                     "reached without exceeding this limit.")

  return True

def create_joint_trajectory_from_csv(file_name, frequency_of_trajectory=250):

  trajectory = np.genfromtxt(file_name, delimiter=',')

  if validate_trajectory(trajectory, frequency_of_trajectory, 'joint'):
    return trajectory
  else:
    raise ReferenceError("Invalid Trajectory")


def create_cart_trajectory_from_csv(file_name, frequency_of_trajectory=250):

  trajectory = np.genfromtxt(file_name, delimiter=',')

  if validate_trajectory(trajectory, frequency_of_trajectory, 'cart'):
    return trajectory
  else:
    raise ReferenceError("Invalid Trajectory")


def generate_trajectory_from_equation(equation, start_t, end_t, duration_of_trajectory, frequency_of_trajectory):
  pass