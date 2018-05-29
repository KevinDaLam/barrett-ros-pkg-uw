#!/usr/bin/env python

# WAMPy. Control a Barrett WAM Robot Cartesian using Python.
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



import wam_joint
from geometry_msgs.msg import PoseStamped

def get_wam_cartesian_coordinates():

	msg = rospy.wait_for_message('/wam/pose', PoseStamped)
    return list(msg.position)

def create_cartesian_line_trajectory(start_position, end_positon, duration_of_trajectory, frequency_of_trajectory):

	frequency_of_ros_messages = frequency_of_trajectory # in Hz.
    number_of_way_points = duration_of_trajectory * frequency_of_ros_messages
    number_of_coords = start_position.__len__()
    trajectory = np.zeros((number_of_coords, number_of_way_points))

    for i in xrange(number_of_joints):
        trajectory[i] = np.linspace(start_position[i], end_position[i],
                                    number_of_way_points)
    trajectory = trajectory.T.copy()
    vel_lims = np.diff(trajectory, axis=0)
    #Because this is discrete differentiation,
    # the last value is missing: len(vel_lims) = len(trajectory) - 1
    # so we just repeat the last calculated velocity.
    vel_lims = np.append(vel_lims, [[x for x in vel_lims[-1,:]]], axis = 0)
    vel_lims = vel_lims * frequency_of_trajectory
    vel_lims = np.absolute(vel_lims)

    if vel_lims.all() > 0.5:
        raise ValueError("One or more of the values in the specified velocities"
                         "Exceed 0.5 meter / second. The robot won't like this."
                         "Adjust the trajectory so that each point can be "
                         "reached without exceeding this limit.")
    return trajectory, vel_lims


def send_cartesian_trajectory(trajectory, velocities, frequency = 250):

	pub = rospy.Publisher("/wam/cart_pos_cmd", RTCartPos)
    #If wam_node is running, it will be connected to this publisher.
    #Mostly this loop is here because you want to make sure the publisher
    #gets set up before it starts sending information.
    while pub.get_num_connections() < 1:
        print "Waiting on the publisher to go up."
        rospy.sleep(0.5)

    trajectory_length = trajectory.__len__()
    finished = False
    traj_row = 0
    message_for_service = RTCartPos()

    r = rospy.Rate(frequency)

    while not rospy.is_shutdown() and not finished:
        message_for_service.position = trajectory[traj_row]
        message_for_service.rate_limits = velocities[traj_row]
        traj_row += 1
        pub.publish(message_for_service)
        if traj_row == trajectory_length - 1:
            finished = True
        r.sleep()

def create_and_send_wam_trajectory(wam_start, wam_end, duration, frequency=250):
    """ Create and send a trajectory that's a linear interpolation between
    wam_start and wam_end that lasts duration seconds send at frequency.

    args:
      wam_start: a 1x<DOF> array of joint coordinates.
      wam_end: a 1x<DOF> array of joint coordinates.
      duration: A float. The duration of the trajectory in seconds.
      frequency: The frequency of values. With the default, a new position is
        specified for every 0.004 seconds. 250 Hz is what wam_node is expecting.

    returns:
      None
      """

    cart_traj, cart_vels = create_cartesian_line_trajectory(wam_start, wam_end,
                                                     duration, frequency)
    send_cartesian_trajectory(joint_traj, joint_vels, frequency)

