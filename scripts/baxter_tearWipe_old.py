#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter Tear Wipe
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]



def main():
    """Baxter Wave Demo
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("baxter_wave_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")


	###TODO SETUP POINTS FOR TRAJECTORY
    startPos = {
		'left': [0.657694262054, 0.216674786041, -1.60224293112, 1.90980607874, 1.13706325772, 1.3161555145, 2.82866056963],
		'right': [-0.325587421857, 0.126553414856, 1.45613126124, 1.86186917917, -1.0845244158, 1.18576714768, -0.261543724036]
		}

    pos1 = {
		'left': [-0.322519460284, 0.133456328394, -1.97768472852, 2.19320902897, -0.39269908125, 0.316000041943, 3.04917030764],
		'right': [0.444470932782, 0.19021361748, 2.03214104643, 2.13415076871, -0.131922347607, 0.373524321423, 0.590582602661]
	}

    pos2 = {
		'left': [0.255407800891, 0.717136017517, -2.00836434424, 2.46472362812, -0.22012624281, 0.91041759657, 3.04955380283],
		'right': [0.102776712671, 0.7190534935, 2.63346151459, 2.19435951456, -0.164519439313, 1.32842736079, 1.01204382365]
	}

    pos3 = {
		'left': [-0.772359325818, 1.54548564203, -0.112364092584, -1.82428664991, 2.21890320714, 0.426830153741, -0.0567572890869],
		'right': [0.636602026245, 2.29828671282, -0.107762150226, 0.706781647211, 2.36501487702, -0.147262155469, 1.01587877562]
	}
	

    if limb == 'left':
        otherArm = 'right'
    else:
        otherArm = 'left'
    traj = Trajectory(limb)
    trajOther = Trajectory(otherArm)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

    p1 = pos1[limb]
    p2 = pos2[limb]
    p3 = pos3[limb]
    sPos = startPos[limb]
    sPosOther = startPos[otherArm]

	#TODO ADD POINTS FOR TRAJECTORIES
    traj.add_point(sPos, 7.0)
    trajOther.add_point(sPosOther, 7.0)
    
    traj.add_point(p1, 10.0)
    traj.add_point(p2, 12.0)
    traj.add_point(p1, 14.0)
    traj.add_point(p2, 16.0)
    traj.add_point(sPos, 19.0)
    #traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)
    ''' '''
    trajOther.start()

    traj.start()
    traj.wait(22.0)
    trajOther.wait(9.0)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
