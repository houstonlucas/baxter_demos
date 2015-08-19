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

from sensor_msgs.msg import (
    Image,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

import os
import sys
import cv2
import cv_bridge
import random

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



    startPos = {
		'left': [0.657694262054, 0.216674786041, -1.60224293112, 1.90980607874, 1.13706325772, 1.3161555145, 2.82866056963],
		'right': [-0.325587421857, 0.126553414856, 1.45613126124, 1.86186917917, -1.0845244158, 1.18576714768, -0.261543724036]
		}
    
	

    if limb == 'left':
        otherArm = 'right'
    else:
        otherArm = 'left'
    traj = Trajectory(limb)
    trajOther = Trajectory(otherArm)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

   
    sPos = startPos[limb]
    sPosOther = startPos[otherArm]
    
    t=5.0
    dt = 0.3
    
    traj.add_point(sPos, t)
    trajOther.add_point(sPosOther, t)
    traj.start()
    trajOther.start()
    trajOther.wait(9.0)
    traj.wait(9.0)
    while 1:
        send_image("images/neutral.PNG")
        gameMotion(traj, limb)
        traj.add_point(sPos,1.5)
        traj.start()
        traj.wait(2.0)

    print("Exiting - Joint Trajectory Action Test Complete")

def gameMotion(traj, limb):
    pos1 = [0.336708782556, 0.428364134528, -1.77289829357, 1.71767498527, 0.614742800043, 0.591733088251, 3.04917030764]
    pos2 = [0.385796167712, 0.434500057672, -1.65746623942, 1.63790798439, 1.11060208916, 1.05307780968, 2.63537899058]
    traj.clear(limb)
    traj.add_point(pos1,2.0)
    traj.add_point(pos2,3.0)
    
    traj.add_point(pos1,4.0)
    traj.add_point(pos2,5.0)
    
    traj.add_point(pos1,6.0)
    traj.add_point(pos2,7.0)
    
    traj.start()
    traj.wait(7.0)
    options = ['rock.jpeg','paper.jpg','scissors.jpeg']
    choice = int(random.random()*3)
    send_image("images/"+options[choice])

# Speaks the given string
def tts(text):
   text = "'"+text+":'"
   return os.system("espeak  -s 155 -ven-us+f3 -p 30 -a 200 "+text+" " )

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



def send_image(path):
    if not os.access(path, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (path,))
        return 1
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path,1)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

if __name__ == "__main__":
    main()
