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

    lMacWristFlick = [
        [-0.683864181714, 0.0, -1.65324779225, 0.0, 1.63637400361, -0.0, -0.0874369048096],
        [-0.683864181714, 0.0, -1.65324779225, 0.0, 0.0, 1.21874773458, 0.103160207867],
        [-0.683864181714, 0.0, -1.65324779225, 0.0, -1.5600584595, -0.0, -0.0732475825378]
    ]

    lMacArmFold = [
        [-0.416475783435, -0.0157233030579, -1.64442740273, 1.92552938179, 0.0138058270752, -0.075548553717, -0.0387330148499],
        [-0.494708803528, 0.768140878656, -1.46111669879, 1.69619925427, 0.802271951147, 0.0444854427979, -0.0836019528442]
    ]
    
    rMacWristFlick = [translateCoords(i) for i in lMacWristFlick]

    macarena = {}
    macarena["Left_Wrist_Flick"] = lMacWristFlick
    macarena["Right_Wrist_Flick"] = rMacWristFlick
    macarena["Left_Arm_Drop"] = [-0.683864181714, 0.4, -1.65324779225, 0.0, -1.5600584595, -0.0, -0.0732475825378]

    macarena["Left_Arm_Head"] = [0.630849598297, -0.183310703943, -2.49348576786, 2.58744209101, 1.36961681938, -1.44616038613, 1.33379629354]
    macarena["Right_Arm_Head"]= translateCoords(macarena["Left_Arm_Head"])


	

    if limb == 'left':
        otherArm = 'right'
    else:
        otherArm = 'left'
    lTraj = Trajectory(limb)
    rTraj = Trajectory(otherArm)
    rospy.on_shutdown(lTraj.stop)
    rospy.on_shutdown(rTraj.stop)
    
    t = 0.0
    t = WristFlick(rTraj, lTraj, t, macarena)

    lMacArmFold[0] = translateCoords(lMacArmFold[0])
    lTraj.add_point(macarena["Left_Arm_Drop"], t+5.0)
    rTraj.add_point(lMacArmFold[0], t+5.0)
    rTraj.add_point(lMacArmFold[0], t+10.0)
    lTraj.add_point(lMacArmFold[1], t+10.0)
    rTraj.add_point(macarena["Right_Arm_Head"], t+20.0)
    

    

    lTraj.start()
    rTraj.start()
    lTraj.wait(30.0)
    rTraj.wait(30.0)
    
    #send_image("images/neutral.PNG")
    print("Exiting - Joint Trajectory Action Test Complete")

def WristFlick(rTraj, lTraj, CT, macarena):
    lTraj.add_point(macarena["Left_Wrist_Flick"][0], CT+3.0)
    rTraj.add_point(macarena["Right_Wrist_Flick"][0], CT+3.0)

    
    rTraj.add_point(macarena["Right_Wrist_Flick"][0], CT+5.0)
    rTraj.add_point(macarena["Right_Wrist_Flick"][1], CT+5.8)
    rTraj.add_point(macarena["Right_Wrist_Flick"][2], CT+6.6)
    rTraj.add_point(macarena["Right_Wrist_Flick"][2], CT+9.0)

    lTraj.add_point(macarena["Left_Wrist_Flick"][0], CT+7.4)
    lTraj.add_point(macarena["Left_Wrist_Flick"][1], CT+8.2)
    lTraj.add_point(macarena["Left_Wrist_Flick"][2], CT+9.0)

    return CT+9.0

def ARMTHING():
    pass

# Speaks the given string
def tts(text):
   text = "'"+text+":'"
   return os.system("espeak  -s 155 -ven-us+f3 -p 30 -a 200 "+text+" " )

def translateCoords(coords):
    translate = [-1,1,-1,1,-1,1,1]
    newCoords = []
    for i in range(7):
        newCoords.append(coords[i]*translate[i])
    return newCoords

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





def macarena(lTraj, rTraj, t):

    dt = 1.0
    #TEAR WIPE POSITIONS
    


    p1 = tw1[limb]
    p2 = tw2[limb]
    p3 = tw3[limb]


    traj.add_point(p1, t)
    t+=dt
    traj.add_point(p2, t)
    t+=dt
    traj.add_point(p1,t)
    t+=dt
    traj.add_point(p2, t)
    t+=dt+.5
    return t

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
