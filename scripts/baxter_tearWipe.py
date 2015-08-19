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

import time

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


#### IKsolver ##########

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


import struct

from std_msgs.msg import Header


###################




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

    cubeHoldPos = {
        'left': [-0.431048600903, -0.524621428857, -0.763922431494, 0.477068024487, -0.958354496136, 1.60569438788, -0.233165079492],
        'right': [0.431048600903, -0.524621428857, 0.763922431494, 0.477068024487, 0.958354496136, 1.60569438788, -0.233165079492]
    }
    
	

    if limb == 'left':
        otherArm = 'right'
    else:
        otherArm = 'left'
    traj = Trajectory(limb)
    trajOther = Trajectory(otherArm)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)
   

    btnHandler = ButtonHandler()

    sPos = startPos[limb]
    sPosOther = startPos[otherArm]
    t=3.0
    dt = 0.3

    btnHandler.loop = True
    while btnHandler.loop and not rospy.is_shutdown():
        time.sleep(0.1)
    btnHandler.loop = True

    '''
    leftHandPositionCubeDOWN = [0.56, 0.04, 0.37, '-z']
    #leftHandPositionCubeUP = 
    

    x=0.6
    y=0.1
    z=0.3

    jointPosRight = getJointPos(otherArm, x,-y, z, '+y')
    jointPosLeft = getJointPos(limb, 0.56,0.04,.23,'+z')

    
    traj.add_point(jointPosLeft, 3.0)
    trajOther.add_point(jointPosRight, 3.0)
    traj.start()
    trajOther.start()
    traj.wait(20.0)
    trajOther.wait(20.0)

    '''
    
   
    traj.add_point(sPos, 5.0)
    t+=2.0
    trajOther.add_point(sPosOther, t)
   
    traj.add_point(sPos, t)
    t+=3.0
    t = wave(traj, limb, t)
    traj.add_point(sPos, t)
    t+= 3.0
  
    
    
    send_image("images/happy.PNG")
    trajOther.start()
    traj.start()
    traj.wait(15.0)
    #tts("Hello there")
    #print("Happy")
    trajOther.wait(9.0)
    traj.clear(limb)
    trajOther.clear(otherArm)

    btnHandler.loop = True
    while btnHandler.loop and not rospy.is_shutdown():
       
        time.sleep(0.1)
    btnHandler.loop = True

    t = 3.0
    traj.add_point(sPos, t)
    t+= 2.0
    t = cry(traj, limb, t)
    traj.add_point(sPos, t)
    

    send_image("images/sad.PNG")
    traj.start()
    #print("SAD!!")
    traj.wait(15.0)
    trajOther.wait(9.0)
    send_image("images/neutral.PNG")
    '''
    print("Exiting - Joint Trajectory Action Test Complete")
    '''


def getJointPos(limb, X,Y,Z, direction):
    
    iksString = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(iksString, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    positions = { '-y':[1.0, 0.0, 0.0, 1.0], '+y':[0.0 ,1.0 ,1.0 ,0.0],
                  '-y-90':[-1.0, 1.0, -1.0, -1.0], '-y+90':[-1.0, -1.0, 1.0, -1.0],
                  '+y-90':[1.0, -1.0, -1.0, -1.0], '+y+90':[-1.0, -1.0, -1.0, 1.0],
                  '-z':[1.0, 1.0, 0.0, 0.0], '+z':[0.0, 0.0, 1.0, 1.0]}
    
    position = positions[direction]

    ikTestPose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=X,
                    y=Y,
                    z=Z,
                ),
                orientation=Quaternion(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                    w=position[3],
                ),
            ),
        )


    limb_joints = []

    ikreq.pose_stamp.append(ikTestPose)
    try:
        rospy.wait_for_service(iksString, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        

    
    jointPos = [limb_joints[i] for i in [limb+'_s0', limb+'_s1', limb+'_e0', limb+'_e1', limb+'_w0', limb+'_w1', limb+'_w2']]
    return jointPos

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



def goZero(traj, otherTraj, t):
    pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    traj.add_point(pos, 5.0)
    otherTraj.add_point(pos, 5.0)
    traj.start()
    otherTraj.start()
    traj.wait(7.0)
    otherTraj.wait(7.0)

def cry(traj, limb, t):

    dt = 1.0
    #TEAR WIPE POSITIONS
    tw1 = {
	    'left': [-0.322519460284, 0.133456328394, -1.97768472852, 2.19320902897, -0.39269908125, 0.316000041943, 3.04917030764],
		'right': [0.444470932782, 0.19021361748, 2.03214104643, 2.13415076871, -0.131922347607, 0.373524321423, 0.590582602661]
	}

    tw2 = {
		'left': [0.255407800891, 0.717136017517, -2.00836434424, 2.46472362812, -0.22012624281, 0.91041759657, 3.04955380283],
		'right': [0.102776712671, 0.7190534935, 2.63346151459, 2.19435951456, -0.164519439313, 1.0842736079, 1.01204382365]
	}

    tw3 = {
		'left': [-0.772359325818, 1.54548564203, -0.112364092584, -1.82428664991, 2.21890320714, 0.426830153741, -0.0567572890869],
		'right': [0.636602026245, 2.29828671282, -0.107762150226, 0.706781647211, 2.36501487702, -0.147262155469, 1.01587877562]
	}

    p1 = tw1[limb]
    p2 = tw2[limb]
    p3 = tw3[limb]

    traj.add_point(p1, t)
    t+=dt
    traj.add_point(p2, t)
    t+=dt
    traj.add_point(p1, t)
    t+=dt
    traj.add_point(p2, t)
    t+=dt+.5
    return t

def wave(traj, limb,  t):
    #WAVE POSITIONS
    #TODO right wave does work

    newWave1 = [0.217825271631, 1.05307780968, -1.98995657481, 2.30902457833, 3.06719458187, -0.450990351123, 3.04878681244]
    newWave2 = [0.416475783435, 1.05307780968, -2.32704885256, 2.17671873552, 2.31055855911, -0.388864129285, 3.04840331724]
   

    dt = 1.0
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt + 2.0
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


class ButtonHandler:
    
    def __init__(self,limb = 'left'):
        self.loop = True
        self.navigator_io = baxter_interface.Navigator(limb)
        # Navigator scroll wheel button press
        self.navigator_io.button0_changed.connect(self.onButtonPress)
        self.navigator_io.wheel_changed.connect(self.onWheelChange)
        
        self.oldWheelValue = self.navigator_io.wheel
        self.wheelChange = 0

    def onWheelChange(self, vale):
        current = self.navigator_io.wheel
        difference = current - self.oldWheelValue
        if difference > 128: difference = 255-difference
        if difference < -128: difference = difference + 255
        self.wheelChange = difference

    def onButtonPress(self, value):
        if value:
            self.loop = False


if __name__ == "__main__":
    main()
