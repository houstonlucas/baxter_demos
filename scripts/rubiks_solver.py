#!/usr/bin/env python

"""
Baxter Rubik's Cube Solver
"""
import argparse
import sys
import os
import cv2
import cv_bridge
import numpy as np
import time
import copy

import rospy
import actionlib
import baxter_interface
from baxter_interface import CHECK_VERSION

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

##### IKSOLVER ##########
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
#########################

from CubeDetection import *

# Quit flag
quit = False

def main():
    """
    Baxter Rubik's Cube Solver
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-m', '--mode', required=False, choices=['test', 'demo'],
        help='Chooses a mode to run in'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    mode = args.mode
    if mode == None: mode = 'demo'

    if mode == 'demo':
        demoMode()
    elif mode == 'test':
        testMode()

def testMode():
    print("Running in test mode.")
    print("Initializing node... ")
    rospy.init_node("baxter_rubiks_demo")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit") 
    
    limb = 'left'
    otherLimb = 'right'

    btnHandler = ButtonHandler()

    traj = Trajectory(limb)
    trajOther = Trajectory(otherLimb)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

    left = traj.gripper
    right = trajOther.gripper

    left.calibrate()
    right.calibrate()

    #frontSideCW(traj, trajOther)
    #return

    moveArmsToStart(traj, trajOther)
    left.open()
    right.open()
    
    btnHandler.reset()
    while btnHandler.loop and not rospy.is_shutdown():
        offset = btnHandler.wheelChange
        current = left.position()
        left.command_position(current + offset)
        time.sleep(0.1)
    btnHandler.reset()
    

    #ignore = raw_input("continue")
    print("Grabbing Cube")

    # CLOSE GRIPPERS
    left.close()
    right.open()

    readInCubeState(traj, trajOther)
    moveToCubeHandlingPos(traj, trajOther, t = 2.0)
    rightSideCW(traj, trajOther )
    moveToCubeHandlingPos(traj, trajOther, t = 2.0 )
    cubeGrab( traj, trajOther )
    moveToCubeHandlingPos(traj, trajOther, t = 2.0 )

    holdOutCube( traj, trajOther )
    #frontSideCW(traj, trajOther)


    #FAILS HORRIBLY on left
    #cubeGrab(traj, trajOther)
    
    btnHandler.reset()
    while btnHandler.loop and not rospy.is_shutdown():
        offset = btnHandler.wheelChange
        current = left.position()
        left.command_position( current + offset )
        time.sleep(0.1)
    btnHandler.reset()

    left.open()
    right.open()

    print "exit"

def demoMode():
    print("Running in demo mode.")
    print("Initializing node... ")
    rospy.init_node("baxter_rubiks_demo")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    limb = 'left'
    otherLimb = 'right'

    btnHandler = ButtonHandler()

    traj = Trajectory(limb)
    trajOther = Trajectory(otherLimb)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

    left = traj.gripper
    right = trajOther.gripper
    
    left.calibrate()
    right.calibrate()

    while btnHandler.loop and not rospy.is_shutdown():
        offset = btnHandler.wheelChange
        current = left.position()
        left.command_position(current + offset)
        time.sleep(0.1)
    btnHandler.loop = True
    time.sleep(2.0)

    moveArmsToStart(traj, trajOther)
    left.open()
    right.open()

    btnHandler.loop = True
    while btnHandler.loop and not rospy.is_shutdown():
        offset = btnHandler.wheelChange
        current = left.position()
        left.command_position(current + offset)
        time.sleep(0.1)
    btnHandler.loop = True
    print("Grabbing Cube")
    
    # CLOSE GRIPPERS
    left.close()

    right.open()
    time.sleep(2.0)


    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(traj, trajOther)
    #TODO TEST SOME ROTATIONS OF THE CUBE
    time.sleep(2.0)
    rightSideCW(traj, trajOther, left, right)
    time.sleep(2.0)
    
    cubeRotation(traj, trajOther, left, right)
    time.sleep(2.0)
    moveToCubeHandlingPos(traj, trajOther)
    
    btnHandler.loop = True
    while btnHandler.loop and not rospy.is_shutdown():
        time.sleep(1.0)
    btnHandler.loop = True
    
    left.open()
    right.open()

def cubeGrab(cubeHolder, cubeGrabber):
    time.sleep(3.0)
    current = [0.6, -0.1, 0.35]
    new = [0.6, -0.05, 0.35]
    neededMotionY = None

    holdingGripper = cubeHolder.gripper
    grabbingGripper = cubeGrabber.gripper

    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(cubeGrabber, cubeHolder)

    ic = image_capturer(cubeGrabber.limb)
    time.sleep(2.0)

    poseOrientation = "+y+90"


    motionFactors = [-0.00025, -0.00035, -0.0004 ]
    startPos = [0.6, -0.1, 0.35, poseOrientation]

    if cubeGrabber.limb == "left":
        #convert variables for a left handed grabbing
        motionFactors = [-1*i for i in motionFactors]
        loc = translateCoords(startPos[:3])
        poseOrientation = mirrorOrientation(poseOrientation)
        loc.append(orientation)
        startPos = loc
        current = translateCoords(current)
        new = translateCoords(new)




    currentPos = startPos[:3]

    print startPos
    cubeGrabber.add_point( startPos, 2.0 )
    moveLimb( cubeGrabber, poseOrientation, currentPos, [0.6, -0.041, 0.35], 3.0, 6.0 );

    cubeGrabber.start()
    cubeGrabber.wait()
    time.sleep(6.0)
    cubeGrabber.clear()

    '''
    while str(neededMotionY) != "Good" and not rospy.is_shutdown():
        #TODO REMOVE print "entering Loop"
        neededMotionY, neededMotionZ = seeCube(ic)

        if neededMotionY != None:
            if abs(neededMotionY) > 3.0:
                factorIndex = max(int(abs(neededMotionY)/20), len(motionFactors)-1)
                yOffset = motionFactors[factorIndex]*neededMotionY
                zOffset = motionFactors[factorIndex]*neededMotionZ
                neededOffset = [0,yOffset,zOffset]
                #adds cooresponding elements
                newPos = map(sum, zip(currentPos, neededOffset ))
                moveLimb(cubeGrabber, poseOrientation, currentPos, newPos, it = 0.20, ft = 0.60, n = 30 )
                cubeGrabber.start()
                cubeGrabber.wait()
                time.sleep(0.65)
                cubeGrabber.clear()
                currentPos = newPos
            elif abs(neededMotionY) <= 3.0 and abs(neededMotionZ) < 6.0:
                neededMotionY = "Good"
        time.sleep(0.1)
    '''
    #return
    # CLOSE GRIPPERS
    grabbingGripper.close()
    time.sleep(1.0)
    holdingGripper.open()

    #Move arm back to handling position
    cubeGrabber.add_point(startPos, 2.0)
    cubeGrabber.start()
    cubeGrabber.wait()
    cubeGrabber.clear()
    

    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(cubeGrabber, cubeHolder)

def seeCube(ic, layers=1):
    neededY = None
    neededZ = 0

    yAccuracy = 15.0

    zStickerPlacement = 115.25
    stickerWidthWanted = 88.5
    if layers == 2:
        #needs more adjustment maybe tweak which square it grabs
        stickerWidthWanted = 70.0
    
    #Retrieve the image from baxters hand
    img = ic.getImage()

    if img == None:
        print "Image Capture Failure"
        sys.exit()

    # BELOW SECTION FINDS CUBE
    edges = cv2.Canny(img, 40,100)

    ret, thresh = cv2.threshold(edges, 127,255,0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    validCont = reduceToStickers(contours, [1000, 50000])
    if len(validCont) > 0:

        cv2.drawContours(img, validCont, -1, (0,0,255), 2)

        pt1 = validCont[0][0]
        pt2 = validCont[0][1]

        stickerWidthFound = dist(pt1, pt2)
        neededY = stickerWidthFound - stickerWidthWanted


        if abs(neededY) < yAccuracy:
            centers = [getCenter(i) for i in validCont]
            zCenter = sum([ centers[i][1] for i in range(len(centers))])/len(centers)
            
            #3 is just a scaling factor
            #don't remove unless you want the robot to hit itself
            # =)
            neededZ = (zStickerPlacement - zCenter)/3.0
        print neededY, neededZ


    cv2.drawContours(img, validCont, -1, (0,0,255), 2)
    showImage(img, "/myImage")
    showImage(cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), "/myEdges")
    showImage(img, '/robot/xdisplay')
    return neededY, neededZ

def showImage(img, topicName):
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher(topicName, Image, latch=True, queue_size=10)
    pub.publish(msg)

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
        print("INVALID POSE - No Valid Joint Solution Found For: %s, on %s arm"%(str(list((X,Y,Z,direction))), limb))
        
        
    jointPos = [limb_joints[i] for i in [limb+'_s0', limb+'_s1', limb+'_e0', limb+'_e1', limb+'_w0', limb+'_w1', limb+'_w2']]
    return jointPos

def readInCubeState(traj, trajOther):
    leftPos1 = [0.57, 0.05, 0.41, '-z']


    rightPos1 = [0.6, -0.05, 0.35, '+y']
    rightPos2 = [0.6, -0.15, 0.35, '+y']

    t = 6.0

    traj.add_point(leftPos1, t)
    trajOther.add_point(rightPos2, t/2)
    trajOther.add_point(rightPos1, t)

    traj.start()
    trajOther.start()
    traj.wait()
    trajOther.wait()
    traj.clear()
    trajOther.clear()
    time.sleep(2.0)
    ic = image_capturer('right')
    ic.writeImage = True
    time.sleep(1.0)


    traj.add_point(leftPos1, 1.0)
    trajOther.add_point(rightPos2, 1.0)
    rotateWrist(traj, 180.0, 2.0)
    trajOther.add_point(rightPos1, 3.0)
    traj.start()
    trajOther.start()
    traj.wait()
    trajOther.wait()
    traj.clear()
    trajOther.clear()

    time.sleep(2.0)
    ic.writeImage = True
    time.sleep(1.0)


    trajOther.add_point(rightPos2, t/2)
    trajOther.start()
    trajOther.wait()
    trajOther.clear()

def holdOutCube( traj, trajOther ):
    RightArmOut = [0.0218592262024, 0.179859247174, 1.57156331539, 0.971009837622, 0.941480707489, 0.955670029761, -0.0962572943298]

    start = [0.6, -0.1, 0.35, '+y']
    nextPos = [0.6, -0.25, 0.35, '+y']

    moveLimb(trajOther, "+y", start[:3], nextPos[:3])

    trajOther.add_point(RightArmOut, 8.0)
    trajOther.start()
    trajOther.wait()
    trajOther.clear()

    
def rotateWrist(traj, angle, t = 1.0):
    currentPos = copy.copy(traj._goal.trajectory.points[-1].positions)
    wristPos = currentPos[-1]
    wristAngle = wristPos * 180.0 / np.pi
    newAngle = wristAngle + angle
    if newAngle > 180.0:
        newAngle -= 360.0
    elif newAngle < -180.0:
        newAngle += 360.0
    newPos = newAngle * np.pi / 180.0
    currentPos[-1] = newPos
    traj.add_point(currentPos, traj.t + t)

def translateCoords(coords):
    if len(coords) == 7:
        translation = [-1,1,-1,1,-1,1,-1]
    elif len(coords) == 3:
        translation = [1,-1,1]
    newCoords = [translation[i]*coords[i] for i in range(len(coords))]
    return newCoords

def mirrorOrientation(orientation):
    if "y" in orientation:
        if orientation[0] == '+':
            orientation = orientation.replace('+', '-', 1)
        else:
            orientation = orientation.replace('-', '+', 1)
    return orientation

def moveArmsToStart(traj, trajOther, t=3.0):
    
    startPos = {'left': [0.6,  0.1, 0.35, '-y'],
                'right':[0.6, -0.1, 0.35, '+y+90']
                }

    cubeGrabPos = {
                'left': [0.6,  0.3, 0.35, '-y'],
                'right':[0.6, -0.3, 0.35, '+y+90']
    }
    traj.add_point(startPos[traj.limb], t)
    trajOther.add_point(startPos[trajOther.limb], t)
    t+= 2.0
    traj.add_point(cubeGrabPos[traj.limb], t)
    traj.start()
    trajOther.start()
    traj.wait()
    trajOther.wait()
    traj.clear()
    trajOther.clear()

def moveLimb(traj, orientation, (ix,iy,iz), (fx, fy, fz), it = 1.0, ft = 5.0, n = 10):
    dx = (fx-ix)/n
    dy = (fy-iy)/n
    dz = (fz-iz)/n
    dt = (ft-it)/n
    traj.add_point( (ix, iy, iz, orientation), it)
    for i in range(1,n+1):
        traj.add_point( (ix+i*dx, iy+i*dy, iz+i*dz, orientation), it+i*dt)

def cubeRotation(traj, trajOther, left, right, t=6.0):
    leftPos1 = [0.6, 0.0, 0.35, '+z']
    leftPos2 = [0.6, 0.1, 0.35, '-y']
    leftPos3 = [0.6, 0.05, 0.35, '-y']

    rightPos1 = [0.6, -0.2, 0.425, '+y']
    rightPos2 = [0.6, -0.075, 0.425, '+y']
    rightPos3 = [0.6, -0.2, 0.35, '+y+90']
    rightPos4 = [0.6, -0.1, 0.35, '+y+90']


    #Move arms into first handoff position
    traj.add_point(leftPos1,t)    
    trajOther.add_point(rightPos1, t/2)
    trajOther.add_point(rightPos2, t*2)
    traj.start()
    trajOther.start()
    traj.wait(t)
    trajOther.wait(t*2)
    traj.clear()
    trajOther.clear()
    time.sleep(1.0)

    #First Cube handoff
    right.close()
    time.sleep(1.0)
    left.open()
    time.sleep(1.0)

    #move to inter-grabbing position
    traj.add_point(leftPos1, t/2.0)
    traj.add_point(leftPos2, t)
    trajOther.add_point(rightPos1, t/2.0)
    traj.start()
    trajOther.start()
    traj.wait(t)
    trajOther.wait(t)
    traj.clear()
    trajOther.clear()
    time.sleep(3.0)

    #move to second cube handoff position
    traj.add_point(leftPos3, t/2)
    trajOther.add_point(rightPos3, t/2)
    trajOther.add_point(rightPos4, t)
    traj.start()
    trajOther.start()
    traj.wait()
    trajOther.wait()
    traj.clear()
    trajOther.clear()
    time.sleep(1.0)

    #Second Cube handoff
    left.close()
    time.sleep(1.0)
    right.open()
    time.sleep(1.0)

    trajOther.add_point(rightPos3, t/2)
    trajOther.start()
    trajOther.wait()
    trajOther.clear()

def leftSideCW(leftArm, rightArm, t=6.0):
    R0 = [0.6, -0.06, 0.35, '+y']
    R2 = [0.6, -0.1, 0.35, '+y+90']
    
    LeftPos1 = [0.6, 0.06, 0.35, '+z']
    LeftPos2 = [0.6, 0.1, 0.35, '-y+90']

    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(leftArm, rightArm)

    leftArm.add_point(LeftPos1, t)
    leftArm.start()
    leftArm.wait(t)
    leftArm.clear()
    leftArm.gripper.close()
    
    
    leftArm.add_point(LeftPos1, 1.0)
    rotateWrist(leftArm, 90.0)
    leftArm.start()
    leftArm.wait(2.0)
    leftArm.clear()
    leftArm.gripper.open()
    moveLimb(leftArm, '-y+90', (0.6, 0.06, 0.35), (0.6, 0.1, 0.35))
    
    leftArm.start()
    leftArm.wait()
    leftArm.clear()

def frontSideCW(leftArm, rightArm, t = 5.0):
    #ALMOST SETUP FOR A CUBE ROTATION
    R0 = [0.6, -0.06, 0.35, '+y']
    R2 = [0.6, -0.1, 0.35, '+y+90']

    rightPos1 = [0.6, -0.2, 0.42, '-z']
    rightPos2 = [0.6, -0.05, 0.42, '-z']
    
    LeftPos1 = [0.6, 0.05, 0.35, '-y']
    LeftPos2 = [0.6, 0.1, 0.35, '-y+90']

    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(leftArm, rightArm)

    moveLimb(rightArm, '+y', (0.6, -0.1, 0.35), (0.6, -0.3, 0.55), ft = t)
    rightArm.add_point(rightPos1, t+4.0)
    rightArm.add_point(rightPos2, t+6.0)
    rightArm.start()
    rightArm.wait()
    time.sleep(11.0)
    rightArm.clear()
    rightArm.gripper.close()

    leftArm.add_point(LeftPos1, t)
    leftArm.start()
    leftArm.wait()
    time.sleep(5.0)
    leftArm.clear()
    leftArm.gripper.close()
    
    leftArm.add_point(LeftPos1, 1.0)
    rotateWrist(leftArm, 90.0)
    leftArm.start()
    leftArm.wait(2.0)
    leftArm.clear()
    leftArm.gripper.open()
    moveLimb(leftArm, '-y+90', (0.6, 0.06, 0.35), (0.6, 0.1, 0.35))
    
    leftArm.start()
    leftArm.wait()
    leftArm.clear()

    rightArm.add_point(rightPos2, t+2.0)
    rightArm.add_point(rightPos1, t+4.0)
    rightArm.start()
    rightArm.wait()
    time.sleep( t+4.0 )
    rightArm.clear()
    rightArm.gripper.close()

    moveToCubeHandlingPos(leftArm, rightArm)

def rightSideCW(leftArm, rightArm, t=6.0):
    R0 = [0.6, -0.06, 0.35, '+y']
    R2 = [0.6, -0.1, 0.35, '+y+90']
    
    # MOVE ARMS TO CUBE HANDLING STATE
    moveToCubeHandlingPos(leftArm, rightArm)

    rightArm.add_point(R0, t)
    rightArm.start()
    rightArm.wait()
    time.sleep(1.0)
    rightArm.clear()
    rightArm.gripper.close()
    
    rightArm.add_point( R0, 1.0 )
    rotateWrist( rightArm, 90.0, t = 3.0 )
    rightArm.start()
    rightArm.wait()
    time.sleep(3.0)
    rightArm.clear()

    rightArm.gripper.open()
    moveLimb( rightArm, '+y+90', (0.6, -0.06, 0.35), (0.6, -0.1, 0.35) )
    rightArm.start()
    rightArm.wait()
    time.sleep(2.0)
    rightArm.clear()

def moveToCubeHandlingPos(traj, trajOther, t=5.0):

    cubeHandlePos = {
                'left': [0.6, 0.1, 0.35, '-y'],
                'right':[0.6, -0.1, 0.35, '+y']
                }
    traj.add_point(cubeHandlePos[traj.limb], t)
    trajOther.add_point(cubeHandlePos[trajOther.limb], t)
    traj.start()
    trajOther.start()
    traj.wait()
    trajOther.wait()
    traj.clear() 
    trajOther.clear() 

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

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self.limb = limb
        self.t = 0.0
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
        self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()

        if len(positions) == 4:
            positions = getJointPos(self.limb, positions[0],positions[1],positions[2], positions[3])
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self.t = max(time, self.t)
        self._goal.trajectory.points.append(point)


    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout = 45.0 ):
        # if timeout == None:
        #     timeout = self.t
        #     print self.t
        print "Timeout: %f"%(timeout)
        success = self._client.wait_for_result(timeout=rospy.Duration(timeout))
        if( not success ):
            self._client.cancel_goal()
        else:
            state  = self._client.get_state()
            print state

    def result(self):
        return self._client.get_result()

    def clear(self, limb = None):
        if limb == None:
            limb = self.limb
        self.t = 0.0
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

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

    def reset(self):
        self.loop = True
        self.wheelChange = 0

class image_capturer:
    def __init__(self, limb):
        self.cam_sub = rospy.Subscriber("/cameras/%s_hand_camera/image" % limb, Image, self.callback)
        self.bridge = cv_bridge.CvBridge()
        self.writeImage = False
        self.data = None
        time.sleep(2)
        

    def callback(self, data):
        if self.writeImage != False:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError, e:
                print e
            if self.writeImage == True:
                #Write image to file
                now = rospy.get_rostime()
                #cv2.imwrite("images/Cube_%i.jpg" % now.secs, cv_image)
                self.writeImage = False
            else:
                self.data = cv_image
                self.writeImage = False

    def getImage(self):
        self.writeImage = "data"
        startTime = time.time()
        while(self.data == None and time.time() - startTime < 10.0):
            time.sleep(0.1)

        if self.data == None:
            raise Exception("Time out while waiting for image.")

        img = copy.copy(self.data)
        self.data == None
        return img

if __name__ == "__main__":
    main()
