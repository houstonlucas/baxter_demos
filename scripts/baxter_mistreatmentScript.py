#!/usr/bin/env python

"""
Baxter Mistreatment
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

import time
import socket

SAD = "images/sad.PNG"
HAPPY = "images/happy.PNG"
NEUTRAL = "images/neutral.PNG"

# Quit flag
quit = False

# Dictionary of items
items = {1 : "a shaving mirror", 2 : "a five gallon can of", 3 : "a case of army rations", 4 : "an atlas of the pacific", 5 : "a floating seat cushion", 6 : "a small transistor radio", 7 : "the shark repellent", 8 : "fifteen feet of nylon", 9 : "two boxes of chocolate bars", 10 : "a fishing kit"}

def main():
    """
    Baxter Mistreatment
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=False, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if limb == None: limb = 'left'

    print("Initializing node... ")
    rospy.init_node("baxter_wave_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    otherLimb = ""
    if limb == 'left':
        otherLimb = 'right'
    else:
        otherLimb = 'left'
    
    traj = Trajectory(limb)
    trajOther = Trajectory(otherLimb)
    rospy.on_shutdown(traj.stop)
    rospy.on_shutdown(trajOther.stop)

    send_image(NEUTRAL)
    moveArmsToStart(traj, trajOther, limb, otherLimb)


    # set up socket for TCP connection to get external IP
    s = socket.socket ( socket.AF_INET, socket.SOCK_STREAM ) # TCP

    # get external IP
    s.bind ( ( '', 0 ) )
    s.connect ( ( 'google.com', 80 ) )
    ThisIP = s.getsockname()[0]
    s.close()

    # Create the welcome port
    welcome = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    welcome.bind((ThisIP, 0))
    welcome.listen(5)

    # Get the port number
    ThisPort = int ( welcome.getsockname()[1] )

    size = 1024

    socket.setdefaulttimeout(180)

    # Show the IP and port
    print "IP: " + ThisIP
    print "Port: " + str(ThisPort)

    # Accept the connection
    oz, addr = welcome.accept()

    # Loop forever
    while not quit:
        send_image(NEUTRAL)
        attempt = 0
        while attempt < 10 and attempt != -1:
            # Get the command
            try:
                data = oz.recv(size)
                attempt = -1
            except:
                attempt +=1
                print("what")
                time.sleep(3)
        if attempt >= 10:
            print("Connection Dropped\nCLOSING...")
            oz.close()
            exit()
        print data
        # Get the command and content
        divide = data.split()
        command = divide[0]
        content = divide[1]

        # Excecute based on the command

        if command == "wave":
            send_image(HAPPY)
            wave(traj, 'left')
            print("test")
        
        # Start message
        if command == "start":
            tts("Hello you have two minutes")

        # Item prompt
        elif command == "prompt":
            # Give the prompt phrase
            tts("What is your choice for")
            tts(content)

        # Item confimation
        elif command == "confirm":
            # Ask for confrimation
            tts("Did you choose")
            tts(items[int(content)])

        # Happy face
        elif command == "happy":
            # Show the happy face
            send_image(HAPPY)
            tts("Yipeeeee")
            time.sleep(1)

        # Sad face
        elif command == "sad":
            # Show the sad face
            send_image(SAD)

            # Make phrase based on the item number
            if content == '3':
                tts("I am sorry I am still")

            else:
                cry(traj, 'left')

            time.sleep(1)

        # Failure
        elif command == "fail":
            tts("I am sorry I do not")

        # Good bye
        elif command == "bye":
            tts("That is all five items")

        oz.send("continue")
    oz.close()
    

# Speaks the given string
def tts(text):
   os.system("aplay 'Sounds/%s.wav'"%text)

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





def cry(traj, limb, t=3.0):

    dt = 1.0
    #TEAR WIPE POSITIONS
    tw1 = {
	    'left': [-0.322519460284, 0.133456328394, -1.97768472852, 2.19320902897, -0.39269908125, 0.316000041943, 3.04917030764],
		'right': [0.444470932782, 0.19021361748, 2.03214104643, 2.13415076871, -0.131922347607, 0.373524321423, 0.590582602661]
	}

    tw2 = {
		'left': [0.255407800891, 0.717136017517, -2.00836434424, 2.46472362812, -0.22012624281, 0.91041759657, 3.04955380283],
		'right': [0.102776712671, 0.7190534935, 2.63346151459, 2.19435951456, -0.164519439313, 1.02842736079, 1.01204382365]
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
    traj.add_point(p1,t)
    t+=dt
    traj.add_point(p2, t)
    traj.start()
    traj.wait(t/2)
    tts("I am sorry I know that")
    traj.wait(t/2)
    traj.clear(limb)
    moveArmToStart(traj, limb, t = 3.0)

def moveArmsToStart(traj, trajOther, limb, otherLimb, t=5.0):
    startPos = {
		'left': [0.657694262054, 0.216674786041, -1.60224293112, 1.90980607874, 1.13706325772, 1.3161555145, 2.82866056963],
		'right': [-0.325587421857, 0.126553414856, 1.45613126124, 1.86186917917, -1.0845244158, 1.18576714768, -0.261543724036]
		}
    traj.add_point(startPos[limb], t)
    trajOther.add_point(startPos[otherLimb], t)
    traj.start()
    trajOther.start()
    traj.wait(t)
    trajOther.wait(t)
    traj.clear(limb)
    trajOther.clear(otherLimb)

def moveArmToStart(traj, limb, t=5.0):
    startPos = {
	'left': [0.657694262054, 0.216674786041, -1.60224293112, 1.90980607874, 1.13706325772, 1.3161555145, 2.82866056963],
	'right': [-0.325587421857, 0.126553414856, 1.45613126124, 1.86186917917, -1.0845244158, 1.18576714768, -0.261543724036]
	}
    traj.add_point(startPos[limb], t)
    traj.start()
    traj.wait(t)
    traj.clear(limb)
    

def wave(traj, limb, t=3.0):
    #WAVE POSITIONS
    newWave1 = [0.217825271631, 1.05307780968, -1.98995657481, 2.30902457833, 3.06719458187, -0.450990351123, 3.04878681244]
    newWave2 = [0.416475783435, 1.05307780968, -2.32704885256, 2.17671873552, 2.31055855911, -0.388864129285, 3.04840331724]

    if limb != "left":
        newWave1 = translateCoords(newWave1)
        newWave2 = translateCoords(newWave2)
   
    dt = 0.75
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt
    traj.add_point(newWave1, t)
    t+= dt
    traj.add_point(newWave2, t)
    t+=dt + .5
   
    traj.start()
    traj.wait(t/2)
    tts("Hello There Friends")
    traj.wait(t/2)    
    traj.clear(limb)
    moveArmToStart(traj, limb, 2.5)
    

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
