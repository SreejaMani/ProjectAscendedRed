#!/usr/bin/env python

#Do I add this in? I think I have to for copyright...

# Copyright (c) 2013-2015, Rethink Robotics
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

import argparse
import random
import math
import time
import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Twist

DEADZONE = 20 #deadzone angle from centre of Red
ROTATION_SPEED = math.pi/8 #takes about 16 seconds to do one rotation

#psudo-thread to continuously publish to safebase
def timer_callback(event):
	pub.publish(twist_output)

#callback for lidar detect group angle information
def angle_callback(data):
	global twist_output
	angleData = data.data #obtain data
	try:
		#if callback isn't empty
		if len(angleData) != 0:
			#obtain  angle of closest object
			closestAngle = angleData[closestDistID]
			#update twist output
			twist_output = rotate_your_owl(closestAngle) 
			#radian = math.radians(closestAngle);
	except:
		pass

#callback for lidar detect group distance information
def dist_callback(data):
	global closestDistID 
	ctr = 0
	distData = data.data #obtain data
	try:
		#if callback isn't empty
		if len(distData) != 0:
			minDist = min(distData)
			#obtain ID of closest object
			for dist in distData:
				if (dist == minDist):
					closestDistID = ctr
					break
				ctr += 1
	except:
		pass

#returns twist info depending on angle
def rotate_your_owl(angle):
	#defines no roation
	zero_twist = Twist()
	zero_twist.angular.z = 0.0
	#defines clockwise rotation
	clockwise_twist = Twist()
	clockwise_twist.angular.z = ROTATION_SPEED
	#defines anticlockwise rotation
	anticlockwise_twist = Twist()
	anticlockwise_twist.angular.z = -ROTATION_SPEED

	#if detects person to Red's right
	if angle < 180 - DEADZONE and angle > 90:
		return anticlockwise_twist
	#if detects person to Red's left
	elif angle > 180 + DEADZONE < 270:
		return clockwise_twist
	#else (if within deadzone)
	return zero_twist

#main thread
def main():
	#init
	#update publisher
	global pub
	pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)

	# subscribes to publisher generated from lidar_detect_crowd.py
	rospy.Subscriber("/input/lidar/distances",Float32MultiArray,dist_callback)
	rospy.Subscriber("/input/lidar/angles",Float32MultiArray,angle_callback)

	print("Initializing node... ")
	rospy.init_node("face_closest_person", anonymous=True)

	global twist_output
	twist_output = Twist()

	rospy.Timer(rospy.Duration(0.1), timer_callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
