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
import rospy
import time

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

DEADZONE = 15 #deadzone angle to left and right of centre
ROTATION_SPEED = math.pi/12 #takes about 16 seconds to do one rotation
DELAY_ANGLE = 8
DELAY_TIME = 1.5
lastAngle = 0

stallTime = time.time()

#psudo-thread to continuously publish to safebase
def timer_callback(event):
	pub.publish(twist_output)

#callback for lidar detect group angle information
def callback(data):
	global twist_output
	twist_output = rotate_your_owl(int(data.data))

#returns twist info depending on angle
def rotate_your_owl(angle):
	#use the global lastAngle
	global lastAngle
	global stallTime
	
	currTime = time.time()
	deltaTime = currTime - stallTime
	print deltaTime

	#defines no roation
	zero_twist = Twist()
	zero_twist.angular.z = 0.0
	#defines clockwise rotation
	clockwise_twist = Twist()
	clockwise_twist.angular.z = ROTATION_SPEED
	#defines anticlockwise rotation
	anticlockwise_twist = Twist()
	anticlockwise_twist.angular.z = -ROTATION_SPEED

	"""
	#calculate angle difference of last scan
	deltaAngle = abs(lastAngle - angle)
	print deltaAngle
	#if difference is greater than DELAY_ANGLE
	if (deltaAngle > DELAY_ANGLE):
		#don't turn
		lastAngle = angle
		return zero_twist
	#update lastAngle global
	lastAngle = angle
	"""

	#if detects person to Red's right
	if angle < 180 - DEADZONE:
		if deltaTime > DELAY_TIME:
			print "I like to move it move it"
			return anticlockwise_twist
		else:
			return zero_twist
	#if detects person to Red's left
	elif angle > 180 + DEADZONE:
		if deltaTime > DELAY_TIME:
			print "I like to move it move it"
			return clockwise_twist
		else:
			return zero_twist
	#else (if within deadzone)
	stallTime = time.time()
	return zero_twist

#main thread
def main():
	#init
	global twist_output
	twist_output = Twist()

	#update publisher
	global pub
	pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)

	# subscribes to publisher generated from lidar_detect_crowd.py

	print("Initializing node... ")
	rospy.init_node("face_closest_person", anonymous=True)
	rospy.Subscriber("/par/closest_lidar/",Int32,callback)

	rospy.Timer(rospy.Duration(0.1), timer_callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
