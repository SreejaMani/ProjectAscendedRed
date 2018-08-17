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
import math
import time
import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Twist

DEADZONE = 20 #deadzone angle from centre of Red
ROTATION_SPEED = math.pi/8 #pi/8 (takes about 16 seconds to do one rotation)

#Callback for angle publisher, grabs angle of "objects" detected by lidar
def angle_callback(data):
	#global start_time
	#this might get handy here
	#elapsed_time = time.time() - start_time
	#print elapsed_time
	angleData = data.data
	try:
		closestAngle = angleData[closestDistID]
		radian = math.radians(closestAngle)
		print data.data
		print "Closest angle: " + str(closestAngle) + "\n"
		#motionTimed(rotate_your_owl(closestAngle), calcTimeAway(closestAngle));
		moveLoop(rotate_your_owl(closestAngle), closestAngle)
	except:
		print "No angle"

#prototype code of discovering closest target
#get obj number
def dist_callback(data):
	global closestDistID 
	ctr = 0
	distData = data.data
	try:
		minDist = min(distData)
		for dist in distData:
			if (dist == minDist):
				closestDistID = ctr
				break
			ctr += 1
	except:
		print "Nothing close by!"

def rotate_your_owl(targetAngle):
	#defines no roation
	global zero_twist
	zero_twist = Twist()
	zero_twist.angular.z = 0
	#defines clockwise rotation
	global clockwise_twist
	clockwise_twist = Twist()
	clockwise_twist.angular.z = ROTATION_SPEED
	#defines anticlockwise rotation
	global anticlockwise_twist
	anticlockwise_twist = Twist()
	anticlockwise_twist.angular.z = -ROTATION_SPEED

	#turns anticlockwise if detects person to Red's right
	if targetAngle < 180 - DEADZONE and targetAngle > 90:
		return anticlockwise_twist
	#turns clockwise if detects person to Red's left
	elif targetAngle > 180 + DEADZONE < 270:
		return clockwise_twist
	#if within deadzone, don't turn
	return zero_twist

#calculate time it takes for Red to turn
def calcTimeAway(targetAngle):
	deltaAngle = targetAngle - 180
	duration = 0
	try:
		duration = degreesToRad(abs(deltaAngle)) / (ROTATION_SPEED / math.pi)
		"""
		print "degrees = " + str(abs(duration))
		print "degreesToRad = " + str(degreesToRad(abs(duration)))
		print "ROTATION_SPEED = " + str(ROTATION_SPEED)
		"""
	except:
		pass
	return duration

#convert degrees to radians
def degreesToRad(degrees):
	return degrees * math.pi / 180

"""
#do a twist motion for certain duration
def motionTimed(twist, duration):
	pub.publish(twist)
	rospy.sleep(duration)
	#time.sleep(duration)
"""

def moveLoop(twist, angle):
	t0 = rospy.Time.now().to_sec()
	t1 = rospy.Time.now().to_sec()
	duration = calcTimeAway(angle)
	
	while(t1 - t0 < duration):
		print "Twistin! ", round((t1-t0), 2),  " out of ", round(duration, 2)
		t1 = rospy.Time.now().to_sec()
		pub.publish(twist)

def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	parser.parse_args(rospy.myargv()[1:])

	# Test the input lidar code before the head wobbler
	print("Initializing node... ")
	rospy.init_node("mobility_face_closest_wlidar", anonymous=True)

	# subscribes to publisher generated from lidar_detect_crowd.py
	rospy.Subscriber("/input/lidar/distances",Float32MultiArray,dist_callback)
	rospy.Subscriber("/input/lidar/angles",Float32MultiArray,angle_callback)

	# publishers to give safebase some movement input
	global pub
	pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)
		#????time.sleep(1)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
