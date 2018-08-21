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

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

publishCounter = 0
publishRate = 20

#callback for lidar raw info
# data.ranges and data.intensities
# index is from 0..3599, where
# - 0 is rear
# - 900 is (base's) right
# - 1800 is front
# - 2700 is (base's) left
def callback(data):
	global publishCounter
	if publishCounter == publishRate:
		print scanFront(data, 950, 2650)
		publishCounter = 0
	publishCounter += 1

def scanFront(data, min, max):
	minDist = 500
	minAngle = -1
	i = min
	while i < max:
		# exclude the disruptions caused by the 4 bars around the lidar
		# the disruptions that would disturb angles 950-2650 are at 1340-1370 and 2240-2270
		if (i in range(1340, 1370)) or (i in range(2240, 2270)):
			i += 1
			continue
		#print "at i = " + str(i) + "\t" + str(data.ranges[i])
		if (data.ranges[i] < minDist):
			minDist = data.ranges[i]
			minAngle = i
		i += 1
	return minAngle 

#main thread
def main():
	print("Initializing node... ")
	rospy.init_node("par_raw_lidar_test", anonymous=True)
	rospy.Subscriber('/laser_birdcage_r2000/scan',LaserScan, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
