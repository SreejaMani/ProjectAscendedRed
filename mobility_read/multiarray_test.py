#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

def dist_callback(data):
	global closestDistID 
	ctr = 0
	distData = data.data
	minDist = min(distData)
	for dist in distData:
		if (dist == minDist):
			closestDistID = ctr
			print data.data
			print "I found one! ctr = " + str(ctr) + "\n"
			break
		ctr += 1

def angle_callback(data):
	angleData = data.data
	try:
		closestAngle = angleData[closestDistID]
		print data.data
		print "Does this make sense? " + str(closestAngle) + "\n"
	except:
		print "Error!"

def main():
	print "\nTesting main!"
	rospy.init_node("par_multiarray_test", anonymous=True)
	rospy.Subscriber("/input/lidar/distances",Float32MultiArray,dist_callback)
	rospy.Subscriber("/input/lidar/angles",Float32MultiArray,angle_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
