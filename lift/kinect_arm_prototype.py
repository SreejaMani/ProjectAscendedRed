#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import tf2_ros
import sys
from tf import TransformListener

def systemArgHandler():
	if len(sys.argv) == 2:
		tempID = sys.argv[1]
		if tempID.isdigit():
			print '\033[92m' + "User ID is set to " + tempID
			return tempID
		else:
			print '\033[93m' + "User ID must be an integer"
			print '\033[93m' + "User ID is set to default (1)"
			return 1
	else:
		print "User ID is set to default (1)"
		return 1

def main():
	#init node
        rospy.init_node('do_you_even_lift_red', anonymous=True)

	#init tf buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

	#define refresh rate
        rate = rospy.Rate(10.0)

	#define userID
	userID = systemArgHandler()
	
	#main loop
        while not rospy.is_shutdown():
                try:
			#defines the pose of the child from the parent
                        buffUserLeft = tf_buffer.lookup_transform('cob_body_tracker/user_'+str(userID)+'/left_elbow', 'cob_body_tracker/user_'+str(userID)+'/left_hand', rospy.Time())
			#get translations
                        x = buffUserLeft.transform.translation.x
                        y = buffUserLeft.transform.translation.y
                        z = buffUserLeft.transform.translation.z

		#catch and continue after refresh rate
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue

                #print buffUserLeft
		#print math.degrees((math.atan2(y,x) + math.pi/2) % (math.pi * 2))
		print math.degrees(math.atan2(x,z))
		print ""

		#Refresh rate
                rate.sleep()

if __name__ == '__main__':
        main()
