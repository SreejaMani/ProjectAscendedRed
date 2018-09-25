#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import tf2_ros
from tf import TransformListener

def main():
	#init node
        rospy.init_node('do_you_even_lift_red', anonymous=True)

	#init tf buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

	#define refresh rate
        rate = rospy.Rate(10.0)
	
	#main loop
        while not rospy.is_shutdown():
                try:
			#defines the pose of the child from the parent
                        buffUserLeft = tf_buffer.lookup_transform('cob_body_tracker/user_2/left_shoulder', 'cob_body_tracker/user_2/left_hand', rospy.Time())
			#get translations
                        x = buffUserLeft.transform.translation.x
                        y = buffUserLeft.transform.translation.y
                        z = buffUserLeft.transform.translation.z

			#translate the translations
			tranRightX = -z * 1.0 + 0.2
			tranRightY =  x * 1.1 - 0.28
			tranRightZ =  y * 0.0 + 0.40 #locked for now

		#catch and continue after refresh rate
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue

                #print buffUserLeft
		print "test x ", tranRightX
		print "test y ", tranRightY
		print "test z ", tranRightZ
		print ""

		#Refresh rate
                rate.sleep()

if __name__ == '__main__':
        main()
