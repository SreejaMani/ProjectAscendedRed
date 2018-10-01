#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import tf2_ros
from tf import TransformListener

def main():
        rospy.init_node('do_you_even_lift_red', anonymous=True)

        tf_Buffer = tf2_ros.Buffer()
        tf_Listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
                try:
                        buffUserLeft = tf_Buffer.lookup_transform('cob_body_tracker/user_1/left_shoulder', 'cob_body_tracker/user_1/left_hand', rospy.Time())
                        x = buffUserLeft.transform.translation.x
                        y = buffUserLeft.transform.translation.y
                        z = buffUserLeft.transform.translation.z

			tranRightX = -z * 1.0 + 0.2
			tranRightY =  x * 1.1 - 0.28
			tranRightZ =  y * 0.0 + 0.40 #locked for now

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue

                #print buffUserLeft
		print "test x ", tranRightX
		print "test y ", tranRightY
		print "test z ", tranRightZ
		print ""
                rate.sleep()

if __name__ == '__main__':
        main()
