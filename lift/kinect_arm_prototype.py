#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import tf2_ros
from tf import TransformListener

def main():
        rospy.init_node('do_you_even_lift_red', anonymous=True)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
                try:
                        trans = tfBuffer.lookup_transform('camera_link', 'cob_body_tracker/user_1/torso', rospy.Time())
                        buffTFLSE = tfBuffer.lookup_transform('cob_body_tracker/user_1/left_shoulder', 'cob_body_tracker/user_1/left_elbow', rospy.Time())
                        y2 = buffTFLSE.transform.translation.y
                        x2 = buffTFLSE.transform.translation.x
                        pitch = math.atan2(y2, x2)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue
                #print buffTFLSE
                #print y2, x2
                print math.degrees(-(pitch - 180))
                rate.sleep()

if __name__ == '__main__':
        main()
