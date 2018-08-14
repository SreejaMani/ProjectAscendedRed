#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def distanceCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " Distance, %s", data.data)
def angleCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " Angle, %s", data.data)

def main():
    rospy.init_node('par_follow_rotate', anonymous=True)

    # subscribes to publisher generated from lidar_detect_crowd.py
    rospy.Subscriber("/input/lidar/distances", Float32MultiArray, distanceCallback)
    rospy.Subscriber("/input/lidar/angles", Float32MultiArray, angleCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
