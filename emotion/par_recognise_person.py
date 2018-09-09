#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash

from std_msgs.msg import Int32

def listener(data):

    global running, happy_cmd_listener, nearby_object
    print abs(nearby_object - data.data)

    run_cmd = DigitalIOState()
    run_cmd.state = 1 
    #sleep for random int b/w 10-30 seconds before checking for new user
    #delay = randint(10,30)

    print 'Data:', data
    if abs(nearby_object - data.data) > 30:
        # while running:
        happy_cmd_listener.publish(run_cmd)
        print 'published...\n'
    nearby_object = data.data


def main():
    global happy_cmd_listener, nearby_object
    nearby_object = 0
    print 'emotional red active...\n'
    happy_cmd_listener = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=False, queue_size=1)
    rospy.Subscriber("/par/closest_lidar/",Int32,listener)
    rospy.init_node('happy_baxter_trigger')
    rospy.spin()


if __name__ == '__main__':
    main()

    #listener()
