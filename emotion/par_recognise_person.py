#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash

from std_msgs.msg import UInt8

def listener(data):

    global running, emotion_listener, nearby_object_pos
    # print abs(nearby_object_pos - data.data)

    run_cmd = DigitalIOState()
    run_cmd.state = 1 
    #sleep for random int b/w 10-30 seconds before checking for new user
    #delay = randint(10,30)
    #
    print 'Data:', data
    if data.data is 1
        # 1 user - A friend! Happy
        red.publish(0)
        green.publish(100)
        movie.publish("2b")
        movie.publish("3b")
    if data.data is 2
        # 2 users - A party! Love
        red.publish(80)
        green.publish(20)
        movie.publish("2d")
        movie.publish("3d")
    # if abs(nearby_object_pos - data.data) > 30:
    #     happy_cmd_listener.publish(run_cmd)
    #     print 'published...\n'
    # nearby_object_pos = data.data



def main():
    global emotion_listener, nearby_object_pos
    nearby_object_pos = 0
    print 'emotional red active...\n'
    emotion_listener = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=False, queue_size=1)
    rospy.Subscriber("/input/lidar/numobjects",UInt8,listener)

    rospy.init_node('happy_baxter_trigger')
    rospy.spin()


if __name__ == '__main__':
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    movie = rospy.Publisher('/video_request', request, latch=False, queue_size=3)
    main()
