#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash

from nas_launch.msg import video, request

from std_msgs.msg import UInt32
from std_msgs.msg import Float32

def listener(data):

    global running, emotion_listener, nearby_object_pos, num_objects

    # print abs(nearby_object_pos - data.data)

    run_cmd = DigitalIOState()
    run_cmd.state = 1 
    #sleep for random int b/w 10-30 seconds before checking for new user
    delay = randint(3,10)
    #
    print 'Data:', data
    if data.data != num_objects:
        if data.data is 0:
            # Neutral blank state / reset colour
            red.publish(0)
            green.publish(100)
        elif data.data is 1 or data.data is 2:
            # 1 users - A friend! Happy
            red.publish(0)
            green.publish(100)
            movie.publish("2b")
            movie.publish("3b")
            movie.publish("4b")
        elif data.data is 3:
            # 2/3 users - A party! Love
            red.publish(80)
            green.publish(20)
            movie.publish("2c")
            movie.publish("3c")
            movie.publish("4c")
        elif data.data is 4:
            # 3/4 users - Too many! a n g e r
            red.publish(100)
            green.publish(0)
            movie.publish("2a")
            movie.publish("3a")
            movie.publish("4a")

    num_objects = data.data
    # if abs(nearby_object_pos - data.data) > 30:
    #     happy_cmd_listener.publish(run_cmd)
    #     print 'published...\n'
    # nearby_object_pos = data.data

def main():

    global emotion_listener, nearby_object_pos, num_objects
    nearby_object_pos = 0
    num_objects = 0
    print 'Emotions active...\n'

    emotion_listener = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=False, queue_size=1)
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    movie = rospy.Publisher('/video_request', request, latch=False, queue_size=3)
    rospy.Subscriber("/input/lidar/numobjects",UInt32,listener)
    rospy.init_node('happy_baxter_trigger')
    rospy.spin()


if __name__ == '__main__':

    main()
