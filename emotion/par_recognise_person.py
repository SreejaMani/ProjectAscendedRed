#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash
from nas_launch.msg import request, video

from std_msgs.msg import Int32, Float32, UInt32


def listener(data):

    global running, emotion_listener, nearby_object_pos, current_emotion_num
    # print abs(nearby_object_pos - data.data)

    run_cmd = DigitalIOState()
    run_cmd.state = 1


    #sleep for random int b/w 10-30 seconds before checking for new user
    #delay = randint(10,30)
    #
    print 'Data:', data.data
    print 'Current emotion number:', current_emotion_num
    if data.data != 0 and current_emotion_num != data.data:
        print 'Loops working?'
        if data.data is 1:
            print 'testing??', data.data, current_emotion_num
            # 1 user - A friend! Happy
            red.publish(0)
            green.publish(100)
            movie.publish("2b")
            movie.publish("3b")
            movie.publish("4b")
        if data.data is 2:
            # 2 users - A party! Love
            red.publish(80)
            green.publish(20)
            movie.publish("2c")
            movie.publish("3c")
            movie.publish("4c")
        if data.data is 4:
            # 4 users - A crowd - Scared
            red.publish(75)
            green.publish(75)
            movie.publish("2e")
            movie.publish("3e")
            movie.publish("4e")

        current_emotion_num = data.data
    # if abs(nearby_object_pos - data.data) > 30:
    #     happy_cmd_listener.publish(run_cmd)
    #     print 'published...\n'
    # nearby_object_pos = data.data


def main():
    global emotion_listener, nearby_object_pos, current_emotion_num
    nearby_object_pos = 0
    current_emotion_num = 0

    rospy.init_node('emotional_red_trigger')
    print 'emotional red active...\n'
    emotion_listener = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=False, queue_size=1)
    rospy.Subscriber("/input/lidar/numobjects",UInt32,listener)

    rospy.spin()


if __name__ == '__main__':
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    movie = rospy.Publisher('/video_request', request, latch=True, queue_size=3)
    main()
