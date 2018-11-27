#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash
from nas_launch.msg import request, video

from std_msgs.msg import Int32, Float32, UInt32


def listener(data):

    global running, current_emotion
    # print abs(nearby_object_pos - data.data)

    run_cmd = DigitalIOState()
    run_cmd.state = 1


    print 'Data:', data.data
    print 'Current emotion:', current_emotion
    if data.data != 0 and running != True:
        running = True
        if data.data in range(1,2):
            # 1 user - Alone... Scared
            # Lime
            headColour(75, 75)
            playEmotion("e")
            current_emotion = "Fear"

        if data.data in range(2,6):
            # 2-3 users - A party! Happy
            # Green
            headColour(0, 100)
            playEmotion("b")
            current_emotion = "Happy"

        if data.data in range (6,30):
            # 4 users - A crowd! Love
            # Orange
            headColour(80, 20)
            playEmotion("c")
            current_emotion = "Love"

        # sleep for random int b/w 35-60 seconds before checking for new emotion
        rospy.sleep(randint(35, 60))
        running = False

    # if abs(nearby_object_pos - data.data) > 30:
    #     happy_cmd_listener.publish(run_cmd)
    #     print 'published...\n'
    # nearby_object_pos = data.data

def headColour(redLevel, greenLevel):
    red.publish(redLevel)
    green.publish(greenLevel)

def playEmotion(emotion):
    movie.publish("2"+emotion)
    movie.publish("3"+emotion)
    movie.publish("4"+emotion)

def main():
    global current_emotion, running
    current_emotion = "Default"
    running = False

    rospy.init_node('emotional_red_trigger')
    print 'Emotional red active...\n'
    rospy.Subscriber("/input/lidar/numobjects",UInt32, listener)

    rospy.spin()


if __name__ == '__main__':
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    movie = rospy.Publisher('/video_request', request, latch=True, queue_size=3)
    main()
