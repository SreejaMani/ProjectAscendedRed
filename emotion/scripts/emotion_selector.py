#! /usr/bin/env python

import rospy,time
from std_msgs.msg import Float32, UInt32
from baxter_core_msgs.msg import DigitalIOState
from nas_launch.msg import request, video

value = 0;
play_pressed = False;

def colour_selector(data):
    global value
    value = data.data%5

    if value is 0:
        #Green
        red.publish(0)
        green.publish(100)
    elif value is 1:
        #Lime
        red.publish(75)
        green.publish(75)
    elif value is 2:
        #Orange
        red.publish(80)
        green.publish(20)
    elif value is 3:
        #Dim green
        red.publish(10)
        green.publish(10)
    elif value is 4:
        #Red
        red.publish(100)
        green.publish(0)


def display_pattern(data):
    global play_pressed
    if data.state and not play_pressed:
        if value is 0:
            #Green
            # movie.publish("Happy")
            movie.publish("2b")
            movie.publish("3b")
            movie.publish("4b")

        elif value is 1:
            #Lime
            # movie.publish("Fear")
            movie.publish("2e")
            movie.publish("3e")
            movie.publish("4e")
        elif value is 2:
            #Orange
            # movie.publish("Love")
            movie.publish("2c")
            movie.publish("3c")
            movie.publish("4c")
        elif value is 3:
            #Dim green
            # movie.publish("Sad")
            movie.publish("2d")
            movie.publish("3d")
            movie.publish("4d")
        elif value is 4:
            #Red
            # movie.publish("Angry")
            movie.publish("2a")
            movie.publish("3a")
            movie.publish("4a")

    play_pressed = data.state

def listener():
    rospy.init_node('emotion_selector')
    rospy.Subscriber("/robot/analog_io/torso_right_wheel/value_uint32", UInt32, colour_selector)

    rospy.sleep(0.3)
    rospy.Subscriber("/robot/digital_io/torso_right_button_ok/state", DigitalIOState, display_pattern)
    rospy.Subscriber("/robot/digital_io/torso_left_button_ok/state", DigitalIOState, display_pattern)

    rospy.spin()

if __name__ == '__main__':
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    movie = rospy.Publisher('/video_request', request, latch=False, queue_size=3)
    listener()







   

