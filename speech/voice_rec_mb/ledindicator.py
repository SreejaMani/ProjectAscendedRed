#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, String

value = 0;
active = False;

def toggle_colour():
    global value

    if value is 0:
        #Green
        red.publish(0)
        green.publish(100)
        value = 1
    elif value is 1:
        #Lime
        red.publish(75)
        green.publish(75)
        value = 0

    rospy.sleep(0.3)

def colour_indicator(data):

    global active
    if "Say something!" in data.data and not active:
        active = True
        while active:
            toggle_colour()

    #Any debug message that isn't "Say something!" will deselect 'active'
    active = False

    red.publish(0)
    green.publish(100)

def listener():
    rospy.init_node('led_indicator')
    rospy.Subscriber("/red/speech_debug", String, colour_indicator)
    rospy.spin()

if __name__ == '__main__':
    red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
    green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
    listener()







   

