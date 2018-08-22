#! /usr/bin/env python

import rospy, time
from random import randint
from baxter_core_msgs.msg import DigitalIOState
from unite_conference.msg import clash

def listener():
    rospy.init_node('angry_baxter_trigger')
    print 'emotion selector active...\n'    
    global running, angry_cmd_listener

    running = False

    rospy.Subscriber("/robot/unity_command_start", clash, start_listening)
    rospy.Subscriber("/robot/unity_command_end", clash, stop_listening)

    run_cmd = DigitalIOState()
    run_cmd.state = 1 

    delay = randint(10,30)

    print running, '\n'

    while True:
        while running:
	    angry_cmd_listener.publish(run_cmd)
	    print 'published...\n'
	    rospy.sleep(delay)

    rospy.spin()

def start_listening(state):
    global running
    running = True
    print 'started listening...\n'

def stop_listening(state):
    global running
    running = False
    print 'stopped listening...\n'

if __name__ == '__main__':
    global angry_cmd_listener

    angry_cmd_listener = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=False, queue_size=1)
    listener()
