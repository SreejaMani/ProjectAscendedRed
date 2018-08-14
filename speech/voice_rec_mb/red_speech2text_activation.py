#!/usr/bin/python
import rospy
from std_msgs.msg import String
from baxter_core_msgs.msg import DigitalIOState

pub = rospy.Publisher('/red/commands', String, queue_size=5)
pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=5)

def debug(v):
  print(v)
  pubdebug.publish(v)

prevstate = 0;
waiting = 0;

def dobutton(data):
	global prevstate
	global waiting
	if data.state is not prevstate and not waiting:
		if data.state:
			debug('sending restart_speech command')
			pub.publish('restart_speech')
		# timeout before checking button state again(?!)
		waiting = 500
        if waiting:
		waiting = waiting - 1
	prevstate = data.state

try:
    rospy.init_node('red_speech_activation', anonymous=False)
    rospy.Subscriber("/robot/digital_io/torso_right_button_ok/state", DigitalIOState, dobutton)
    rospy.Subscriber("/robot/digital_io/torso_left_button_ok/state", DigitalIOState, dobutton)
    debug('red_speech_activation started')
    rospy.spin()
except KeyboardInterrupt:
    pass
