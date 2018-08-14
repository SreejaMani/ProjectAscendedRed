#!/usr/bin/python
import rospy

from std_msgs.msg import String

pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=5)

def debug(str):
  print(str)
  pubdebug.publish(str)

import urllib2

def tuck_arms():
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/gandalf.tuck_disable")

def untuck_arms():
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/gandalf.enable_untuck")

def dispatch(data):
	command = data.data
	if command == "tuck_arms":
		tuck_arms()
	if command == "untuck_arms":
		untuck_arms()

try:
    rospy.init_node('red_dispatcher_test', anonymous=False)
    untuck_arms()

except KeyboardInterrupt:
    pass
