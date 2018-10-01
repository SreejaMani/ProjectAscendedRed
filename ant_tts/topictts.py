#!/usr/bin/python
# By Ian, Anthony

import rospy
import os
from std_msgs.msg import String
from gtts import gTTS

pubdebug = rospy.Publisher('/par/tts_debug', String, queue_size=5)

def playWav(speak):
 	#cmd = "rosrun voice_recognition tts.sh " + speak
	#print cmd
        #os.system(cmd)
	tts = gTTS(text= speak, lang= 'en')
	tts.save("/tmp/temp.mp3")
	os.system("mpg321 /tmp/temp.mp3")
	os.system("rm /tmp/tempt.mp3")

def debug(str):
	global pubdebug
  	print(str)
  	pubdebug.publish(str)

def dospeech(data):
	print 'dospeech',data.data
  	playWav(str(data.data))
  	debug(data)

try:
    	print "started"
    	rospy.init_node('par_text2speech', anonymous=False)
    	rospy.Subscriber('/red/text2speech', String, dospeech)
    	rospy.spin()
except KeyboardInterrupt:
    pass
