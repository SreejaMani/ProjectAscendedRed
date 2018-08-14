#!/usr/bin/python

import rospy
from std_msgs.msg import String
import os

#the publishing topic for debugging purpose
pubdebug = rospy.Publisher('/guesta/speech_debug', String, queue_size=1)

#the message sent to the debugging topic
def debug(str):
    print(str)
    pubdebug.publish(str)

#the function which does the audio playing when command is receive from the subscriber /input/speech/commands
def speech(data):
    debug("Speaking: "+str(data.data))
    text = data.data
    
    if "big" in text and "wave" in text:
        debug("Speaking: Hello")
        os.system("./play-audio Hello2.wav")

    elif "hand" in text and "shake" in text:
        debug("Speaking: Hand Shake")
	os.system("./play-audio Hi.wav")

    elif "hello" in text:
        debug("Speaking: Welcome")
	os.system("./play-audio Welcome2.wav")

    elif "wave" in text:
        debug("Speaking: Hi")
        os.system("./play-audio Hello3.wav")

    elif "dab" in text:
        debug("Speaking: Dab")
        os.system("./play-audio Do_you_really_want_a_dab.wav")

    elif "fist" in text and "bump" in text:
        debug("Speaking: fist Bump")
	os.system("./play-audio Okay.wav")

    elif "understand" in text:
        debug("Speaking: Understand")
        os.system("./play-audio Understood.wav")

    elif "come" in text or "here" in text:
        debug("Speaking: Challenged Accpeted")
        os.system("./play-audio Challenge_accepted.wav")

    elif "high" in text and "five" in text:
        debug("Speaking: High Five")
        os.system("./play-audio High_Five.wav")

    elif "stop" in text:
        debug("Speaking: See_you_next_time")
        os.system("./play-audio See_you_next_time.wav")

    elif "understand" in text:
        debug("Speaking: Understood")
        os.system("./play-audio Understood.wav")

    elif "waiting" in text:
        debug("Speaking: Dont understand")
        os.system("./play-audio Sorry_can_you_repeat_that.wav")
    
    elif "neutral" in text:
        debug("Speaking: Going neutral position")
        os.system("./play-audio Neutral.wav")

    else: 
        debug("Speaking: Red not in keyword")
        os.system("./play-audio Huh.wav")
        
try:
    rospy.init_node('guesta_audio_replay', anonymous=False)
    rospy.Subscriber("/input/speech/commands", String, speech)
    debug("starting")
    rospy.spin()
    
except KeyboardInterrupt:
    pass
    

