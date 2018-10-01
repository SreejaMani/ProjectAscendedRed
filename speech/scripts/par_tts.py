#!/usr/bin/python

import rospy
from std_msgs.msg import String
import os

#the publishing topic for debugging purpose
pubdebug = rospy.Publisher('/par/speech_debug', String, queue_size=1)
pubtts = rospy.Publisher('/red/text2speech', String, queue_size=1)

#the message sent to the debugging topic
def debug(str):
    print(str)
    pubdebug.publish(str)

#the function which does the audio playing when command is receive from the subscriber /input/speech/commands
def speech(data):
    debug("Listening: "+ str(data.data))
    text = data.data
    
    if "hello" == text or "hi" == text:
        debug("Speaking: hello")
	pubtts.publish("Welcome to the Vxlab")

    elif "what is your name" == text:
	debug("Speaking: " + text)
	pubtts.publish("my name is Baxter but you can call me Red")

    elif "what sort of things can you do" == text:
	debug("Speaking: things")
        pubtts.publish("I can rotate, tuck and untuck arms, smile, mimic and other things")
  
    elif "untuck arms" == text:
	debug("Speaking: " + text)
        pubtts.publish("sure... I can do that")
	os.system("/home/par/par_ws/utuck")

    elif "tuck arms" == text:
	debug("Speaking: " + text)
	pubtts.publish("Tucking arms")
	os.system("/home/par/par_ws/tuck")

    elif "mimic me" ==  text:
        debug("Speaking: Follow me")
        os.system("./play-audio Hello2.wav")

    elif "shake hands" == text:
        debug("Speaking: Hand Shake")
	pubtts.publish("Sure... I'll do that")

    elif "wave" in text:
        debug("Speaking: Hi")
        pubtts.publish("ok... I am waving")

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
        pubtts.publish("Challenge accepted")

    elif "high" in text and "five" in text:
        debug("Speaking: High Five")
        os.system("./play-audio High_Five.wav")

    elif "stop" in text:
        debug("Speaking: See_you_next_time")
        pubtts.publish("okay... See you next time")

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
	pubtts.publish("Oops! Didn't catch that. What did you say?")
        
try:
    rospy.init_node('par_audio_replay', anonymous=True)
    rospy.Subscriber("/input/speech/commands", String, speech)
    debug("starting")
    rospy.spin()
    
except KeyboardInterrupt:
    pass
    

