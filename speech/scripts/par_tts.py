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
	textphrase = data.data
	textList = textphrase.split()
	wordFound = False

	for text in textList:
	 
    		if "hello" == text or "hi" == text:
        		debug("Speaking: Welcome")
			pubtts.publish("Welcome to the Vxlab")
			wordFound = True
			break
    		elif "name" == text:
			debug("Speaking: my name")
			pubtts.publish("my name is Baxter but you can call me Red")
			wordFound = True
			break
		elif "how" == text:
			for text in textList:
				if "are" == text or "doing" == text or "going" == text:
					debug("Speaking: I am fine")
		                        pubtts.publish("I'm very well")
                		        wordFound = True
                         		break
				
			if wordFound:
				break		

    		elif "do" == text:
			debug("Speaking: things")
	        	pubtts.publish("I can rotate, tuck and untuck arms, smile, mimic and other things")
			wordFound = True
  			break
    		elif "untuck" == text:
			debug("Speaking: untucking")
	        	pubtts.publish("sure... I can do that")
			os.system("/home/par/par_ws/utuck")
			wordFound = True
			break
	    	elif "tuck" == text:
			debug("Speaking: tucking")
			pubtts.publish("Tucking arms")
			os.system("/home/par/par_ws/tuck")
			wordFound = True
			break
    		elif "mimic" ==  text:
        		debug("Speaking: Follow me")
	        	os.system("./play-audio Hello2.wav")
			wordFound = True
			break
    		elif "shake" == text:
        		debug("Speaking: Hand Shake")
			pubtts.publish("Sure... I'll do that")
			wordFound = True
			break
    		elif "wave" == text:
        		debug("Speaking: Hi")
	       		pubtts.publish("ok... I am waving")
			wordFound = True
			break
   		elif "dab" == text:
        		debug("Speaking: Dab")
	        	os.system("./play-audio Do_you_really_want_a_dab.wav")
			wordFound = True
			break
    		elif "fist" == text :
        		debug("Speaking: fist Bump")
			os.system("./play-audio Okay.wav")
			wordFound = True
			break
    		elif "understand" == text:
        		debug("Speaking: Understand")
	        	os.system("./play-audio Understood.wav")
			wordFound = True
			break
    		elif "come" == text:
        		debug("Speaking: Challenge Accepted")
	        	pubtts.publish("Challenge accepted")
			wordFound = True
			break
    		elif "five" == text:
        		debug("Speaking: High Five")
	        	os.system("./play-audio High_Five.wav")
			wordFound = True
			break
		elif "stop" in text:
        		debug("Speaking: See_you_next_time")
	        	pubtts.publish("okay... See you next time")
			wordFound = True
			break
    		elif "understand" in text:
        		debug("Speaking: Understood")
	        	os.system("./play-audio Understood.wav")
			wordFound = True
			break
    		elif "waiting" in text:
        		debug("Speaking: Dont understand")
	        	os.system("./play-audio Sorry_can_you_repeat_that.wav")
			wordFound = True
    			break
    		elif "neutral" in text:
        		debug("Speaking: Going neutral position")
	        	os.system("./play-audio Neutral.wav")
			wordFound = True
			break

    	if not wordFound: 
        	debug("Speaking: Red not in keyword")
		pubtts.publish("Oops! Didn't catch that. What did you say?")
   
try:
    rospy.init_node('par_audio_replay', anonymous=True)
    rospy.Subscriber("/input/speech/commands", String, speech)
    debug("starting")
    rospy.spin()
    
except KeyboardInterrupt:
    pass
    

