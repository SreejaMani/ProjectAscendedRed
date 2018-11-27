#!/usr/bin/python
# coding: utf-8

import random
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

# This function publishes a msg to text to speech topic and also executes commands when needed
def speech(data):
    	debug("Listening: "+ str(data.data))
	text = data.data
	
	#These are list of possible answers that will be randomly selected to publish on gTTS topic
	longGreetAnswers = ["I am fine, thanks for asking", "I'm very well", "not bad", "I am good thanks", 
		"Great! Couldn’t be better!", "I can’t complain", "I feel good"]
	untuckAnswers = ["sure... I can do that", "sure... I'll do that", "ok... you got it", "untucking arms"]
	tuckAnswers = ["sure... I can do that", "sure... I'll do that", "ok... you got it", "tucking arms"]
	generalAnswers = ["sure... I can do that", "sure... I'll do it", "ok... you got it", "I am able to do that"]
	wordNoFoundAns = ["sorry...", "I didn't catch that", "what was that?","say it again",
			"beg you pardon?", "sorry... come again", "sorry... I didn't get that", "I did not understand"]
    	if "greeting" == text:
        	debug("Speaking: Welcome")
		pubtts.publish("Welcome to the media portal")		
    	elif "name" == text:
		debug("Speaking: my name")
		pubtts.publish("my name is Baxter but you can call me Red")
	elif "long greeting" == text:
		# get a random word from the list
	    	answer = random.choice(longGreetAnswers)
		debug("Speaking: long greeting")
		pubtts.publish(answer)     		                              				
  	elif "do" == text:
		debug("Speaking: things")
        	pubtts.publish("I can rotate, tuck and untuck arms, smile, mimic and other things")		
  	elif "untuck" == text:
		# get a random word from the list
	    	answer = random.choice(untuckAnswers)
		debug("Speaking: untucking")
	       	pubtts.publish(answer)
		os.system("/home/par/par_ws/utuck")			
		#os.system("/home/edwin/ros_ws/utuck")			
	elif "tuck" == text:
		# get a random word from the list
	    	answer = random.choice(tuckAnswers)
		debug("Speaking: tucking")
		pubtts.publish(answer)
		os.system("/home/par/par_ws/tuck")	
		#os.system("/home/edwin/ros_ws/tuck")	
	elif "mimic" ==  text:
		# get a random word from the list
	    	answer = random.choice(generalAnswers)
       		debug("Speaking: mimicking")
        	pubtts.publish(answer)	
		#os.system("rostopic pub /red/commands std_msgs/String 'start_kinect'")
		os.system("rosrun par_package kinect_two_hands.py")
	elif "stop-mimic" ==  text:
		# get a random word from the list
	    	answer = random.choice(generalAnswers)
       		debug("Speaking: stop mimicking")
        	pubtts.publish(answer)	
		#os.system("rostopic pub /red/commands std_msgs/String 'stop_kinect'")
		os.system("rosnode kill rosout lift_ik_prototype")
	elif "rotate" == text:
		# get a random word from the list
	    	answer = random.choice(generalAnswers)
		debug("speaking: rotating")
		pubtts.publish(answer)
		os.system("roslaunch par_package mobility_read.launch")	
	elif "stop-rotating" == text:
		# get a random word from the list
             	answer = random.choice(generalAnswers)
               	debug("speaking: stop rotating")
             	pubtts.publish(answer)
		
   	elif "shake" == text:
		# get a random word from the list
	    	answer = random.choice(generalAnswers)
        	debug("Speaking: Shake hands")
		pubtts.publish(answer)	
	elif "big-wave" == text:
       		debug("Speaking: big waving")
       		pubtts.publish("sure... I'll do a big wave")
	elif "wave" == text:
       		debug("Speaking: waving")
       		pubtts.publish("ok... I am waving")					
	elif "fist" == text :
       		debug("Speaking: fist Bump")
		pubtts.publish("ok... you got it")						
	elif "come" == text:
       		debug("Speaking: Challenge Accepted")
        	pubtts.publish("Challenge accepted")			
	elif "high-five" == text:
       		debug("Speaking: High Five")
        	pubtts.publish("high five")			
	elif "stop" in text:
       		debug("Speaking: See_you_next_time")
        	pubtts.publish("okay... See you next time")						  			
    	elif "wordnotfound" in text:
		# get a random word from the list
	    	answer = random.choice(wordNoFoundAns)
        	debug("Speaking: no word")
		pubtts.publish(answer)
   
try:
    rospy.init_node('par_listening', anonymous=True)
    rospy.Subscriber("/speechPocketsphinx/speech_recognition", String, speech)
    debug("starting")
    rospy.spin()
    
except KeyboardInterrupt:
    pass
    

