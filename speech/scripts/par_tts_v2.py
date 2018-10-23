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

    	if "greeting" == text:
        	debug("Speaking: Welcome")
		pubtts.publish("Welcome to the Vxlab")		
    	elif "name" == text:
		debug("Speaking: my name")
		pubtts.publish("my name is Baxter but you can call me Red")
	elif "long greeting" == text:
		debug("Speaking: I am fine")
		pubtts.publish("I'm very well")     		                              				
  	elif "do" == text:
		debug("Speaking: things")
        	pubtts.publish("I can rotate, tuck and untuck arms, smile, mimic and other things")		
  	elif "untuck" == text:
		debug("Speaking: untucking")
	       	pubtts.publish("sure... I can do that")
		os.system("/home/par/par_ws/utuck")			
		#os.system("/home/edwin/ros_ws/utuck.sh")			
	elif "tuck" == text:
		debug("Speaking: tucking")
		pubtts.publish("Tucking arms")
		os.system("/home/par/par_ws/tuck")	
		#os.system("/home/edwin/ros_ws/tuck.sh")	
	elif "mimic" ==  text:
       		debug("Speaking: mimicking")
        	pubtts.publish("Sure... I'll do that")	
		os.system("rostopic pub /red/commands std_msgs/String 'start_kinect'")		
	elif "stop-mimic" ==  text:
       		debug("Speaking: stop mimicking")
        	pubtts.publish("Sure... I'll do that")	
		os.system("rostopic pub /red/commands std_msgs/String 'stop_kinect'")
	elif "rotate" == text:
		debug("speaking: rotating")
		pubtts.publish("Sure... I'll do that")
		os.system("rosrun par_package par_lidar.py")	
		os.system("rosrun par_package red_delay_rotate.py")
		os.system("rosrun par_package red_head_track.py")	
    	elif "shake" == text:
        	debug("Speaking: Shake hands")
		pubtts.publish("Sure... I'll do that")	
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
        	debug("Speaking: no word")
		pubtts.publish("sorry... I didn't get that?")
   
try:
    rospy.init_node('par_listening', anonymous=True)
    rospy.Subscriber("/speechPocketsphinx/speech_recognition", String, speech)
    debug("starting")
    rospy.spin()
    
except KeyboardInterrupt:
    pass
    

