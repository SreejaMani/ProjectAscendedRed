#!/usr/bin/env python

import time
import os
from os import path
from pocketsphinx import Decoder
import speech_recognition as sr
import rospy
from std_msgs.msg import Float32, String

#MODELDIR = "/home/edwin/ros_ws/src/baxter_examples/RedAscended/speechPocketsphinx/pocketsphinxTools/baxter-dict"
MODELDIR = "/home/par/par_ws/src/par_package/scripts/speechPocketsphinx/pocketsphinxTools/baxter-dict"
#Config the decoder
config = Decoder.default_config()
config.set_string('-hmm', path.join(MODELDIR, 'en-us-baxter'))
config.set_string('-lm', path.join(MODELDIR, 'baxter.lm'))
config.set_string('-dict', path.join(MODELDIR, 'baxter.dict'))


config.set_string("-logfn", os.devnull)
decoder = Decoder(config)

def debug(str):
	print(str)
	pubdebug.publish(str)

def recognize_speech_from_mic(recognizer, microphone):
	"""Transcribe speech from recorded from `microphone`.

    	Returns a dictionary with three keys:
    	"success": a boolean indicating whether or not the API request was
        	       successful
    	"error":   `None` if no error occured, otherwise a string containing
        	       an error message speech was unrecognizable
    	"transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    	"""
    	# check that recognizer and microphone arguments are appropriate type
	if not isinstance(recognizer, sr.Recognizer):
		raise TypeError("`recognizer` must be `Recognizer` instance")

    	if not isinstance(microphone, sr.Microphone):
        	raise TypeError("`microphone` must be `Microphone` instance")

    	# adjust the recognizer sensitivity to ambient noise and record audio
	# from the microphone
	with microphone as source:
		#Adjust the mic energy threshold for ambient noise
		recognizer.adjust_for_ambient_noise(source)
		#Green led indicator		
		red.publish(0)
		green.publish(100)
        	print("A moment of silence, please...")
        	print("Set minimum energy threshold to {}".format(recognizer.energy_threshold))
        	print "Say something!"
        	audio = recognizer.listen(source)
        	print "Got it! Now to recognize it..."
		#Adjust the mic energy threshold for ambient noise
		#recognizer.adjust_for_ambient_noise(source)

    	# set up the response object
	response = {
	       	"success": True,
        	"error": None,
        	"transcription": None
    	}

    	# try recognizing the speech in the recording
    	# if a RequestError or UnknownValueError exception is caught,
    	# update the response object accordingly
    	try:
        	raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
        	decoder.start_utt()
        	decoder.process_raw(raw_data, False, True)
        	decoder.end_utt()
        	hypothesis = decoder.hyp()
        	if hypothesis != None:
        		response["transcription"] = hypothesis.hypstr
        	else:
            		response["transcription"] = hypothesis
            		response["success"] = False
            		response["error"] = "Unable to recognize speech"

    	except:
        	response["success"] = False
        	response["error"] = "Unable to recognize speech"
        	pass

    	return response


def set_up_mic():
    	# create recognizer and mic instances
    	recognizer = sr.Recognizer()
    	recognizer.energy_threshold = 4000
	recognizer.dynamic_energy_threshold = False
    	# find the mic to use
    	wordFound = False
    	microphone = None
    	for index, mic_name in enumerate(sr.Microphone.list_microphone_names()):
        	micsplit = mic_name.split()
        	for micname in micsplit:
            		if micname == "Xbox":
			#if micname == "Webcam":
            		#if micname == "pulse":
                		print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, mic_name))
                		microphone = sr.Microphone(device_index=index)
                		microphone.SAMPLE_RATE = 16000
                		break
        		
			if microphone != None:
        	    		break

    	if microphone == None:
        	print("No working microphones found!")
        	exit()

    	while not rospy.is_shutdown():

        	response = recognize_speech_from_mic(recognizer, microphone)
        	if response["transcription"]:
        		pass
        	if not response["success"]:
        		pass
        	if response["error"]:
            		print("Oops! Didn't catch that. What did you say?\n")
            		print("ERROR: {}".format(response["error"]))
            		response["transcription"] = "word not found"
            		pass

        	phrase = response["transcription"].lower()
        	# print the phrase to the console
        	print("You said: {}".format(phrase))

		#check if red or baxter is in the phrase
		if "red" in phrase or "baxter" in phrase: 
	            	if "hi" == phrase or "hello" in phrase or ("good" in phrase and ("morning" in phrase or "afternoon" in phrase)):
        	        	#red led indicator 
            			red.publish(50)
                      		green.publish(50)
				debug("greeting")
		               	pub.publish("greeting")          
        	 	    	# Wait for 5 seconds before getting another input
        	 	       	for _ in range(50): rospy.sleep(0.1)
             	       	
        	 	elif ("what" in phrase or "do" in phrase) and "name" in phrase:
				#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
				debug("name")
        	        	pub.publish("name")                
        	        	# Wait for 5 seconds before getting another input
        	        	for _ in range(50): rospy.sleep(0.1)
                	
        	    	elif "how" in phrase and ("are" in phrase or "doing" in phrase or "going" in phrase):
        	      		#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
				debug("long greeting")
		        	pub.publish("long greeting")                       
        	       		# Wait for 5 seconds before getting another input
        	       		for _ in range(50): rospy.sleep(0.1)
                    			
			elif ("what" in phrase or "things" in phrase) and "do" in phrase:
        	        	#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
				debug("do")
        	        	pub.publish("do")            
				# Wait for 5 seconds before getting another input
		      		for _ in range(80): rospy.sleep(0.1)
			
		   	elif "untuck" in phrase and "arms" in phrase:
				#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
		                debug("untuck")
        	       		pub.publish("untuck")                
		                # Wait for 10 seconds before getting another input
        	       		for _ in range(100): rospy.sleep(0.1)
                     		
			elif "tuck" in phrase and "arms" in phrase:
				#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
      	         		debug("tuck")
      	         		pub.publish("tuck")               
      	         		# Wait for 10 seconds before getting another input
      	         		for _ in range(100): rospy.sleep(0.1)
      	               		
        		elif ("mimic" in phrase or "mirror" in phrase)and "me" in phrase:
        	       		#red led indicator      
                               	red.publish(50)
                              	green.publish(50)
				debug("mimic")
        	       		pub.publish("mimic")                
        	       		# Wait for 15 seconds before getting another input
        	       		for _ in range(150): rospy.sleep(0.1)
                	       		
        		elif "shake" in phrase and "hands" in phrase:
        	       		#red led indicator      
                               	red.publish(50)
                               	green.publish(50)
				debug("shake")
        	      		pub.publish("shake")
        	       		# Wait for 10 seconds before getting another input
        	       		for _ in range(130): rospy.sleep(0.1)
        	       		
	       		elif "big" in phrase and "wave" in phrase:
		                #red led indicator      
                               	red.publish(50)
                               	green.publish(50)
				debug("big-wave")
        	       		pub.publish("big-wave")    
        	       		# Wait for 10 seconds before getting another input
        	       		for _ in range(160): rospy.sleep(0.1)
	
			elif "wave" in phrase:
				#red led indicator      
                               	red.publish(50)
                                green.publish(50)
		                debug("wave")
        	       		pub.publish("wave")    
        	       		# Wait for 10 seconds before getting another input
        	       		for _ in range(180): rospy.sleep(0.1)
               		
		        elif "fist" in phrase and "bump" in phrase:
		                #red led indicator      
                                red.publish(50)
                                green.publish(50)
				debug("fist-bump")
        	       		pub.publish("fist-bump")
		                # Wait for 10 seconds before getting another input
        	       		for _ in range(130): rospy.sleep(0.1)
                       		
        		elif "come" in phrase and "here" in phrase:
		               #red led indicator      
                                red.publish(50)
                                green.publish(50)
				debug("come-here")
        	       		pub.publish("come-here")
        	       		# Wait for 10 seconds before getting another input
        	      		for _ in range(180): rospy.sleep(0.1)
                       		
       			elif "high" in phrase and "five" in phrase:
        	       		#red led indicator      
                                red.publish(50)
                                green.publish(50)
				debug("high-five")
        	       		pub.publish("high-five")
		                # Wait for 10 seconds before getting another input
        	       		for _ in range(150): rospy.sleep(0.1)
           		
		       	elif "stop" in phrase and "mimic" in phrase:
		                #red led indicator      
                                red.publish(50)
                                green.publish(50)
				debug(phrase)
        	       		pub.publish("stop-mimic")
		                # Wait for 5 seconds before getting another input
        	       		for _ in range(50): rospy.sleep(0.1)
			elif "rotate" in phrase:
				#red led indicator      
                                red.publish(50)
                                green.publish(50)
				debug(phrase)
        	       		pub.publish("rotate")
		                # Wait for 15 seconds before getting another input
        	       		for _ in range(250): rospy.sleep(0.1) 
			elif "stop" in phrase and "rotating" in phrase:
				#red led indicator      
                                red.publish(50)
                                green.publish(50)
				os.system("rosnode kill rosout look_closest_person")
 				os.system("rosnode kill rosout face_closest_person")
		    		os.system("rosnode kill rosout par_lidar")
				debug(phrase)
        	       		pub.publish("stop-rotating")
		                # Wait for 15 seconds before getting another input
        	       		for _ in range(50): rospy.sleep(0.1)              		
			else:
				#red led indicator      
                                red.publish(50)
                                green.publish(50)
        			debug("Speaking: word not found")
				pub.publish("wordnotfound")
				# Wait for 5 seconds before getting another input
        	       		for _ in range(50): rospy.sleep(0.1)
		else:
			debug("Speaking: red or baxter is not in the phrase")

if __name__ == "__main__":
	try:
        	pub = rospy.Publisher('/speechPocketsphinx/speech_recognition', String, queue_size=10)
	        pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=10)
        	rospy.init_node('speech_recognition', anonymous=False, disable_signals=True)
		red = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level', Float32, latch=True, queue_size=1)
		green = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32, latch=True, queue_size=1)
        	set_up_mic()
	except rospy.ROSInterruptException:
	        pass
