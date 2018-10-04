#!/usr/bin/env python

import time
import os
from os import path
from pocketsphinx import Decoder
import speech_recognition as sr
import rospy
from std_msgs.msg import String

MODELDIR = "baxter-dict"

config = Decoder.default_config()
config.set_string('-hmm', path.join(MODELDIR, 'en-us-baxter'))
config.set_string('-lm', path.join(MODELDIR, 'baxter.lm'))
config.set_string('-dict', path.join(MODELDIR, 'baxter.dict'))


config.set_string("-logfn", os.devnull)
decoder = Decoder(config)

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
		print("A moment of silence, please...")
		recognizer.adjust_for_ambient_noise(source, duration=1)
		print("Set minimum energy threshold to {}".format(recognizer.energy_threshold))
		print "Say something!"
		audio = recognizer.listen(source)
		print "Got it! Now to recognize it..."

	# set up the response object
	response = {
		"success": True,
		"error": None,
		"transcription": None
	}

	# try recognizing the speech in the recording
	# if a RequestError or UnknownValueError exception is caught,
	#     update the response object accordingly
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
	pub = rospy.Publisher('/input/speech/commands', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	# create recognizer and mic instances
	recognizer = sr.Recognizer()
	recognizer.energy_threshold = 4000
	# find the mic to use
	for index, name in enumerate(sr.Microphone.list_microphone_names()):
		if name == "alsa_input.usb-046d_HD_Pro_Webcam_C920_D02D1A9F-02.analog-stereo":
			print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))
			microphone = sr.Microphone(device_index=index)
			microphone.SAMPLE_RATE = 16000
			break
	if microphone == None:
		print("No working microphones found!")
		exit()	
	
	while True:
	
		response = recognize_speech_from_mic(recognizer, microphone)
		if response["transcription"]:
			pass
		if not response["success"]:
			pass
		if response["error"]:
			print("Oops! Didn't catch that. What did you say?\n")
			print("ERROR: {}".format(response["error"]))
			response["transcription"] = "Oops! Didn't catch that. What did you say?"
			pass

		# show the user the transcription
		print("You said: {}".format(response["transcription"]))
		rospy.loginfo(response)
		pub.publish(response["transcription"].lower())
		rate.sleep()
		# Wait for 5 seconds before getting another input
		for _ in range(50): time.sleep(0.1)


if __name__ == "__main__":
	try:
		set_up_mic()
	except rospy.ROSKeyboardInterrupt:
		pass
