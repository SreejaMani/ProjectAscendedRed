#!/usr/bin/env python

import time
import os
from os import path
from pocketsphinx import Decoder
import speech_recognition as sr
import rospy
from std_msgs.msg import String

MODELDIR = "/home/edwin/ros_ws/src/baxter_examples/RedAscended/speechPocketsphinx/pocketsphinxTools/baxter-dict"

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
    # find the mic to use
    wordFound = False
    microphone = None
    for index, mic_name in enumerate(sr.Microphone.list_microphone_names()):
        micsplit = mic_name.split()
        for micname in micsplit:
            #	if micname == "Xbox":
            if micname == "pulse":
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
            response["transcription"] = "Oops! Didn't catch that. What did you say?"
            pass

        phrase = response["transcription"].lower()
        # print the phrase to the console
        print("You said: {}".format(phrase))

        wordList = phrase.split()
        for key_word in wordList:
            if "hello" == key_word or "hi" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 5 seconds before getting another input
                for _ in range(50): rospy.sleep(0.1)
                wordFound = True
                break
            elif "name" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 5 seconds before getting another input
                for _ in range(50): rospy.sleep(0.1)
                wordFound = True
                break
            elif "how" == key_word:
                for key_word in wordList:
                    if "are" == key_word or "doing" == key_word or "going" == key_word:
                        debug(key_word)
                        pub.publish(key_word)                        
                        # Wait for 5 seconds before getting another input
                        for _ in range(50): rospy.sleep(0.1)
                        wordFound = True
                        break
                if wordFound:
                    break
            elif "do" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 5 seconds before getting another input
                for _ in range(50): rospy.sleep(0.1)
                wordFound = True
                break
            elif "untuck" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "tuck" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "mimic" == key_word:
                debug(key_word)
                pub.publish(key_word)                
                # Wait for 15 seconds before getting another input
                for _ in range(150): rospy.sleep(0.1)
                wordFound = True
                break
            elif "shake" == key_word:
                debug(key_word)
                pub.publish(key_word)
                # Wait for 10 seconds before getting another input
                for _ in range(10): rospy.sleep(0.1)
                wordFound = True
                break
            elif "big" == key_word or "wave" == key_word:
                debug(key_word)
                pub.publish(key_word)    
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "fist" == key_word:
                debug(key_word)
                pub.publish(key_word)
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "come" == key_word:
                debug(key_word)
                pub.publish(key_word)
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "five" == key_word:
                debug(key_word)
                pub.publish(key_word)
                # Wait for 10 seconds before getting another input
                for _ in range(100): rospy.sleep(0.1)
                wordFound = True
                break
            elif "stop" in key_word:
                debug(key_word)
                pub.publish(key_word)
                # Wait for 5 seconds before getting another input
                for _ in range(50): rospy.sleep(0.1)
                wordFound = True
                break
            elif "neutral" in key_word:
                debug(key_word)
                pub.publish(key_word)          
                # Wait for 5 seconds before getting another input
                for _ in range(50): rospy.sleep(0.1)
                wordFound = True
                break

        if not wordFound:
            debug("Speaking: word not found")
            pub.publish("wordnotfound")


if __name__ == "__main__":
    try:
        pub = rospy.Publisher('/speechPocketsphinx/speech_recognition', String, queue_size=10)
        pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=10)
        rospy.init_node('speech_recognition', anonymous=False, disable_signals=True)
        set_up_mic()
    except rospy.ROSInterruptException:
        pass
