#!/usr/bin/python
from gtts import gTTS
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
    debug("Speaking: "+ str(data.data))
    text2speech = data.data
   
    if "hello" in text2speech and "hi":
        debug("Speaking: Welcome")
	tts = gTTS(text = 'Welcome to the vxlab', lang= 'en')
	tts.save("temp.mp3")
        os.system("mpg321 temp.mp3")
    elif "big" in text2speech and "wave" in text2speech:
        debug("Speaking: Hello there")
	tts = gTTS(text = "sure, I'll do that", lang= 'en')
	tts.save("temp.mp3")
        os.system("mpg321 temp.mp3")

    elif "hand" in text2speech and "shake" in text2speech:
        debug("Speaking: Hand Shake")
	os.system("./play-audio Hi.wav")


    elif "wave" in text2speech:
        debug("Speaking: Hi")
        os.system("./play-audio Hello3.wav")

    elif "dab" in text2speech:
        debug("Speaking: Dab")
        os.system("./play-audio Do_you_really_want_a_dab.wav")

    elif "fist" in text2speech and "bump" in text2speech:
        debug("Speaking: fist Bump")
	os.system("./play-audio Okay.wav")

    elif "understand" in text2speech:
        debug("Speaking: Understand")
        os.system("./play-audio Understood.wav")

    elif "come" in text2speech or "here" in text2speech:
        debug("Speaking: Challenged Accpeted")
        os.system("./play-audio Challenge_accepted.wav")

    elif "high" in text2speech and "five" in text2speech:
        debug("Speaking: High Five")
        os.system("./play-audio High_Five.wav")

    elif "stop" in text2speech:
        debug("Speaking: See_you_next_time")
        os.system("./play-audio See_you_next_time.wav")

    elif "understand" in text2speech:
        debug("Speaking: Understood")
        os.system("./play-audio Understood.wav")

    elif "waiting" in text2speech:
        debug("Speaking: Dont understand")
        os.system("./play-audio Sorry_can_you_repeat_that.wav")
    
    elif "neutral" in text2speech:
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
    

