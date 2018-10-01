#!/usr/bin/python

import rospy
from std_msgs.msg import String

#the publisher topic which publishing command to /input/speech/commands, and if need for debugging at /guesta/speech_debug
pubdebug = rospy.Publisher('/par/speech_debug', String, queue_size=1)
pub = rospy.Publisher('/input/speech/commands', String, queue_size=1)

#the debugging part
def debug(str):
    print(str)
    pubdebug.publish(str)

listens_remaining = 0;
speechlock = 0;


# The part where check for word in the text list and send out the command to the publishing command.
# Can just edit the bottom code to tailor made for own use by changing the incoming word and the output command
def dospeech(data):
    debug("Do speech "+str(data.data))
    text = data.data
    if "Red" in text or "Baxter" in text:
        debug("ears: Found keyphrase RED")
	if "hello" in text and "hi" in text:
	    debug("ears: Hello")
	    pub.publish(text)
        if "follow" in text and ("me" in text):
            debug("ears: follow me")
            pub.publish("follow me")

        elif "hand" in text and ("shake" in text or "shaking" in text):
            debug("ears: hand shake")
            pub.publish("hand shake")

        elif "wave" in text or "waving" in text:
            debug("ears: wave")
            pub.publish("wave")

        elif "fist" in text or "bump" in text:
            debug("ears: fist bump")
            pub.publish("fist bump")

        elif "hello" in text or "hey" in text:
            debug("ears: Welcome")
            pub.publish("hello")

        elif "understand" in text:
            debug("ears: understand")
            pub.publish("understand")

        elif "challenge" in text or "challenging" in text:
            debug("ears: come here")
            pub.publish("come here")

        elif "high" in text and "five" in text:
            debug("ears: high five")
            pub.publish("high five")

        elif "dab" in text or "dabbing" in text:
            debug("ears: dab")
            pub.publish("dab")

        elif "stop" in text or "finished" in text or "finish" in text:
            debug("ears: stop")
            pub.publish("stop")
            
        elif "neutral" in text or "standby" in text or ("stand" in text and "by" in text):
            debug("ears: Going to neutral position")
            pub.publish("neutral")
    
        else: 
            debug("ears: don't understand...")
            pub.publish("waiting")
    else:
        if "neutral" in text or "standby" in text or ("stand" in text and "by" in text):
            debug("Going to neutral position")
            pub.publish("neutral")
        else:
            debug("ears: Red is not in text...")
            pub.publish("what")

try:
    rospy.init_node('par_speech_command', anonymous=True)
    rospy.Subscriber("/red/speech_test", String, dospeech)
    debug("Started")
    rospy.spin()
except KeyboardInterrupt:
    pass
