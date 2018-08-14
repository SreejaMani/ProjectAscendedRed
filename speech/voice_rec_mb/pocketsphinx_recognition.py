#!/usr/bin/python
import speech_recognition as sr
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/red/speech_test', String, queue_size=1)
pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=5)

def debug(str):
    print(str)
    pubdebug.publish(str)

r = sr.Recognizer()

m = None

listens_remaining = 0;
speechlock = 0;

def dospeech(data):
    # debug("Incoming request")
    # global speechlock
    # if speechlock:
    #     debug("Dropped overrunning speech command")
    #     return
    # else:
    #     speechlock = 1;
    listens = 0
    while True:
        listens += 1 
        try:
            debug("Say something! (Attempt "+str(listens)+")")
            with m as source: audio = r.listen(source)

	#with sr.AudioFile("/home/mb/baxter_speech/speech/helloworld/test-jc.wav") as source:
	#	audio = r.record(source)
	#	r.recognize_google(audio)
	
            debug("Got it! Now to recognize it...")
        # try:
            # recognize speech using Google Web API (only a few requests per day allowed)
            value = r.recognize_sphinx(audio)

            # we need some special handling here to correctly print unicode characters to standard output
            if str is bytes:  # this version of Python uses bytes for strings (Python 2)
                #debug(u"You said { (binary)}".format(value).encode("utf-8"))
                debug("You said: "+value)
		pub.publish(value);
            else:  # this version of Python uses unicode for strings (Python 3+)
                #debug("You said {} (plaintext)".format(value))
                debug("You said: "+value)
		pub.publish(value);
        except sr.UnknownValueError:
            debug("Oops! Didn't catch that")
        except sr.RequestError as e:
            debug("{0}".format(e))
        except KeyboardInterrupt:
            pass
        speechlock = 0

try:
    rospy.init_node('red_speech_test', anonymous=False)

    mic_index = None
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
	 debug("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))
	 if "HD Pro Webcam C920" in name:
	     debug("Using index "+str(index))
	     mic_index = index 

    m = sr.Microphone(device_index=mic_index)

    print("A moment of silence, please...")
    with m as source: r.adjust_for_ambient_noise(source)
    print("Set minimum energy threshold to {}".format(r.energy_threshold))
    #rospy.Subscriber("/red/speech_control", String, dospeech)
    debug("Started")
    dospeech(None)
    #rospy.spin()

except KeyboardInterrupt:
    pass
