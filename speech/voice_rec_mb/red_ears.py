#!/usr/bin/python
import rospy
from std_msgs.msg import String

pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=5)
pub = rospy.Publisher('/red/commands', String, queue_size=5)

def debug(str):
  global pubdebug
  print(str)
  pubdebug.publish(str)

listens_remaining = 0;
speechlock = 0;

def http_done():
	debug("HTTP request done")

def dospeech(data):
	indata = data.data
	text = indata.lower()
	debug("red_ears: received text: "+str(text))
	if "red" in text or "robot" in text or "redback" in text or "baxter" in text:
		debug("ears: Found keyphrase RED")
		if "arms" in text and ("deploy" in text or "untuck" in text or "untuk" in text or "start" in text):
			debug("ears: untuck arms")
			pub.publish("untuck_arms")
		if "arms" in text and ("stow" in text or "tuck" in text):
			debug("ears: tuck arms")
			pub.publish("tuck_arms")
		if "finished" in text:
			debug("ears: tuck arms")
			pub.publish("tuck_arms")
		if ("start" in text or "make" in text) and ("record" in text or "recording" in text):
			debug("ears: start recording")
			pub.publish("start_recording")
		if "stop" in text and ("record" in text or "recording" in text) :
			debug("ears: stop recording")
			pub.publish("stop_recording")
		if "play" in text or "playback" in text:
			debug("ears: playback")
			pub.publish("play_recording")
		if "shut" in text and "down" in text:
			debug("ears: tuck arms")
			pub.publish("tuck_arms")
		if "get" in text and "started" in text:
			debug("ears: untuck arms")
			pub.publish("untuck_arms")
		if ("motors" in text) and ("off" in text or "disable" in text):
			debug("ears: motors off")
			pub.publish("motors_off")
		if ("motors" in text) and ("on" in text or "enable" in text):
			debug("ears: motors on")
			pub.publish("motors_on")

try:
    rospy.init_node('red_ears', anonymous=False)
    rospy.Subscriber("/red/speech_test", String, dospeech)
    debug("red_ears: started")
    rospy.spin()
except KeyboardInterrupt:
    pass
