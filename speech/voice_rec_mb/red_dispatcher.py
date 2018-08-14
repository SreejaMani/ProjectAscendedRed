#!/usr/bin/python
import rospy
from std_msgs.msg import String
import thread

import urllib2

pubdebug = rospy.Publisher('/red/speech_debug', String, queue_size=5)

def lidar_display(t1,t2):
	debug('lidar l1 '+str(t1))
	urllib2.urlopen("http://10.42.1.2/cmd/set_parameter?hmi_application_text_1="+str(t1))
	debug('lidar l2 '+str(t2))
	urllib2.urlopen("http://10.42.1.2/cmd/set_parameter?hmi_application_text_2="+str(t2))

def debug(str):
  global pubdebug
  print('dispatcher:'+str)
  pubdebug.publish('dispatcher:'+str)

import baxter_interface
from baxter_examples import JointRecorder

from baxter_interface import CHECK_VERSION

import urllib2
from subprocess import call

recorder = None

def start_recording():
	debug("start_recording requested")
    	global recorder
	# only if not already recording
    	if recorder is None:
		debug("start_recording since none is in progress")
    #rs = baxter_interface.RobotEnable(CHECK_VERSION)
    #print("Enabling robot... ")
    #rs.enable()
    		recorder = JointRecorder('/home/mb/baxter-recording.csv', 100)
    		recorder.record()

def stop_recording():
	debug("end_recording requested")
    	global recorder
	# only if recording
    	if recorder is not None:
		debug("ending current recording")
    		recorder.stop()
		recorder = None
		urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.play")

def play_recording():
	debug("play_recording requested");
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.play")

def stop_playback():
	debug("stop_playback requested; not implemented");

def tuck_arms():
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.tuck_disable")

def untuck_arms():
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.enable_untuck")

def motors_on():
	debug('motors_on calling urlopen')
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.motors.on")
	debug('motors_on urlopen complete')

def motors_off():
	urllib2.urlopen("http://mb.vx.rmit.edu.au:9000/baxter.motors.off")

def restart_speech():
	debug('restart_speech...')
	call(["systemctl", "--user", "stop", "speech"])
	call(["systemctl", "--user", "start", "speech"])

def dispatch(data):
	debug("received command: "+data.data)
        thread.start_new_thread(dispatch_inner, tuple([data]));
	debug("started thread")

def dispatch_inner(data):
	debug("dispatch_inner: "+data.data)
	command = data.data
	lidar_display('command',command)
	if command == "tuck_arms":
		tuck_arms()
	if command == "untuck_arms":
		untuck_arms()
	if command == "restart_speech":
		restart_speech()
	if command == "motors_on":
		motors_on()
	if command == "motors_off":
		motors_off()
	if command == "start_recording":
		start_recording()
	if command == "stop_recording" or command == "end_recording" or command == "stop":
		stop_recording()
	if command == "play_recording":
		play_recording()
	if command == "stop_playback":
		stop_playback()

try:
    rospy.init_node('red_dispatcher', anonymous=False)
    rospy.Subscriber("/red/commands", String, dispatch)
    debug("started")
    rospy.spin()
except KeyboardInterrupt:
    pass
