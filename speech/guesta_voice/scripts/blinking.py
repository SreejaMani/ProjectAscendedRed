#!/usr/bin/env python
import rospy
from baxter_interface import Limb
import baxter_interface
import time
import cv2
import cv_bridge
import rospkg
from sensor_msgs.msg import Image
import random



def callback(data):
	global msg
	global msg_closed
	if not data == msg_closed:
		msg = data

rospy.init_node('blinking')

_images = '/home/guesta/demo_ws/src/baxter_examples/share/baxter-eyes-master'
img = cv2.imread(_images + '/straight.jpg')
sub = rospy.Subscriber('/robot/xdisplay', Image, callback)

angry = cv2.imread(_images + '/angry.jpg')
angry_disgust = cv2.imread(_images + '/angry_disgust.jpg')

img_closed = cv2.imread(_images + '/closed.jpg')
# msg_closed = cv_bridge.CvBridge().cv2_to_imgmsg(img_closed)
# msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)
msg_closed = cv_bridge.CvBridge().cv2_to_imgmsg(angry)
msg = cv_bridge.CvBridge().cv2_to_imgmsg(angry_disgust)


pub = rospy.Publisher('/robot/xdisplay', Image,latch=True, queue_size=10)

while not rospy.is_shutdown():
	
	timer = random.randint(2,4)	

	time.sleep(timer)
	print "blinking"

	pub.publish(msg_closed)
	#pub.publish(angry)

	timer = random.uniform(0.2,0.5)	
	time.sleep(timer)

	pub.publish(msg)
	#pub.publish(angry_disgust)