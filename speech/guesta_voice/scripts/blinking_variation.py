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

img_closed = cv2.imread(_images + '/closed.jpg')
msg_closed = cv_bridge.CvBridge().cv2_to_imgmsg(img_closed)
msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)

img_left = cv2.imread(_images + '/left.jpg')
msg_left =cv_bridge.CvBridge().cv2_to_imgmsg(img_left);

img_right = cv2.imread(_images + '/right.jpg')
msg_right =cv_bridge.CvBridge().cv2_to_imgmsg(img_right);


pub = rospy.Publisher('/robot/xdisplay', Image,latch=True, queue_size=10)

while not rospy.is_shutdown():
	
	timer = random.randint(2,4)	

	time.sleep(timer)
	print "blinking"

	
	#Closes eyes
	pub.publish(msg_closed)

	timer = random.uniform(0.2,0.5)	
	time.sleep(timer)

	pub.publish(msg)
	
	timer = random.randint(2,4)	
	time.sleep(timer)
	pub.publish(msg_left)
	
	timer = random.randint(2,4)	
	time.sleep(timer)
	pub.publish(msg_right)
