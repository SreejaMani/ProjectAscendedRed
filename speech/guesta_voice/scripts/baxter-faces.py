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
import rospy
from std_msgs.msg import String


pubdebug = rospy.Publisher('/helper/speech_debug', String, queue_size=1)
pub = rospy.Publisher('/robot/xdisplay', Image,latch=True, queue_size=10)

def callback(data):
	global msg
	global msg_closed
	if not data == msg_closed:
		msg = data

def debug(str):
    print(str)
    pubdebug.publish(str)

def faces(data):
    debug("Speaking: "+str(data.data))
    text = data.data
    
    if "big" in text and "wave" in text:
        debug("Face: Hello")
        pub.publish(msg_happy)
    
    elif "hello" in text:
        debug("Face: Hello")
        pub.publish(msg_surprised)
        data=""
        text=""
        

    elif "hand" in text and "shake" in text:
        debug("Face: Hand Shake")
        pub.publish(msg_happy)

    elif "wave" in text:
        debug("Face: Hi")
        pub.publish(msg_happy)

    elif "dab" in text:
        debug("Face: Dab")
        pub.publish(msg_happy)
        pub.publish(msg_happy)

    elif "fist" in text and "bump" in text:
        debug("Face: fist Bump")
        pub.publish(msg_happy)

    elif "understand" in text:
        debug("Face: Understand")
        pub.publish(msg_confused)
        

    elif "come" in text or "here" in text:
        debug("Face: Challenged Accpeted")
        

    elif "high" in text and "five" in text:
        debug("Face: High Five")
        pub.publish(msg_happy)
        pub.publish(msg_happy)


    elif "stop" in text:
        debug("Face: See_you_next_time")
       
    else: 
        debug("Face: Dont understand")
        pub.publish(msg_confused)

        
        
try:
    rospy.init_node('FacialExpressions')
    _images = '/home/guesta/demo_ws/src/baxter_examples/share/baxter-eyes-master'

    img = cv2.imread(_images + '/straight.jpg')
    sub = rospy.Subscriber('/robot/xdisplay', Image, callback)

    img_closed = cv2.imread(_images + '/closed.jpg')
    msg_closed = cv_bridge.CvBridge().cv2_to_imgmsg(img_closed)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img)

    img_happy = cv2.imread(_images + '/happy.jpg')
    msg_happy =cv_bridge.CvBridge().cv2_to_imgmsg(img_happy);

    img_sad = cv2.imread(_images + '/sad.jpg')
    msg_sad =cv_bridge.CvBridge().cv2_to_imgmsg(img_sad);

    img_confused = cv2.imread(_images + '/confused.jpg')
    msg_confused =cv_bridge.CvBridge().cv2_to_imgmsg(img_confused);

    img_surprised = cv2.imread(_images + '/surprised.jpg')
    msg_surprised =cv_bridge.CvBridge().cv2_to_imgmsg(img_surprised);
    
    img_neutral = cv2.imread(_images + '/RedNeutralBaxterFace.jpg')
    msg_neutral =cv_bridge.CvBridge().cv2_to_imgmsg(img_neutral);
   

    while not rospy.is_shutdown():
        pub.publish(msg_neutral)
        timer = random.randint(2,4)	
        time.sleep(timer)
        print "blinking"
        pub.publish(msg_happy)
        #pub.publish(angry)
        timer = random.uniform(0.2,0.5)	
        time.sleep(timer)
        pub.publish(msg_neutral)

        #rospy.init_node('guesta_speech', anonymous=False)
        rospy.Subscriber("/input/speech/commands", String, faces)
        debug("starting")
    #rospy.spin()
    
except KeyboardInterrupt:
    pass
    

