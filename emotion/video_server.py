#! /usr/bin/env python

import rospy, os
import cv2, cv_bridge
from sensor_msgs.msg import Image
from nas_launch.msg import video, request
from nas_launch.srv import play_movie

path = "/home/mb/vxlab_ws/src/nas_launch/videos/"
videos = {}
screen = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
status = rospy.Publisher('/robot/video', video, latch=True, queue_size=10)


def playVideo(vid, name):
    frame = vid.grab()
    framerate = rospy.Rate(vid.get(5))

    while not rospy.is_shutdown() and vid.retrieve()[0]:

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(vid.retrieve()[1], encoding="bgr8")
        screen.publish(msg)
        framerate.sleep()
        frame=vid.grab()
        status.publish(name, True)
    status.publish(None, False)
    videos[name] = cv2.VideoCapture(path+name+".mp4")


def play(data):
    playVideo(videos[data.video_request], data.video_request)

def listener():
    rospy.init_node('movie_player')
    rospy.Subscriber("video_request", request, play, queue_size=3)
    rospy.spin()

if __name__ == '__main__':

    for file in os.listdir(path):
        videos[file.rstrip(".mp4")] = cv2.VideoCapture(path+file)

    listener()

   

