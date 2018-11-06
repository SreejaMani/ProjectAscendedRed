#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import math
import random
import time
import threading
import os
from os import path
from std_msgs.msg import String

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Gesture(object):

	def __init__(self):
       
		self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
	        self._left_arm = baxter_interface.limb.Limb("left") # getting control of the left arm
	        self._right_arm = baxter_interface.limb.Limb("right") # getting control of the right arm
	        self._left_joint_names = self._left_arm.joint_names() # Getting the left joint names
	        self._right_joint_names = self._right_arm.joint_names()  # Getting the names of the right joint

	        # control parameters
	        self._rate = 500.0  # Hz
	        self._left_arm.set_joint_position_speed(0.5)
	        self._right_arm.set_joint_position_speed(0.5)
	
		#I need to test this in the real baxter... we may not need RobotEnable(CHECK_VERSION) neither enable_robot.py -e
		#However, I need to decide between neutral pose or tuck arms. Which is better 
	        print("Getting robot state... ")
	        #self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
	        #self._init_state = self._rs.state().enabled #"it takes the last state of the robot 
		#and enabling it from that state. This may not be a good idea"
	        print("Enabling robot... ")
	        #self._rs.enable()#this is the problem check this out
		#self.set_neutral()
		#os.system("rosrun baxter_tools enable_robot.py -e")
	
       
    	def _reset_control_modes(self):
        	rate = rospy.Rate(self._rate)
        	for _ in xrange(100):
        	    if rospy.is_shutdown():
         	       return False
            	self._left_arm.exit_control_mode()
            	self._right_arm.exit_control_mode()
            	self._pub_rate.publish(100)  # 100Hz default joint state rate
            	rate.sleep()
        	return True

    	def set_neutral(self):
        	"""
        	Sets both arms and the head back into a neutral pose.
        	"""
        	print("Moving to neutral pose...")
        	self._left_arm.move_to_neutral()
        	self._right_arm.move_to_neutral()
        	baxter_interface.Head().set_pan(0.0, speed=0.5, timeout=10)


    	def clean_shutdown(self):
        	print("\nExiting example...")
        	#return to normal
        	self._reset_control_modes()
		#os.system("/home/edwin/ros_ws/tuck")
		os.system("/home/par/par_ws/tuck")
        	#self.set_neutral()
 		#if not self._init_state:
 	        #	print("Disabling robot...")           	
		#self._rs.disable()
        	return True

    	def wave(self):
        
        	"""
        	Performs a royal wave
        	"""
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

        	limb = baxter_interface.Limb('right')
        	limb.set_joint_position_speed(0.7)
		
		wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714,
				'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
		wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 
				'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}
		
        	for _move in range(3):
        		limb.move_to_joint_positions(wave_1, timeout=3, threshold=0.1)
            		limb.move_to_joint_positions(wave_2,timeout=2, threshold=0.1)
		
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

    	def bigwave(self):
 
        	"""
        	Performs an exagerrated wave using the left arm with the s0 joint
        	"""
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 		
		
		limb = baxter_interface.Limb('left')
        	limb.set_joint_position_speed(0.7)
		
        	wave_1 = {'left_s0': 0.0, 'left_s1': -1.3, 'left_e0': 0.0, 'left_e1': 0.0, 
				'left_w0': -1.8, 'left_w1': 1.2, 'left_w2': -1.8}
        	wave_2 = {'left_s0': 0.0, 'left_s1': -1.3, 'left_e0': 0.0, 'left_e1': 0.0, 
				'left_w0': -1.8, 'left_w1':0, 'left_w2': -1.8}
		
        	for _move in range(3):
            		limb.move_to_joint_positions(wave_1, timeout=1.5, threshold=0.008)
            		limb.move_to_joint_positions(wave_2,timeout=1.5, threshold=0.008)
        
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 
   

	def comehere(self):
        
	        """
        	Using its arm performs a gesture callng 
        	"""
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

        	limb = baxter_interface.Limb('left')
        	limb.set_joint_position_speed(0.6)
		
        	wave_1 = {'left_s0': -1, 'left_s1': 0.0, 'left_e0': -2.8, 'left_e1': 0.0, 
				'left_w0': 0.0, 'left_w1': 1.5, 'left_w2': 0.0}
        	wave_2 = {'left_s0': -1, 'left_s1': 0.0, 'left_e0': -2.8, 'left_e1': 0.0, 
				'left_w0': 0.0, 'left_w1': 00, 'left_w2': 0.0}
		
        	for _move in range(3):
            		limb.move_to_joint_positions(wave_1,timeout=1.5, threshold=0.005)
            		limb.move_to_joint_positions(wave_2,timeout=1.5, threshold=0.005)
        
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 


    	def firstbump(self):

		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

        	limb = baxter_interface.Limb('right')
		
        	angles = limb.joint_angles()
        	angles['right_s0']=1.0
        	angles['right_s1']=-0.2
        	angles['right_e0']=0.0
        	angles['right_e1']=0.0
        	angles['right_w0']=0.0
        	angles['right_w1']=0.0
        	angles['right_w2']=0.0
        	print(angles)
        	limb.move_to_joint_positions(angles)
		time.sleep(1)

		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

    	def shakehands(self):

		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 
        	limb = baxter_interface.Limb('right')
		limb.set_joint_position_speed(0.7)
        	angles = limb.joint_angles()

        	angles['right_s0']=1.0
        	angles['right_s1']=0.0
        	angles['right_e0']=1.6
        	angles['right_e1']=0.0
        	angles['right_w0']=0.0
        	angles['right_w1']=0.0
        	angles['right_w2']=0.0
        	print(angles)    
	        limb.move_to_joint_positions(angles)
	        time.sleep(1)
		#os.system("/home/edwin/ros_ws/utuck") 
		os.system("/home/par/par_ws/utuck")

        
	
	def highfive(self):	 
	        """
	        Performs a high five
	        """
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 

	        limb = baxter_interface.Limb('left')
		limb.set_joint_position_speed(0.8)

	        angles = limb.joint_angles()
	        angles['left_s0']=-1
	        angles['left_s1']=-0.9
	        angles['left_e0']=0
	        angles['left_e1']=1.0
	        angles['left_w0']=0.0
	        angles['left_w1']=-1.5
	        angles['left_w2']=0.0
	        print(angles)
        
        
	        vel_0 = {'left_s0': 1, 'left_s1': 1, 'left_e0': 1, 'left_e1': 1, 
		'left_w0': 1, 'left_w1': 1, 'left_w2': 1}
	        #limb.set_joint_velocities(vel_0);
	        cur_vel = limb.joint_velocities()
	        limb.set_joint_torques(vel_0)
	        print(cur_vel)
	        
	        limb.move_to_joint_positions(angles)
	        time.sleep(1)
	        
		os.system("/home/par/par_ws/utuck") 
		#os.system("/home/edwin/ros_ws/utuck") 
        

	def set_neutral(self):
	        """
	        Sets both arms back into a neutral pose.
	        """
	        print("Moving to neutral pose...")
	        self._left_arm.move_to_neutral()
	        self._right_arm.move_to_neutral()    

def debug(str):
	print (str)

def speech_callback(data):
    	debug(str(data.data))
    	print (data.data)
    	if data.data=="wave":
        	baxter_wave = Gesture()
        	baxter_wave.wave()
    	if data.data=="big-wave":
		debug("waving...")
         	baxter_wave = Gesture()
        	baxter_wave.bigwave()
    	if data.data=="shake":
        	baxter_wave = Gesture()
        	baxter_wave.shakehands()
    	if data.data=="come-here":
        	baxter_wave = Gesture()
        	baxter_wave.comehere()
    	if data.data=="fist-bump":
        	baxter_wave = Gesture()
        	baxter_wave.firstbump()
    	if data.data=="high-five":
        	baxter_wave = Gesture()
        	baxter_wave.highfive()
    	if data.data=="neutral":
        	baxter_wave = Gesture()
        	baxter_wave.set_neutral()


def main():
    	"""RSDK Baxter Wave

    	Gesture Script
    	This script contains all the gestures that baxter can perform such as
    	Wave
    	Shake Hands
    	Big Wave
    	Come here
	high five
	"""
    	print("Initializing node... ")
    	rospy.init_node("baxter_gesture")
    	baxter_wave = Gesture()
    	rospy.on_shutdown(baxter_wave.clean_shutdown)
    	rospy.Subscriber("/speechPocketsphinx/speech_recognition", String, speech_callback)
    	rospy.spin()


if __name__ == '__main__':
	main()
