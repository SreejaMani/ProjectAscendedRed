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

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy
import math
import time
import baxter_interface
import baxter_external_devices
from std_msgs.msg import Float32MultiArray

from baxter_interface import CHECK_VERSION
#from std_msgs.msg import Float32
import random
from std_msgs.msg import (
    UInt16,Float32,
)

class Wave(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()
        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        #self._pub_rate.publish(self._rate)

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
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def set_head(self,data):
		self._head.set_pan(data)

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def set_arm(self,radian):
        global flag
        global radian_pos
        if (flag == False):
            if(radian < 3.14):
                #self._left_arm.move_to_neutral()
                limb = baxter_interface.Limb('right')
                angles = limb.joint_angles()
                print radian-2.4
                radian_pos=radian-2.4
                angles['right_s0']=radian-2.4
                angles['right_s1']=0.0
                angles['right_e0']=0.0
                angles['right_e1']=0.0
                angles['right_w0']=0.0
                angles['right_w1']=0.0
                angles['right_w2']=0.0
                limb.move_to_joint_positions(angles)
            else:
                #self._right_arm.move_to_neutral()
                limb = baxter_interface.Limb('left')
                angles = limb.joint_angles()
                print radian-3.9
                radian_pos=radian-3.9
                angles['left_s0']=radian-3.9
                angles['left_s1']=0.0
                angles['left_e0']=0.0
                angles['left_e1']=0.0
                angles['left_w0']=0.0
                angles['left_w1']=0.0
                angles['left_w2']=0.0
                limb.move_to_joint_positions(angles)

    def wave(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
	limb = baxter_interface.Limb('right')
	angles = limb.joint_angles()
	#angles['right_s0']=1.0
	angles['right_s1']=0.0
	angles['right_e0']=0.0
	angles['right_e1']=0.0
	angles['right_w0']=0.0
	angles['right_w1']=0.0
	angles['right_w2']=0.0
	print(angles)
	limb.move_to_joint_positions(angles)
	wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
	wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}
		
	for _move in range(3):
		limb.move_to_joint_positions(wave_1)
		limb.move_to_joint_positions(wave_2)

    def bringBack(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
	limb = baxter_interface.Limb('right')
	angles = limb.joint_angles()
	#angles['right_s0']=1.0
	angles['right_s1']=0.0
	angles['right_e0']=0.0
	angles['right_e1']=0.0
	angles['right_w0']=0.0
	angles['right_w1']=0.0
	angles['right_w2']=0.0
	print(angles)
	limb.move_to_joint_positions(angles)
	wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
	wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}
		
	for _move in range(3):
		limb.move_to_joint_positions(wave_1)
		limb.move_to_joint_positions(wave_2)  


		
def callback(data):
    wave =Wave()
    radian = math.radians(data.data);
    radianFormatted = float("{0:.3f}".format(radian))
    wave.set_arm(radianFormatted);
    #time.sleep(2)

def dist_callback(data):
    distance = data.data
    distFormatted = float("{0:.3f}".format(distance))
    if(distFormatted<200):
        print distFormatted
        flag=True

        wav  = Wave()
        wav.bringBack()
        flag=False
        #time.sleep(2)

def angle_crowd_callback(data):
    avgAnglesEdited = data.data
    #print str(["{0:3.1f}".format(item) for item in avgAnglesEdited]).replace("'", "")
    
    for item in avgAnglesEdited:
        distFormatted = float("{0:.3f}".format(item))
        wave =Wave()
        wave.set_head(distFormatted-3.14)


flag= False
radian_pos =0

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])	
    print("Initializing node... ")
    rospy.init_node("rsdk_gesture_manipulator")
	
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()
    #test()
    #rospy.Subscriber("/input/lidar/angleSlow",Float32,callback)
    #rospy.Subscriber("/input/lidar/distanceSlow",Float32,dist_callback)
    rospy.Subscriber("/input/lidar/angleCrowd",Float32MultiArray,angle_crowd_callback)



    rospy.spin()
    print("Done.")

if __name__ == '__main__':
    main()
