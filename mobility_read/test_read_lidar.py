#!/usr/bin/env python

#Do I add this in? I think I have to for copyright...

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
import random
import math
import time
import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

globvar = 0
timer =0
start_time = time.time()
class DetectIndividual(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = baxter_interface.Head()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self._done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)
		
    def set_head(self,data):
		#self._head.set_pan(data)
        self._head.set_pan(data, speed=0.3, timeout=3)

#this runs for sure
def angle_callback(data):
    global start_time
    angleData = data.data
    avgAnglesEdited = data.data
    #print str(["{0:3.1f}".format(item) for item in avgAnglesEdited]).replace("'", "")
    wobbler = DetectIndividual()
    #elapsed_time = time.time() - start_time
    #print elapsed_time
    global globvar  
    try:
        closestAngle = angleData[closestDistID]
        radian = math.radians(closestAngle);
        radianFormatted = float("{0:.3f}".format(radian))
        wobbler.set_head(radianFormatted-3.14)
        '''
        print average
        print radianFormatted
        print radianFormatted-3.14
        print counter
        '''
        print data.data
        print "Does this make sense? " + str(closestAngle) + "\n"
    except:
        print "Error!"

#prototype code of discovering closest target
#get obj number
def dist_callback(data):
    global closestDistID 
    ctr = 0
    distData = data.data
    minDist = min(distData)
    for dist in distData:
        if (dist == minDist):
            closestDistID = ctr
            break
        ctr += 1
            

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
        description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    # Test the input lidar code before the head wobbler
    print("Initializing node... ")
    rospy.init_node("crowdvsIndividualMode_detection", anonymous=True)
    wobbler =DetectIndividual();
    # subscribes to publisher generated from lidar_detect_crowd.py
    rospy.Subscriber("/input/lidar/distances",Float32MultiArray,dist_callback)
    rospy.Subscriber("/input/lidar/angles",Float32MultiArray,angle_callback)
    time.sleep(1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
