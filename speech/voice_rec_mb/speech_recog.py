#! /usr/bin/env python

import rospy
import sys, os
import thread
from subprocess import call

global ps, ibm

def pocketsphinx_recognise():
    os.chdir("/home/mb/baxter_speech/speech/src/voice_recognition/scripts")
    with open("test.txt", "w" ) as outfile:
        devnull = open(os.devnull, 'w')
        call(["../src/hello_ps"], stdout=outfile, stderr=devnull)
        ps = True

# def ibm_recognise():


if __name__ == "__main__":
    # ps = False
    # ibm = False

    thread.start_new_thread(pocketsphinx_recognise, ())

    # while not ps and not ibm:
    #     # print ps, ibm
    #     pass
