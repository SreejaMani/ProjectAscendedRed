#! /usr/bin/env python

import rospy
from baxter_core_msgs.msg import DigitalIOState, HeadPanCommand
from unite_ws.msg import request
import argparse
import sys, copy

import baxter_interface

from baxter_interface import CHECK_VERSION

selfie = []
tuck = []

tucked = None
pressed_selfie = False
pressed_tuck = False

head = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, latch=True, queue_size=1)
face = rospy.Publisher('/robot/digital_io/torso_right_button_ok/state', DigitalIOState, latch=True, queue_size=1)


def pose_selfie(data):
    global tucked
    global pressed_selfie
    global selfie
    if data.state and tucked and not pressed_selfie:

        head.publish(0.6,0.05,0)
        map_file(selfie, args.loops)
        face.publish(1, True)

        tucked = False
    pressed_selfie = data.state

def pose_return(data):
    global tucked
    global pressed_tuck
    global tuck
    if data.state and not tucked and not pressed_tuck:

        head.publish(0.0,0.05,0)
        map_file(tuck, args.loops)

        tucked = True
    pressed_tuck = data.state


def listener():
    rospy.init_node("selfie_pose")
    rospy.Subscriber("/robot/digital_io/torso_right_button_show/state", DigitalIOState, pose_return)
    rospy.Subscriber("/robot/digital_io/torso_right_button_back/state", DigitalIOState, pose_selfie)
    rospy.spin()


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def clean_line(line, names):
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    left_command = dict((key, command[key]) for key in command.keys()
                        if key[:-2] == 'left_')
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'right_')
    return (command, left_command, right_command, line)


def map_file(lines, loops=1):

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    rate = rospy.Rate(1000)

    l = 0
    # If specified, repeat the file playback 'loops' number of times
    while loops < 1 or l < loops:
        i = 0
        l += 1
        print("Moving to start position...")

        firstline = lines[1]
        _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)
        start_time = rospy.get_time()
        for values in lines[1:]:
            i += 1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                             (i, len(lines) - 1, l, loopstr))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
                if len(lcmd):
                    left.set_joint_positions(lcmd)
                if len(rcmd):
                    right.set_joint_positions(rcmd)
                rate.sleep()
        print
    return True

# reverse lines (excluding header)
# swap field 1 at line i with field 1 at n-i
def reverse(lines):
    new = copy.deepcopy(lines)
    header = new[0]
    rest = new[1:]
    rest.reverse()
    for i in range(0,len(rest)/2):
        asplit = rest[i].split(',')
        ta = asplit[0]
        resta = asplit[1:]
        bsplit = rest[len(rest)-i-1].split(',')
        tb = bsplit[0]
        restb = bsplit[1:]
        rest[i]           =tb+','+(','.join(resta))
        rest[len(rest)-i-1]=ta+','+(','.join(restb))
    return [header]+rest

if __name__ == '__main__':

    epilog = """
Related examples:
  joint_recorder.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=None,
                                     epilog=epilog)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of times to loop the input file. 0=infinite.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    with open(args.file, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')

 
    selfie = lines
    tuck = reverse(selfie)

    listener()
