#!/usr/bin/python

import math
import time
import Vectors
import struct
import threading
import Queue
import rospy
import roslib
from std_msgs.msg import String
import tf
from tf import TransformListener

# for Euler
from tf.transformations import *

from trac_ik_python.trac_ik import IK

import random

import tf2_ros
import tf2_geometry_msgs

import geometry_msgs.msg

# refer to http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        )
import baxter_interface

from baxter_interface import CHECK_VERSION

from threading import Thread
import sys

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Get your URDF from somewhere
urdf_str = rospy.get_param('/robot_description')

global limbRight
global limbLeft
limbRight = 'right'
limbLeft = 'left'

def translate_pose_in_own_frame(ps,localname,dx,dy,dz):
	m = geometry_msgs.msg.TransformStamped()
	m.child_frame_id = localname
	m.header.frame_id = ps.header.frame_id
	m.header.stamp = rospy.Time.now()
	m.transform.translation.x=ps.pose.position.x
	m.transform.translation.y=ps.pose.position.y
	m.transform.translation.z=ps.pose.position.z
	m.transform.rotation.x=ps.pose.orientation.x
	m.transform.rotation.y=ps.pose.orientation.y
	m.transform.rotation.z=ps.pose.orientation.z
	m.transform.rotation.w=ps.pose.orientation.w
	tf_buffer.set_transform(m,"")
	return translate_in_frame(ps,localname,dx,dy,dz)

# translate PoseStamped ps with respect to a given frame
def translate_in_frame(ps,frame,dx,dy,dz):
    ps_f = translate_frame(ps,frame)
    result = PoseStamped(
	header=ps_f.header,
	pose=Pose(
            position=Point(
                x=ps_f.pose.position.x + dx,
                y=ps_f.pose.position.y + dy,
                z=ps_f.pose.position.z + dz
                ),
            orientation=ps_f.pose.orientation
    ))
    return result

# give PoseStamped ps in frame-of-refernce frame
def translate_frame(ps,frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(frame,
      ps.header.frame_id, #source frame
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    #print 'pose in baxter frame',pose_transformed,' ', frame_to
    return pose_transformed

def trac_ik_solve(limb, ps):
	try:
		arm = baxter_interface.Limb(limb)
		state = arm.joint_angles()
		print 'solve current:',arm.joint_angles()
		jointnames = ['s0','s1','e0','e1','w0','w1','w2']
		command = []
		for i in range(0, len(jointnames)):
		    key = limb+"_"+jointnames[i]
		    command.append(state[key])
		print 'candidate seed',command
		local_base_frame = limb+"_arm_mount"
		ik_solver = IK(local_base_frame,
			       limb+"_wrist",
			       #limb+"_gripper",
			       urdf_string=urdf_str)
		#seed_state = [0.0] * ik_solver.number_of_joints
		seed_state = command
		# canonical pose in local_base_frame
		#hdr = Header(stamp=rospy.Time.now(), frame_id=from_frame)
		#ps = PoseStamped(
		#        header=hdr,
		#        pose=pose,
		#        )
		#gripper_ps = translate_in_frame(ps,'right_wrist',0,0,0)
		#gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0.015,-0.02,-0.2)
		#gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0,0,0.05)
		p = translate_frame(ps,local_base_frame)
		#print 'translated frame',p
		soln = ik_solver.get_ik(seed_state,
				p.pose.position.x,p.pose.position.y,p.pose.position.z,  # X, Y, Z
				p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,  # QX, QY, QZ, QW
				0.01,0.01,0.01,
				0.1,0.1,0.1,

		)
		print 'trac soln',soln
		return soln
	except:
		return None

def solve_move_trac(limb,ps):
	print '*********************'
	print 'solve_move_trac',limb,ps
	soln = trac_ik_solve(limb,ps)
	if not soln:
		print '***** NO SOLUTION ******',soln
	else:
		print 'soln',soln
		make_move_trac(soln, limb, 0.8)

# msg: 7-vector of positions corresponding to joints
# limb: 'left'|'right'
def make_move_trac(msg, limb, speed):
	#print 'make_move_trac',msg
	SPEED_SCALE = 1
	speed = speed * SPEED_SCALE
	arm = baxter_interface.Limb(limb)
	#print 'arm',arm
	lj = arm.joint_names()
	command = {}
	# for i in range(0, len(msg.joints[0].name)):
	jointnames = ['s0','s1','e0','e1','w0','w1','w2']
	for i in range(0, len(jointnames)):
		command[limb+"_"+jointnames[i]] = msg[i]
		#print 'current:',arm.joint_angles()
	#print 'make_move: speed',speed
	arm.set_joint_position_speed(speed)
	#print 'make_move_trac: posns',command
	arm.set_joint_positions(command)
	#arm.move_to_joint_positions(command, 0.5, 0.01)
	#print 'make_move: done'

def pose2(pos):
    q_rot = quaternion_from_euler(0.0, math.pi/2, 0.0)
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            #orientation=Quaternion(
            #    x=0,
            #    y=1,
            #    z=0,
            #    w=0
            #    ),
            orientation=normalize(Quaternion(
		x=q_rot[0],
		y=q_rot[1],
		z=q_rot[2],
		w=q_rot[3]
            ))
            )
    return pose_right

def userPose(pos, rot):
    q_rot = quaternion_from_euler(0.0, rot.y(), 0.0)
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            #orientation=Quaternion(
            #    x=0,
            #    y=1,
            #    z=0,
            #    w=0
            #    ),
            orientation=normalize(Quaternion(
		x=q_rot[0],
		y=q_rot[1],
		z=q_rot[2],
		w=q_rot[3]
            ))
            )
    return pose_right

#NOTE: right hand
#index finger point left
init_pos = Vectors.V4D(0.6, #depth (z?) (inline depth 0.2) (limit 1.0)
        -0.60, #left-right (x?) (inline 0.28) (limit 1.0)
        0.40, 0) #height (y?) (inline height 0.40)

# Quaternion -> Quaternion
def normalize(quat):
 	quatNorm = math.sqrt(quat.x * quat.x + quat.y *
                        quat.y + quat.z * quat.z + quat.w * quat.w)
        normQuat = Quaternion(quat.x / quatNorm,
                              quat.y / quatNorm,
                              quat.z / quatNorm,
                              quat.w / quatNorm)
	return normQuat

def systemArgHandler():
	if len(sys.argv) == 2:
		tempID = sys.argv[1]
		if tempID.isdigit():
			print '\033[92m' + "User ID is set to " + tempID
			return tempID
		else:
			print '\033[93m' + "User ID must be an integer"
			print '\033[93m' + "User ID is set to default (1)"
			return 1
	else:
		print "User ID is set to default (1)"
		return 1

def main():
	userID = systemArgHandler()

	global limbRight
	global limbLeft

	rospy.init_node('lift_ik_prototype', anonymous=True)

	"""
	myps = PoseStamped(
		header=Header(stamp=rospy.Time.now(), frame_id='base'),
		pose=pose2(init_pos),
	)
	solve_move_trac(limbRight, myps)
	"""

	rate = rospy.Rate(5000.0)
	#main loop
        while not rospy.is_shutdown():
                try:
			#defines the pose of the child from the parent
                        buffUserLeft = tf_buffer.lookup_transform('cob_body_tracker/user_'+str(userID)+'/left_shoulder', 'cob_body_tracker/user_'+str(userID)+'/left_hand', rospy.Time())
                        buffUserRight = tf_buffer.lookup_transform('cob_body_tracker/user_'+str(userID)+'/right_shoulder', 'cob_body_tracker/user_'+str(userID)+'/right_hand', rospy.Time())
			#get translations
                        xL = buffUserLeft.transform.translation.x
                        yL = buffUserLeft.transform.translation.y
                        zL = buffUserLeft.transform.translation.z
                        xR = buffUserRight.transform.translation.x
                        yR = buffUserRight.transform.translation.y
                        zR = buffUserRight.transform.translation.z

			#translate the translations
			#add offsets to each joint
			tranXL = -zL * 1.2 + 0.2
			tranYL =  xL * 1.5 - 0.28
			tranZL = -yL * 1.1 + 0.40

			tranXR = -zR * 1.2 + 0.2
			tranYR =  xR * 1.5 + 0.28
			tranZR = -yR * 1.1 + 0.40

			#tranRightX = -z * 1.2
			#tranRightY =  x * 1.2
			#tranRightZ = -y * 0.9

		#catch and continue after refresh rate
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue

		user_pos_left = Vectors.V4D(tranXL, #depth (z?) (inline depth 0.2) (limit 1.0)
			tranYL, #left-right (x?) (inline 0.28) (limit 1.0)
			tranZL, 0) #height (y?) (inline height 0.40)

		user_pos_right = Vectors.V4D(tranXR, #depth (z?) (inline depth 0.2) (limit 1.0)
			tranYR, #left-right (x?) (inline 0.28) (limit 1.0)
			tranZR, 0) #height (y?) (inline height 0.40)

		#user_rot = Vectors.V4D(math.pi/2,
		#	((math.atan2(y,x) + math.pi/2) % (math.pi * 2)) - 10*math.pi/180,
		#	0, 0)

		# rotation pointing the infront of Red
		user_rot = Vectors.V4D(0,
			math.pi/2,
			0, 0)

		rightArmPose = PoseStamped(
			header=Header(stamp=rospy.Time.now(), frame_id='base'),
			#header=Header(stamp=rospy.Time.now(), frame_id=limbRight+'_arm_mount'),
			#header=Header(stamp=rospy.Time.now(), frame_id=limbRight+'_lower_shoulder'),
			pose=userPose(user_pos_left, user_rot),
		)

		leftArmPose = PoseStamped(
			header=Header(stamp=rospy.Time.now(), frame_id='base'),
			#header=Header(stamp=rospy.Time.now(), frame_id=limbRight+'_arm_mount'),
			#header=Header(stamp=rospy.Time.now(), frame_id=limbRight+'_lower_shoulder'),
			pose=userPose(user_pos_right, user_rot),
		)

		#solve_move_trac(limbRight, leftArmPose)
		solve_move_trac(limbLeft, leftArmPose)

		#Refresh rate
                rate.sleep()

if __name__ == '__main__':
        main()
