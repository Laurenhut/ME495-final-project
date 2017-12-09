#!/usr/bin/env python
import argparse
import struct
import sys
import copy

import rospy
import rospkg
from starbax.msg import ar_tagstr
from starbax.srv import *

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import (
	Header,
	Empty,
)

from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
	def __init__(self, limb, hover_distance = 0.15, verbose=True):
		self._limb_name = limb # string
		self._hover_distance = hover_distance # in meters
		self._verbose = verbose # bool
		self._limb = baxter_interface.Limb(limb)
		self._gripper = baxter_interface.Gripper(limb)
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		rospy.wait_for_service(ns, 5.0)
		# verify robot is enabled
		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()



	def move_to_start(self, start_angles=None):
		print("Moving the {0} arm to start pose...".format(self._limb_name))
		if not start_angles:
			start_angles = dict(zip(self._joint_names, [0]*7))
		self._guarded_move_to_joint_position(start_angles)
		self.gripper_open()
		rospy.sleep(1.0)
		print("Running. Ctrl-c to quit")

	def ik_request(self, pose):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
		try:
			resp = self._iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False
		# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					   }.get(resp_seeds[0], 'None')
			if self._verbose:
				print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
						 (seed_str)))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			if self._verbose:
				print("IK Joint Solution:\n{0}".format(limb_joints))
				print("------------------")
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			return False
		return limb_joints

	def _guarded_move_to_joint_position(self, joint_angles):
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

	def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

	def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

	def _approach(self, pose):
		approach = copy.deepcopy(pose)
		# approach with a pose the hover-distance above the requested pose
		approach.position.x = approach.position.x - self._hover_distance
		joint_angles = self.ik_request(approach)
		self._guarded_move_to_joint_position(joint_angles)

	def _retract(self):
		# retrieve current pose from endpoint
		current_pose = self._limb.endpoint_pose()
		ik_pose = Pose()
		ik_pose.position.z = current_pose['position'].z + 0.08
		ik_pose.position.y = current_pose['position'].y
		ik_pose.position.x = current_pose['position'].x - self._hover_distance
		ik_pose.orientation.x = current_pose['orientation'].x
		ik_pose.orientation.y = current_pose['orientation'].y
		ik_pose.orientation.z = current_pose['orientation'].z
		ik_pose.orientation.w = current_pose['orientation'].w
		joint_angles = self.ik_request(ik_pose)
		# servo up from current pose
		self._guarded_move_to_joint_position(joint_angles)

	def _retract2(self):
		# retrieve current pose from endpoint
		current_pose = self._limb.endpoint_pose()
		ik_pose = Pose()
		ik_pose.position.z = current_pose['position'].z
		ik_pose.position.y = current_pose['position'].y
		ik_pose.position.x = current_pose['position'].x - self._hover_distance
		ik_pose.orientation.x = current_pose['orientation'].x
		ik_pose.orientation.y = current_pose['orientation'].y
		ik_pose.orientation.z = current_pose['orientation'].z
		ik_pose.orientation.w = current_pose['orientation'].w
		joint_angles = self.ik_request(ik_pose)
		# servo up from current pose
		self._guarded_move_to_joint_position(joint_angles)

	def _servo_to_pose(self, pose):
		# servo down to release
		joint_angles = self.ik_request(pose)
		self._guarded_move_to_joint_position(joint_angles)

	def pick(self, pose):
		# open the gripper
		self.gripper_open()
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		# close gripper
		self.gripper_close()
		# retract to clear object
		self._retract()

	def place(self, pose):
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		# open the gripper
		self.gripper_open()
		# retract to clear object
		self._retract2()


class movement:

	def __init__(self):
		self.sub=rospy.Subscriber("pose_and_item", ar_tagstr,self.callback)

		self.xrcm=0
		self.xrk=0
		self.xrc=0

		self.yrk=0
		self.yrc=0
		self.yrcm=0
		self.keurig_pose = Pose()


	def callback(self,data):
		# Wait for the All Clear from emulator startup
		#rospy.wait_for_message("/robot/sim/started", Empty)


		print data


		if data.id== "coffee":
			self.xrcm=data.pose.position.x
			self.yrcm=data.pose.position.y
			self.keurig_pose = data.pose
			print self.xrcm
			print self.yrcm


			print "coffee"


		elif data.id== "cup"  :
			self.xrc=data.pose.position.x
			self.yrc=data.pose.position.y
			print self.xrc
			print self.yrc

			print "cup"

		elif data.id== "kcup":
			self.xrk=data.pose.position.x
			self.yrk=data.pose.position.y
			print self.xrk
			print self.yrk

			print "kcup"



		if  self.xrc !=0 and self.xrcm !=0 and self.xrk !=0 :


			limb = 'left'

			hover_distance = 0.15 # meters
			# Starting Joint angles for left arm
			starting_joint_angles = {'left_w0': 0,'left_w1': -0.7854,'left_w2': 0,'left_e0': 0,'left_e1': 2.356,'left_s0': -0.7854,'left_s1': -0.7854}
			pnp = PickAndPlace(limb, hover_distance)
			# An orientation for gripper fingers to be overhead and parallel to the obj
			overhead_orientation = Quaternion(x=0.0,y=0.7071067811,z=0.0,w=0.7071067811)
			block_poses = list()
			# The Pose of the block in its initial location.
			# You may wish to replace these poses with estimates

			# from a perception node.
			# cup pick up pose (correct)
			block_poses.append(Pose(position=Point(x=self.xrc, y=self.yrc-.15, z=-0.1+.1),orientation=overhead_orientation))
			#cup place pose
			block_poses.append(Pose(position=Point(x=self.xrcm-.07, y=self.yrcm-.07, z=-0.05+.1),orientation=overhead_orientation))

			#k-cup pick up pose
			block_poses.append(Pose(
				position=Point(x=self.xrk, y=self.yrk, z=-0.1+.1),
				orientation=overhead_orientation))
			#k_cup place pose
			block_poses.append(Pose(
				position=Point(x=self.xrcm-.10, y=self.yrcm-.04, z=0.03+.15),
				orientation=overhead_orientation))
			# Move to the desired starting angles
			pnp.move_to_start(starting_joint_angles)
			idx = 0
			while not rospy.is_shutdown():
				print("\nPicking the cup...")
				pnp.pick(block_poses[0])
				print("\nPlacing the cup...")
				#idx = (idx+1) % len(block_poses)
				pnp.place(block_poses[1])
				#move to start
				pnp.move_to_start(starting_joint_angles)


				# call to Ian's service - open the lid
				rospy.wait_for_service('opener')
				try:
					opener = rospy.ServiceProxy('opener',Open2)
					resp1 = opener(self.keurig_pose) #resp1 is a bool indicating success
				except rospy.ServiceException, e:
					print "Open call failed: %s"%e 

				#move to start
				pnp.move_to_start(starting_joint_angles)
				print("\nPicking the k_cup...")
				pnp.pick(block_poses[2])
				print("\nPlacing the k_cup...")
				pnp.place(block_poses[3])

				# call to Ian's service - close the lid
				rospy.wait_for_service('closer')
				try:
					closer = rospy.ServiceProxy('closer',Open2)
					resp2 = closer(self.keurig_pose)
				except rospy.ServiceException, e:
					print "Close call failed: %s"%e


				pnp.move_to_start(starting_joint_angles)
				print("\nPicking the cup...")
				pnp.pick(block_poses[1])
				print("\nPlacing the cup...")
				#idx = (idx+1) % len(block_poses)
				pnp.place(block_poses[0])
				#move to start
				pnp.move_to_start(starting_joint_angles)
				print("\nAll set, Enjoy your coffee...")



				break

			return 0


def main():
	rospy.init_node("pick_and_place")

	ic= movement()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

	

if __name__ == '__main__':
	sys.exit(main())
