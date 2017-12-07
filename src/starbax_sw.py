#!/usr/bin/env python
import rospy
from math import pi
import baxter_interface

# def main():
rospy.init_node('initial')
# Define your image topic

limb = baxter_interface.Limb('left')
angles = limb.joint_angles()

angles['left_s0']=0
angles['left_s1']=-pi/4
angles['left_e0']=0.0
angles['left_e1']=pi/4
angles['left_w0']=0.0
angles['left_w1']=90*pi/180
angles['left_w2']=pi

limb.move_to_joint_positions(angles)

angles['left_s0']=-pi/4
angles['left_s1']=-pi/4
angles['left_e0']=0.0
angles['left_e1']=pi/6
angles['left_w0']=0.0
angles['left_w1']=100*pi/180
angles['left_w2']=pi

limb.move_to_joint_positions(angles)


# if __name__ == '__main__':
# 	main()