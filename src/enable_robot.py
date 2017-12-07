#!/usr/bin/python2

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
import os
import sys

import rospy
import cv2
import cv_bridge
import baxter_interface

from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import (
    Image,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--state', const='state',
                        dest='actions', action='append_const',
                        help='Print current robot state')
    parser.add_argument('-e', '--enable', const='enable',
                        dest='actions', action='append_const',
                        help='Enable the robot')
    parser.add_argument('-d', '--disable', const='disable',
                        dest='actions', action='append_const',
                        help='Disable the robot')
    parser.add_argument('-r', '--reset', const='reset',
                        dest='actions', action='append_const',
                        help='Reset the robot')
    parser.add_argument('-S', '--stop', const='stop',
                        dest='actions', action='append_const',
                        help='Stop the robot')
    args = parser.parse_args(rospy.myargv()[1:])

    if args.actions == None:
        parser.print_usage()
        parser.exit(0, "No action defined")

    rospy.init_node('rsdk_robot_enable')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)

    try:
        for act in args.actions:
            if act == 'state':
                print rs.state()
            elif act == 'enable':
                rs.enable()

                #declare joint dictionaries
                raise_r = {'right_s0': -1.0, 'right_s1': -1.0, 'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': -0.0, 'right_w1': -0.0, 'right_w2': -0.0}
                raise_l = {'left_s0': 1.0, 'left_s1': -1.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0}
                rest_r = {'right_s0': 0.0, 'right_s1': 0.5, 'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': -0.0, 'right_w1': -0.0, 'right_w2': -0.0}
                rest_l = {'left_s0': 0.0, 'left_s1': 0.5, 'left_e0': 0.0, 'left_e1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0}


                #Command Joints
                baxter_interface.Limb('left').move_to_joint_positions(rest_l)
                baxter_interface.Limb('right').move_to_joint_positions(rest_r)
                rospy.sleep(3)

                #Command joints
                # baxter_interface.Limb('left').move_to_joint_positions(raise_l)
                # baxter_interface.Limb('right').move_to_joint_positions(raise_r)
                #Set Image
                # img = cv2.imread('media/TheSun2.png')
                # msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
                # pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
                # # pub.publish(msg)
                # rospy.sleep(3)

                #Command joints
                # baxter_interface.Limb('left').move_to_joint_positions(rest_l)
                # baxter_interface.Limb('right').move_to_joint_positions(rest_r)
                # Set Image
                # img = cv2.imread('media/researchsdk.png')
                # msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
                # pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
                # pub.publish(msg)

            elif act == 'disable':
                rs.disable()
            elif act == 'reset':
                rs.reset()
            elif act == 'stop':
                rs.stop()
    except Exception, e:
        rospy.logerr(e.strerror)

    return 0

if __name__ == '__main__':
    sys.exit(main())
