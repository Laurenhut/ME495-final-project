#!/usr/bin/env python
import rospy
import sys
import numpy as np
import baxter_interface
from geometry_msgs.msg import (
    Transform,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Bool,
    Header,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from me495finaltest.srv import *


#To Do:
# 1.Replace stand in dTke with actuall desired offset based on actual frame. It's kind of in world coords right now? Don't know what I'm actually doing.
# 2.Replace dQke with a composition of quaternion rotations from base to keurig to desired end effector


def request_handler(req):
    (Qk,pk) = posetonp(req)  #Horizontal np.array([[w, x, y, z]]), vertical np.array([[x],[y],[z]])


    #Set variables
    x1 = -.25
    x2 = -.09
    x3 = -.105
    x4 = -.25
    y1 = 0
    y2 = 0
    y3 = 0
    y4 = 0
    z1 = .0516
    z2 = .0516
    z3 = -.0573
    z4 = -.0573
    ikreq = SolvePositionIKRequest()
    lefthand = baxter_interface.Gripper('left')
    left = baxter_interface.Limb('left')
    # dQke1 = 1 #compose Qbk with Qke to get the desired Qbe
    # dTke1 = np.array([[0,1,0,x1],[0,0,-1,y1],[-1.,0,0,z1],[0,0,0,1.]])
    # dQke2 = 1
    # dTke2 = np.array([[0,1,0,x2],[0,0,-1,y2],[-1.,0,0,z2],[0,0,0,1.]])
    # dQke3 = 1
    # dTke3 = np.array([[0,1,0,x3],[0,0,-1,y3],[-1.,0,0,z3],[0,0,0,1.]])
    # dQke4 = 1
    # dTke4 = np.array([[0,1,0,x4],[0,0,-1,y4],[-1.,0,0,z4],[0,0,0,1.]])
    #Changeme the placeholder rotations are incorrect
    dQke1 = 1
    dTke1 = np.array([[0,.7071,-.7071,x1],[0,-.7071,-.7071,y1],[-1.,0,0,z1],[0,0,0,1.]])
    dQke2 = 1
    dTke2 = np.array([[0,.7071,-.7071,x2],[0,-.7071,-.7071,y2],[-1.,0,0,z2],[0,0,0,1]])
    dQke3 = 1
    dTke3 = np.array([[0,.7071,-.7071,x3],[0,-.7071,-.7071,y3],[-1.,0,0,z3],[0,0,0,1]])
    dQke4 = 1
    dTke4 = np.array([[0,.7071,-.7071,x4],[0,-.7071,-.7071,y4],[-1.,0,0,z4],[0,0,0,1]])

    #Quaternion multiplication does represent composition of rotations.
    #So if I translate a desired R for the EE in the keurig frame into a quaternion (or just figure out 180 about world z in quaternion?) then I should be able to multiply the keurig quaternion by the R quaternion to get a world quaternion for the end effector.


    #Create list of poses to solve for
    dpose = poser(Qk,pk,dQke1,dTke1)
    print dpose
    ikreq.pose_stamp.append(dpose)

    dpose = poser(Qk,pk,dQke2,dTke2)
    print dpose
    ikreq.pose_stamp.append(dpose)

    dpose = poser(Qk,pk,dQke3,dTke3)
    print dpose
    ikreq.pose_stamp.append(dpose)

    dpose = poser(Qk,pk,dQke4,dTke4)
    print dpose
    ikreq.pose_stamp.append(dpose)
    #print ikreq

    #Send request message to ikcaller
    sol_list = ikcaller(ikreq)
    # print sol_list[0]
    # print sol_list[1]
    # print sol_list[2]
    # print sol_list[3]






    #Will need to index sol_list. It's a list of dictionaries
    left.move_to_joint_positions(sol_list[0])
    rospy.sleep(5)
    #Open the gripper
    lefthand.open()
    left.move_to_joint_positions(sol_list[1])
    rospy.sleep(1)
    #Close the gripper
    lefthand.close()
    rospy.sleep(1)
    left.move_to_joint_positions(sol_list[2])
    rospy.sleep(1)
    lefthand.open()
    rospy.sleep(1)
    left.move_to_joint_positions(sol_list[3])

    return Open2Response(True)


def ikcaller(poselist):
    #This is using element zero, which may mean its designed for one joint solution solving.
    iksvc = rospy.ServiceProxy("ExternalTools/left/PositionKinematicsNode/IKService", SolvePositionIK)

    try:
        rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
        resp = iksvc(poselist)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    #Be careful to respect the feasible movement limits
    if (resp.isValid[0]):
        print resp.isValid
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = []
        for i in range(0,4):
            limb_joints.append(dict(zip(resp.joints[i].name, resp.joints[i].position)))
        #print limb_joints
        return limb_joints
    else:
        print resp.isValid
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 1




def poser(Qk,pk,dQke,dTke):
    #Translate pk to dpw
    Twk = np.array([[1,0,0,pk[0,0]],[0,1,0,pk[1,0]],[0,0,1,pk[2,0]],[0,0,0,1]])
    dpw = Twk.dot(dTke.dot(np.array([[0],[0],[0],[1]])))


    #To start, I will just collect dQe from data.
    #later, Qwk*Qke=Qwe
    #Once I have any desired end effector orientation, I can rotate that about world z to get the one I want for a keurig rotation
    dQe = np.array([[0.50833,-0.48533,0.49162,-0.51417]]) #w, x, y, z

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    dpose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=dpw[0,0],
                    y=dpw[1,0],
                    z=dpw[2,0],
                ),
                orientation=Quaternion(
                    x=dQe[0,1],
                    y=dQe[0,2],
                    z=dQe[0,3],
                    w=dQe[0,0],
                ),
            ),
        ),

    return dpose[0]



def posetonp(msg):
    #Given a Pose message for the keurig, convert to numpy arrays p and Q.
    Qk = np.array([[msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]])
    pk = np.array([[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]]).T
    return (Qk,pk)



#Changeme node name is a placeholder
def open_server():
    rospy.init_node('closer_node')
    svc = rospy.Service('closer', Open2, request_handler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



if __name__ == '__main__':
    open_server()
