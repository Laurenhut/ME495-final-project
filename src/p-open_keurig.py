import rospy
import numpy as np
import baxter_interface
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Transform,
    Vector3,
    Quaternion,
    Posestamped,
    Pose,
    Point,
)
from starbax.srv import Open
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)




def request_handler(req):

    #Request should be a transform(Q,p) message
    #I'm instating a constraint that the keurig can be at any translation, but cannot be rotated. This will allow me to use a fixed quaternion. Later I can make the code to translate it the quaternian and unfix it.


    #Cartesian coords of desired position in keurig frame.
    xd = 0
    yd = 0
    zd = 0
    destinationInK = np.array([[xd], [yd], [zd]])

    #Creates an SE(3) matrix from request message.
    transform = np.array([[1,0,0,req.transform.translation.x],[0,1,0,req.transform.translation.y],[0,0,1,req.transform.translation.z],[0,0,0,1]])


    #Determines world coordinates of desired position in Keurig frame
    destinationInB = transform.dot(destinationInK)

    #Set up posestamped message
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()


    #Establish service proxy for left arm.
    iksvc = rospy.ServiceProxy("ExternalTools/left/PositionKinematicsNode/IKService", SolvePositionIK)



    #Set up Posestamped message
    #Quaternion predetermined? Can't do this for keurig. Need to sidestep it or learn it.
    poses = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=destinationInB[0][0],
                    y=destinationInB[1][0],
                    z=destinationInB[2][0],
                ),
                orientation=Quaternion(
                    x=-0.159821886749,
                    y=0.984127886766,
                    z=-0.00293647198525,
                    w=0.0770755742012,
                ),
            ),
        ),


    ikreq.pose_stamp.append(poses)

    #Call the IK service and give it the desired base frame pose.
    try:
        rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return OpenResponse(False)


    if (resp.isValid[0]):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        #Command motion
        left = baxter_interface.Limb('left')
        left.move_to_joint_positions(limb_joints)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return OpenResponse(False)


    #Set up Posestamped message toward the handle
    #Quaternion predetermined? Can't do this for keurig. Need to sidestep it or learn it.
    poses = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=destinationInB[0][0],
                    y=destinationInB[1][0],
                    z=destinationInB[2][0],
                ),
                orientation=Quaternion(
                    x=-0.159821886749,
                    y=0.984127886766,
                    z=-0.00293647198525,
                    w=0.0770755742012,
                ),
            ),
        ),


    ikreq.pose_stamp.append(poses)

    #Call the IK service and give it the desired base frame pose.
    try:
        rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return OpenResponse(False)


    if (resp.isValid[0]):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        #Command motion
        left = baxter_interface.Limb('left')
        left.move_to_joint_positions(limb_joints)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return OpenResponse(False)


    #Close the gripper
    lefthand = baxter_interface.Gripper('left')
    lefthand.close()


    #Perform open action sequence


    #Open the gripper
    lefthand.open()


    #Set up Posestamped message to move gripper away
    #Quaternion predetermined? Can't do this for keurig. Need to sidestep it or learn it.
    poses = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=destinationInB[0][0],
                    y=destinationInB[1][0],
                    z=destinationInB[2][0],
                ),
                orientation=Quaternion(
                    x=-0.159821886749,
                    y=0.984127886766,
                    z=-0.00293647198525,
                    w=0.0770755742012,
                ),
            ),
        ),


    ikreq.pose_stamp.append(poses)

    #Call the IK service and give it the desired base frame pose.
    try:
        rospy.wait_for_service("ExternalTools/left/PositionKinematicsNode/IKService", 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return OpenResponse(False)


    if (resp.isValid[0]):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        #Command motion
        left = baxter_interface.Limb('left')
        left.move_to_joint_positions(limb_joints)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return OpenResponse(False)



    #publish to all clear topic
    #Confirm success to tell Baxter to continue
    return OpenResponse(True)




def open_server():
    rospy.init_node('keurig_open')
    svc = rospy.Service('open_keurig', Open, request_handler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



if __name__ == '__main__':
    open_server()
