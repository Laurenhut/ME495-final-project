#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from starbax.msg import ar_tag
import rospy
import sys
import numpy as np

from geometry_msgs.msg import Twist


class ar_transform:

    def __init__(self):
        self.pub=rospy.Publisher("ar_pose_id", ar_tag)
        self.sub=rospy.Subscriber("ar_pose_marker", AlvarMarkers,self.callback)
    
    
    def callback(self,data):
        tagnumber=len(data.markers)
        for i in range(tagnumber):
        
            ar=ar_tag()
            ar.id=data.markers[i].id
            ar.pose=data.markers[i].pose.pose
            
 
            self.pub.publish(ar)
       
   

def main(args):
	rospy.init_node('ar_info', anonymous=True)
	ic= ar_transform()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	


if __name__ == '__main__':
    main(sys.argv)
   

