#!/usr/bin/env python
import rospy
from starbax.msg import ar_tagstr
from starbax.msg import ar_tag
from std_msgs.msg import String
import numpy as np

class glue:

	def __init__(self):

		self.pose_items = rospy.Publisher("pose_and_item",ar_tagstr,queue_size = 10)
	 	self.sbid = rospy.Subscriber("pos_items",String,self.id_items)
		self.ar = rospy.Subscriber("ar_pose_id",ar_tag,self.ar_info)
		self.list_obj = []
		self.id_list = []
		self.pose_list = []
		
	

	def id_items(self,strid):
		
		self.list_obj = strid.data.split(' ')
		


	
		

	def ar_info(self,ar_msg):
	    
	    
		if len(self.id_list) < 3 and self.list_obj is not None:
			self.id_list.append(ar_msg.id)
			self.pose_list.append(ar_msg.pose)
			print len(self.pose_list),len(self.pose_list)

		if len(self.id_list) == 3 and (self.id_list[0] != self.id_list[1]) and (self.id_list[0] != self.id_list[2]) and (self.id_list[1] != self.id_list[2]):
			ar = ar_tagstr()
	    	
	    	y0 = self.pose_list[0].position.y
	    	y1 = self.pose_list[1].position.y
	    	y2 = self.pose_list[2].position.y
	    	print self.pose_list[2]
	    	pos = np.argsort([y0,y1,y2])
	    	pos = list(pos)

	    	ind0 = pos.index(0)
	    	ind1 = pos.index(1)
	    	ind2 = pos.index(2)

	    	 
	    	ar.id=self.list_obj[2]
	    	
	    	ar.pose=self.pose_list[2]
	    	print ar
	    	#self.pose_items.publish(ar)

	    	# ar.id[ind0] = self.id_list[0]
	    	# ar.pose[ind0] = self.pose_list[0]
	    	# ar.id[ind1] = self.id_list[1]
	    	# ar.pose[ind1] = self.pose_list[1]
	    	# ar.id[ind2] = self.id_list[2]
	    	# ar.pose[ind2] = self.pose_list[2]
	    	# self.pose_items.publish(ar)
	    	self.id_list = []
	    	self.pose_list =[]
    	





	    
		
		# 	x=1

def main():

	rospy.init_node('pose_item')
	gl = glue()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
