#!/usr/bin/env python
import rospy
from starbax import ar_tagstr
from starbax import ar_tag
from std_msgs.msg import String
import numpy as np

class glue:

	def _init_(self):

		self.pose_items = rospy.Publisher("pose_and_item",ar_tagstr,queue_size = 10)
	 	self.sbid = rospy.Subscriber("id_items",String,id_items)
		self.ar = rospy.Subscriber("ar_pose_id",ar_tag,ar_info)
		self.list_obj = []
		self.id_list = []
		self.pose_list = []

	def id_items(self,strid):
		self.list_obj = strid.split()

	def ar_info(self,ar_msg)
		if len(self.id_list) < 3:
			self.id_list.append(ar_msg.id)
			self.pose_list.append(ar_msg.pose)
		if len(self.id_list) == 3 and self.id_list[0] != self.id_list[1] and self.id_list[0] != self.id_list[2] and self.id_list[1] != self.id_list[2]:
			ar = ar_tagstr()
			y0 = self.pose_list[0].Point.y
			y1 = self.pose_list[1].Point.y
			y2 = self.pose_list[2].Point.y
			pos = np.argsort([y0,y1,y2])
			pos = list(pos)

			ind0 = pos.index(0)
			ind1 = pos.index(1)
			ind2 = pos.index(2)

			ar.id[ind0] = self.id_list[0]
			ar.pose[ind0] = self.pose_list[0]
			ar.id[ind1] = self.id_list[1]
			ar.pose[ind1] = self.pose_list[1]
			ar.id[ind2] = self.id_list[2]
			ar.pose[ind2] = self.pose_list[2]
			self.pose_items.publish(ar)
			self.id_list = None
			self.pose = None

def main():

	rospy.init_node('pose_item')
	gl = glue()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()