#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt

class starbaxHome:

	def __init__(self):
		
		self.bridge = CvBridge()
		image_topic = "/cameras/left_hand_camera/image"
		self.sb = rospy.Subscriber(image_topic, Image, self.image_callback)
		self.gotMask = False
	def image_callback(self,msg):

		# Convert your ROS Image message to OpenCV2
		table = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		cv2.waitKey(3)
		hsv_frame = cv2.cvtColor(table, cv2.COLOR_BGR2HSV)
		if self.gotMask == True:
			self.Mask = cv2.inRange(hsv_frame,self.lower,self.upper)
			table = cv2.bitwise_and(table,table,mask=self.Mask)	
		else:
			self.gotMask = True
			hist_maskH = cv2.calcHist([hsv_frame],[0],None,[180],[0,180])
			hist_maskS = cv2.calcHist([hsv_frame],[1],None,[256],[0,256])
			flipH = (hist_maskH[::-1].copy())
			flipS = (hist_maskS[::-1].copy())
			index = np.where(max(hist_maskH))
			start = 0
			end = 0
			if index >= 80:
				start = 80
			else:
				end = 80
				print 'greater'
			l_h = np.argmax(hist_maskH[start:]>max(hist_maskH)*.02) + start
			l_s = np.argmax(hist_maskS>max(hist_maskS)*.02)
			u_h = 180 - (np.argmax(flipH[end:]>max(flipH)*.02)+end)
			u_s = 256 - np.argmax(flipS>max(flipS)*.02)
			print l_h,l_s,u_h,u_s
			# plt.figure('histogram')
			# plt.subplot(211)
			# plt.plot(hist_maskH,'r')
			# plt.subplot(212)
			# plt.plot(hist_maskS,'g')
			# plt.show()
			self.lower = np.array([0,l_s,0],np.uint8)
			self.upper = np.array([30,u_s,255],np.uint8)
			
		cv2.imshow('table',table)
		

def main():

	ic = starbaxHome()
  	rospy.init_node('table_cam')
  
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()