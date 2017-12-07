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
		self.lower = np.zeros(3)
		self.upper = np.zeros(3)
	def image_callback(self,msg):

		# Convert your ROS Image message to OpenCV2
		frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		row,col,_ = frame.shape
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		k = cv2.waitKey(10)
		bsize = 25
		if k == ord('b'):
			mask = np.zeros((row,col), np.uint8)
			mask[int(row/2.-bsize):int(row/2.+bsize),int(col/2.-bsize):int(col/2.+bsize)] = 255
			masked_img = cv2.bitwise_and(frame,frame,mask = mask)
			masked_img = masked_img[int(row/2.-bsize):int(row/2.+bsize),int(col/2.-bsize):int(col/2.+bsize)]
			cv2.imshow('masked_img',masked_img)
			cv2.waitKey(30)
			cv2.imwrite('coffee.png',masked_img)
			self.gotMask = True
			mask_hsv = cv2.cvtColor(masked_img, cv2.COLOR_BGR2HSV)
			hist_maskH = cv2.calcHist([mask_hsv],[0],None,[180],[0,180])
			hist_maskS = cv2.calcHist([mask_hsv],[1],None,[256],[0,256])
			flipH = (hist_maskH[::-1].copy())
			flipS = (hist_maskS[::-1].copy())
			index = hist_maskH.argmax()
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
			u_s = 255 - np.argmax(flipS>max(flipS)*.02)
			print l_h,l_s,u_h,u_s
			plt.figure('histogram')
			plt.subplot(211)
			plt.plot(hist_maskH,'r')
			plt.subplot(212)
			plt.plot(hist_maskS,'g')
			plt.show()
			self.lower = np.array([l_h,l_s,0],np.uint8)
			self.upper = np.array([u_h,u_s,255],np.uint8)
		if self.gotMask == False:
			cv2.rectangle(frame,(int(col/2.-bsize),int(row/2.-bsize)),(int(col/2.+bsize),int(row/2.+bsize)),(0,255,0),3)
		else:
			frame = cv2.inRange(hsv_frame,self.lower,self.upper)
		
		cv2.imshow('Calibration',cv2.flip(frame,1))
		

def main():
	
	ic = starbaxHome()
	rospy.init_node('pic_cal')
  
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()