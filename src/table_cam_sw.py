#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String

class starbaxHome:

	def __init__(self):
		
		self.pos_items = rospy.Publisher("pos_items",String, queue_size = 10)
		self.bridge = CvBridge()
		image_topic = "/cameras/left_hand_camera/image"
		self.sb = rospy.Subscriber(image_topic, Image, self.image_callback)
		self.lower_k,self.upper_k = self.bounds(cv2.imread('/home/solomon/catkin_ws/src/starbax/media/kcup.png'))
		self.lower_c,self.upper_c = self.bounds(cv2.imread('/home/solomon/catkin_ws/src/starbax/media/coffee.png'))
		self.lower_t,self.upper_t = self.bounds(cv2.imread('/home/solomon/catkin_ws/src/starbax/media/cup.png'))
		self.kernel = np.ones((5,5),np.uint8)
	def bounds(self,masked_img):

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
		l_h = np.argmax(hist_maskH[start:]>max(hist_maskH)*.02) + start
		l_s = -10+np.argmax(hist_maskS>max(hist_maskS)*.02)
		u_h = 180 - (np.argmax(flipH[end:]>max(flipH)*.02)+end)
		u_s = 10+255 - np.argmax(flipS>max(flipS)*.02)
		print l_h,l_s,u_h,u_s
		# plt.figure('histogram')
		# plt.subplot(211)
		# plt.plot(hist_maskH,'r')
		# plt.subplot(212)
		# plt.plot(hist_maskS,'g')
		# plt.show()
		lower = np.array([l_h,l_s,0],np.uint8)
		upper = np.array([u_h,u_s,255],np.uint8)

		return lower,upper

	def image_callback(self,msg):
		
		# Convert your ROS Image message to OpenCV2
		frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		row,col,_ = frame.shape
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		# hsv_frame = hsv_frame[100:(row-100),100:(col-100)]
		cv2.waitKey(1)
		Mask_k = cv2.inRange(hsv_frame,self.lower_k,self.upper_k)
		Mask_t = cv2.inRange(hsv_frame,self.lower_t,self.upper_t)
		Mask_c = cv2.inRange(hsv_frame,self.lower_c,self.upper_c)
	
		mask_k = cv2.erode(Mask_k,self.kernel,iterations = 1)
		mask_k = cv2.dilate(mask_k,self.kernel,iterations = 2)
		
		mask_t = cv2.erode(Mask_t,self.kernel,iterations = 1)
		mask_t = cv2.dilate(mask_t,self.kernel,iterations = 2)

		mask_c = cv2.erode(Mask_c,self.kernel,iterations = 1)
		mask_c = cv2.dilate(mask_c,self.kernel,iterations = 2)

		# k_count = np.sum(mask_k == 255)
		# t_count = np.sum(mask_t == 255)
		# c_count = np.sum(mask_c == 255)
		# print k_count,t_count,c_count

		#stdvtn = np.std([k_count, t_count, c_count])
		#max_ind = np.argmax([k_count,t_count,c_count])
		#if stdvtn > 1000:

		contourk= cv2.findContours(mask_k, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		contourt= cv2.findContours(mask_t, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		contourc= cv2.findContours(mask_c, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		ak,at,ac = 0,0,0
		k_candidate = False
		t_candidate = False
		c_candidate = False

		if len(contourk) > 0:	
			ck = max(contourk, key = cv2.contourArea)
			ak = cv2.contourArea(ck)
			if ak > 1000:
				k_candidate = True
				momentsk = cv2.moments(ck)
				if momentsk['m00'] != 0:
					cxk = int(momentsk['m10']/momentsk['m00'])
					cyk = int(momentsk['m01']/momentsk['m00'])
				cv2.drawContours(frame,ck,-1, (147,20,255),3)
				cv2.circle(frame,(int(cxk),int(cyk)),10,(147,20,255),-1)


		if len(contourt) > 0:	
			ct = max(contourt, key = cv2.contourArea)
			at = cv2.contourArea(ct)
			if at > 1000:
				t_candidate = True
				momentst = cv2.moments(ct)
				if momentst['m00'] != 0:
					cxt = int(momentst['m10']/momentst['m00'])
					cyt = int(momentst['m01']/momentst['m00'])
				cv2.drawContours(frame,ct,-1, (255,0,0),3)
				cv2.circle(frame,(int(cxt),int(cyt)),10,(255,0,0),-1)

		if len(contourc) > 0:	
			cc = max(contourc, key = cv2.contourArea)
			
			ac = cv2.contourArea(cc)
			if ac > 1000:
				c_candidate = True
				momentsc = cv2.moments(cc)
				if momentsc['m00'] != 0:
					cxc = int(momentsc['m10']/momentsc['m00'])
					cyc = int(momentsc['m01']/momentsc['m00'])
				cv2.drawContours(frame,cc,-1, (0,255,255),3)
				cv2.circle(frame,(int(cxc),int(cyc)),10,(0,255,255),-1)

		#print ak,at,ac

		if k_candidate == True and t_candidate == True and c_candidate == True:

			pos = np.argsort([cxk,cxt,cxc])
			pos = list(pos)
			indk = pos.index(0)
			indt = pos.index(1)
			indc = pos.index(2)

			labels = ['','','']
			labels[indk] = 'kcup'
			labels[indt] = 'cup'
			labels[indc] = 'coffee'

			print labels
			self.pos_items.publish(' '.join(labels))


		# cv2.imshow('mask_k',cv2.flip(mask_k,1))
		# cv2.waitKey(1)
		# cv2.imshow('mask_t',cv2.flip(mask_t,1))
		# cv2.waitKey(1)
		# cv2.imshow('mask_c',cv2.flip(mask_c,1))
		# cv2.waitKey(1)
		cv2.imshow('frame',frame)
		cv2.waitKey(1)

def main():

	rospy.init_node('table_cam')
	ic = starbaxHome()
 	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
