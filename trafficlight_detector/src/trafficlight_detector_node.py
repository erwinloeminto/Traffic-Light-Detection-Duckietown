#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, FSMState
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import cv2
import sys
import time
import threading
import argparse
class TrafficLightDetectorNode(object):
	def __init__(self):
		self.node_name = "Trafficlight Detector"

		self.thread_lock = threading.Lock()
		self.active = True
		self.state = "JOYSTICK_CONTROL"
		# to do: initial no-trafficlights-detected as trafficlight_detected senario
		self.trafficlight_detected = 0

		self.bridge = CvBridge()
		self.red_detected = 0
		self.yellow_detected = 0
		self.green_detected = 0
		
		# Publishers

		# To do : publish ros message topic: /node_name/car_cmd, datatype: Twist2DStamped 
		self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

		self.pub_image_trafficlight = rospy.Publisher("~image_with_trafficlight", Image, queue_size=1)
		self.pub_state_mode = rospy.Publisher("fsm_node/mode", FSMState, queue_size=1)

        # Subscribers

		self.sub_joystick_car_cmd = rospy.Subscriber("~joystick_car_cmd", Twist2DStamped, self.cbJoystick,queue_size=1)
		self.sub_state_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cbFSMState, queue_size=1)
		self.sub_image_origin = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

		# Send stop command
		car_control_msg = Twist2DStamped()
		car_control_msg.v = 0.0
		car_control_msg.omega = 0.0
		self.publishCmd(car_control_msg)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbImage(self, image_msg):
		if not self.active:
			return

		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()

	def processImage(self, image_msg):
		if not self.thread_lock.acquire(False):
			return

		try:
			self.cbtrafficlightdetect(image_msg)
		finally:
			self.thread_lock.release()

	def publishCmd(self,car_cmd_msg):

		# to do: using pub_car_cmd publisher we initialed at line 24 to publish car command message
		self.pub_car_cmd.publish(car_cmd_msg)
	
	def cbJoystick(self,car_cmd_msg):

		# to do: if trafficlight_detected senario is no-trafficlight-detected, keep joystikck command as car control command
		if self.trafficlight_detected == 0:

			# to do: initial a car commad message for publish, datatype: Twist2DStamped
			car_control_msg = Twist2DStamped()

            # to do: using joystikck command as car command
			car_control_msg.v = car_cmd_msg.v
			car_control_msg.omega = car_cmd_msg.omega

			# to do: publish car control command
			self.publishCmd(car_cmd_msg)

	def cbtrafficlightdetect(self, image_msg):

		#Range warna
		greenLower = (23, 14, 245)	#31 Mei 2018		
		greenUpper = (52, 119, 255) #31 Mei 2018
		yellowLower = (17, 89, 223) #31 Mei 2018
		yellowUpper = (30, 227, 255) #31 Mei 2018
		redLower = (0, 8, 255)
		redUpper = (179, 21, 255)
		# Decompress image and convert ROS image message to cv image
		narr = np.fromstring(image_msg.data, np.uint8)
		frame = cv2.imdecode(narr, cv2.IMREAD_COLOR)
        
        # Initial opencv CascadeClassifier class to detect objects and import trafficlight detection module
			#trafficlightCascade = cv2.CascadeClassifier('/home/ubuntu/duckietown/catkin_ws/src/spring2016_nctu/wama/trafficlight_detector/src/haarcascade_frontaltrafficlight_default.xml')
			#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		#Localization
		image = frame[150:300, 250:800]
		#####BATAS EDIT#####
		#Proses BGR to HSV & Denoising
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask1 = cv2.inRange(hsv, redLower, redUpper)	
		mask1 = cv2.erode(mask1, None, iterations=2)
		mask1 = cv2.dilate(mask1, None, iterations=2)

		mask2 = cv2.inRange(hsv, yellowLower, yellowUpper)
		mask2 = cv2.erode(mask2, None, iterations=2)
		mask2 = cv2.dilate(mask2, None, iterations=2)

		mask3 = cv2.inRange(hsv, greenLower, greenUpper)
		mask3 = cv2.erode(mask3, None, iterations=2)
		mask3 = cv2.dilate(mask3, None, iterations=2)
		
		trafficlights1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
		trafficlights2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
		trafficlights3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

		#####BATAS EDIT#####

		# Detect trafficlights in the image
			#trafficlights = trafficlightCascade.detectMultiScale(gray,scaleFactor=2,minNeighbors=5,minSize=(10, 10),flags = cv2.CASCADE_SCALE_IMAGE)
		#print "Found {0} Red!".format(len(trafficlights1))
		#print "Found {0} Yellow!".format(len(trafficlights2))
		#print "Found {0} Green!".format(len(trafficlights3))
        # Draw trafficlight detections region proposals in the image
		#for (x, y, w, h) in trafficlights:
		#	cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
		#for (x, y, radius) in trafficlights:
		#	cv2.rectangle(image, (x, y), (x+radius, y+radius), (0, 255, 0), 2)
		#cv2.circle(image, center, 5, (0, 0, 255), -1)
		#cv2.imshow("Frame", image)
			
		

		# to do: if trafficlights detected, using stop command as car control command
		#if len(trafficlights) != 0:
		if len(trafficlights1) > 0:		
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
			c1 = max(trafficlights1, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c1)
			M1 = cv2.moments(c1)
			center1 = (int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
 
			# only proceed if the radius meets a minimum size
			if radius > 0 and radius < 15:
				
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(image, (int(x), int(y)), int(radius),
					(0, 0, 255), 2)
				cv2.circle(image, center1, 5, (0, 0, 255), -1)
				cv2.putText(image,"Red Detected", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
										
		if len(trafficlights2) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
			c2 = max(trafficlights2, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c2)
			M2 = cv2.moments(c2)
			center2 = (int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))
 
			# only proceed if the radius meets a minimum size
			if radius > 0 and radius < 15:
				
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(image, (int(x), int(y)), int(radius),
					(0, 0, 255), 2)
				cv2.circle(image, center2, 5, (0, 0, 255), -1)
				cv2.putText(image,"Yellow Detected", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

		if len(trafficlights3) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
			c3 = max(trafficlights3, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c3)
			M3 = cv2.moments(c3)
			center3 = (int(M3["m10"] / M3["m00"]), int(M3["m01"] / M3["m00"]))
 
			# only proceed if the radius meets a minimum size
			if radius > 0 and radius < 15:
				
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(image, (int(x), int(y)), int(radius),
					(0, 0, 255), 2)
				cv2.circle(image, center3, 5, (0, 0, 255), -1)
				cv2.putText(image,"Green Detected", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
		# Convert cv image to ROS image message
		image_msg_out = self.bridge.cv2_to_imgmsg(image, "bgr8")
		image_msg_out.header.stamp = image_msg.header.stamp

		# to do: using pub_image_trafficlight publisher we initialed at line 27 to publish image with trafficlight region proposals
		self.pub_image_trafficlight.publish(image_msg_out)

        # to do: initial a car commad message for publish, datatype: Twist2DStamped
		car_control_msg = Twist2DStamped()		

		if len(trafficlights1) != 0:
			self.trafficlight_detected = 1
			self.red_detected = 1
	        # to do: use stop command as car command
			car_control_msg.v = 0
			car_control_msg.omega = 0

			# to do: publish car control command
			self.publishCmd(car_control_msg)
			#cv2.putText(image,"Green Detected", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
			# to do: set trafficlights-detected as trafficlight_detected senario
			

		# to do: if no trafficlights detected,  set no-trafficlights-detected as trafficlight_detected senario
		if len(trafficlights1) == 0:
            # to do: set no-trafficlights-detected as trafficlight_detected senario
			self.trafficlight_detected = 0
			self.red_detected = 0
			#car_control_msg.v=0
			#car_control_msg.omega=0
			#self.publishCmd(car_control_msg)
		if len(trafficlights2) != 0:
            # to do: set no-trafficlights-detected as trafficlight_detected senario
			self.trafficlight_detected = 1
			self.yellow_detected = 1
		if len(trafficlights2) == 0:
            # to do: set no-trafficlights-detected as trafficlight_detected senario
			self.trafficlight_detected = 0
			self.yellow_detected = 0
		if len(trafficlights3) != 0:
            # to do: set no-trafficlights-detected as trafficlight_detected senario
			self.trafficlight_detected = 1
			self.green_detected = 1
		if len(trafficlights3) == 0:
            # to do: set no-trafficlights-detected as trafficlight_detected senario
			self.trafficlight_detected = 0
			self.green_detected = 0
		# only proceed if the radius meets a minimum size
		# draw the circle and centroid on the frame,
		# then update the list of tracked points
		#	cv2.circle(image, (int(x), int(y)), int(radius),
		#	(0, 255, 255), 2)

	def cbFSMState (self, msg):
		self.state = msg.state
		#print 'state skrg {0}'.format(self.sub_state_mode)
		#print 'ngulang'
		while self.state == "INTERSECTION_CONTROL" : #and len(trafficlights2) > 0:
			#print 'state skrg  123 {0}'.format(self.sub_state_mode)
			
			#print 'lewat detect merah'
			if self.yellow_detected != 0 or self.green_detected != 0:
				print 'Kuning{0}'.format(self.yellow_detected)
				print 'Hijau{0}'.format(self.green_detected)
				msg_state = FSMState()
				msg_state.header.stamp = msg.header.stamp
				msg_state.state = "LANE_FOLLOWING_AVOID"
				print 'state berubah jadi {0}'.format(msg_state)
				self.pub_state_mode.publish(msg_state)
			if self.red_detected != 0 and (self.yellow_detected == 0 or self.green_detected == 0):
				print 'Merah{0}'.format(self.red_detected)

	def onShutdown(self):
		rospy.loginfo("[TrafficlightDetectorNode] Shutdown.")

if __name__ == "__main__":
	# to do: initial a node named "trafficlight_detector_X", X= you duckiebot name
	rospy.init_node("trafficlight_detector",anonymous=False)
	trafficlight_detector_node = TrafficLightDetectorNode()
	rospy.on_shutdown(trafficlight_detector_node.onShutdown)
	rospy.spin()
