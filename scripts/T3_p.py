#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu,NavSatFix,LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
import cv2
import rospy
import time
import tf
from vitarana_drone.msg import value
import cv2
from matplotlib import pyplot as plt
import math


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
		rospy.init_node('T3_p')					# initializing ros node with name position_controller
		self.drone_position = [18.999241138,  71.9998195496, 16.6600201637] # self.drone_setpoint1 = [19.0009248718,71.9998318945 , 25.16]
		self.drone_setpoint2 = [0,0,0]	
		self.drone_setpoint3 = [18.999241138,  71.9998195496, 25.2]
		# self.drone_setpoint3 = [18.9990965925, 71.9999050292, 28.2]
		self.landing = 0.31							# altitude setpoint for landing
		self.i_term = [0,0,0]						# integral term

	# Kp, Ki, Kd values for z,x,y axis
		self.throttle = [180,0.001,8300]
		self.x_pid = [2500,0.001,1100000]
		self.y_pid = [2500,0.001,1100000]
	
	# Declaring rc_cmd of message type edrone_cmd and initializing value
		self.rc_cmd = edrone_cmd()
		self.rc_cmd.rcRoll = 0.0
		self.rc_cmd.rcPitch = 0.0
		self.rc_cmd.rcYaw = 0.0
		self.rc_cmd.rcThrottle = 0.0

		self.prev_values = [0,0,0]		# storing previous errors in each axis [x, y, z]
		self.error = [0,0,0]			# storing updated errors in each axis [x, y, z]
		self.sample_time = 0.01			# this is the sample time in which you need to run pid.
	
	# ROS Publishers
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
		self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
		self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
		self.throttle_error_pub = rospy.Publisher('/altitude_error/data', PidTune, queue_size=1)
		self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

		self.x_m = rospy.Publisher('/edrone/err_x_m', Float32, queue_size=1)
		self.y_m = rospy.Publisher('/edrone/err_x_m', Float32, queue_size=1)
		self.id  = rospy.Publisher('/edrone/curr_marker_id', Float32, queue_size=1)

	# ROS Subscribers
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan ,self.range_finder_bottom)
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
		self.bridge = CvBridge()

    def range_finder_bottom(self,msg):
		self.altitude = msg.ranges[0]

    # GPS callback function
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    # Callback function for /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.throttle[0] = alt.Kp * 1
        self.throttle[1] = alt.Ki * 0.1
        self.throttle[2] = alt.Kd * 2

    def image_callback(self, data):
        try:
            logo_cascade = cv2.CascadeClassifier('/home/khush/catkin_ws/src/vitarana_drone/scripts/intro_cascade_classifiers_training_and_usage/data/cascade.xml')
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            image = self.img
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
			# cv2.imshow('image',image)
        #    cv2.waitKey(1)
            for (x, y, w, h) in logo:
				cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
				plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)) 
				plt.show()
				X = ((2*x)+w/2) * self.altitude/(200/math.tan(1.3962634/2))	#focal_length
				Y = ((2*y)+h/2) * self.altitude/(200/math.tan(1.3962634/2))	#focal_length
				self.x_m.publish(X)
				self.y_m.publish(Y)
				#key = cv2.waitKey(1) & 0xFF
			# if the 'q' key is pressed, stop the loop
				#if key == ord("q"):
				#	break
        except CvBridgeError as e:
            print(e)
            return


    # main function
    def pid(self):
		if not(self.drone_position[0] > (self.drone_setpoint3[0] - 0.0000000516) and self.drone_position[0] < (self.drone_setpoint3[0] + 0.0000000516) and self.drone_position[1] > (self.drone_setpoint3[1] - 0.0000004380) and self.drone_position[1] < (self.drone_setpoint3[1] + 0.0000004380)):

			print("in")
			self.error[2] = self.drone_setpoint3[2] - self.drone_position[2] - 1
			
			self.p_throttle = self.throttle[0] * self.error[2]
			self.i_throttle = self.throttle[1] * self.error[2] + self.i_term[2]
			self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
				
			self.th_out = self.p_throttle + self.d_throttle + self.i_throttle
			
			self.prev_values[2] = self.error[2]
			
			self.rc_cmd.rcThrottle = self.th_out + 511
			self.rc_cmd.rcRoll = 1500.0
			self.rc_cmd.rcPitch = 1500.0
			self.rc_cmd.rcYaw = 1500.0

			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)
			

			if(self.drone_position[2] > 27):
				self.error[0] = self.drone_setpoint3[0] - self.drone_position[0]
				self.error[1] = self.drone_setpoint3[1] - self.drone_position[1]
				
				self.p_x = self.x_pid[0] * self.error[0]
				self.i_x = self.x_pid[1] * self.error[0] + self.i_term[0]
				self.d_x = self.x_pid[2] * (self.error[0] - self.prev_values[0])

				self.p_y = self.y_pid[0] * self.error[1]
				self.i_y = self.y_pid[1] * self.error[1] + self.i_term[1]
				self.d_y = self.y_pid[2] * (self.error[1] - self.prev_values[1])
				
				self.x_out = self.p_x + self.d_x + self.i_x		
				self.y_out = self.p_y + self.d_y + self.i_y

				self.prev_values[0] = self.error[0]
				self.prev_values[1] = self.error[1]

				print("if2")

				self.rc_cmd.rcThrottle = self.th_out + 511
				self.rc_cmd.rcRoll = 1500.0 + self.x_out
				self.rc_cmd.rcPitch = 1500.0 + self.y_out
				self.rc_cmd.rcYaw = 1500.0
				
				self.zero_error_pub.publish(0)
				self.z_error_pub.publish(self.th_out)
				self.drone_cmd_pub.publish(self.rc_cmd)
		else:
			self.error[2] = self.landing - self.drone_position[2]
			
			self.p_throttle = self.throttle[0] * self.error[2]
			self.i_throttle = self.throttle[1] * self.error[2] + self.i_term[2]
			self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
				
			self.th_out = self.p_throttle + self.d_throttle + self.i_throttle
			
			self.prev_values[2] = self.error[2]
			
			self.rc_cmd.rcThrottle = self.th_out + 511
			self.rc_cmd.rcRoll = 1500.0
			self.rc_cmd.rcPitch = 1500.0
			self.rc_cmd.rcYaw = 1500.0
        
if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
