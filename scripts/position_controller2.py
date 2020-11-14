#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu,NavSatFix
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

global qr_chk
qr_chk=0
count = 0
class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller2')					# initializing ros node with name position_controller

        self.drone_position = [19.0009248718, 71.9998318945,22.16]
	# self.drone_setpoint1 = [19.0009248718,71.9998318945 , 25.16]
	self.drone_setpoint2 = [0,0,0]	
	self.drone_setpoint3 = [19.0007046575, 71.9998955286, 25.1599967919]

	self.landing = 22.1599967919							
	self.i_term = [0,0,0]

	# Kp, Ki, Kd values for z,x,y axis
	self.throttle = [180,0.001,8300]
	self.x_pid = [5500,27.3,2100000]
	self.y_pid = [5500,27.3,2100000]
	
	# Declaring rc_cmd of message type edrone_cmd and initializing value
        self.rc_cmd = edrone_cmd()
        self.rc_cmd.rcRoll = 0.0
        self.rc_cmd.rcPitch = 0.0
        self.rc_cmd.rcYaw = 0.0
        self.rc_cmd.rcThrottle = 0.0

        self.prev_values = [0,0,0]
	self.error = [0,0,0]
        self.sample_time = 0.05
	
 	self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('/altitude_error/data', PidTune, queue_size=1)
	self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
	
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
	
    def image_callback(self, data):
	try:
		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		image=self.img
		global qr_chk
		cv2.imshow('image',image)
		#decode(Image.open('qrcode.png'), symbols=[ZBarSymbol.QRCODE])
		#decode(Image.open(self.img))
		barcodes=pyzbar.decode(image)
		for barcode in barcodes:
			barcodeData=barcode.data.decode("utf-8")
			if(barcodeData != 0):
				qr_chk = 1
			if(qr_chk == 1):
				self.drone_setpoint2 = float(barcodeData.split(","))
				print(self.drone_setpoint2)
				count = 1
		cv2.waitKey(1)
		
	except CvBridgeError as e:
		print(e)
		return
  
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude
 
    def altitude_set_pid(self, alt):
        self.throttle[0] = alt.Kp * 1
        self.throttle[1] = alt.Ki * 0.1
        self.throttle[2] = alt.Kd * 2
  #    self.drone_position[0] != self.drone_setpoint3[0] and self.drone_position[1] != self.drone_setpoint3[1]
    def pid(self):
	global count
	if(count == 0):
		if not(self.drone_position[0] > (self.drone_setpoint3[0] - 0.0000000516) and self.drone_position[0] < (self.drone_setpoint3[0] + 0.0000000516) and self.drone_position[1] > (self.drone_setpoint3[1] - 0.0000004380) and self.drone_position[1] < (self.drone_setpoint3[1] + 0.0000004380)):

			print("in")
			self.error[2] = self.drone_setpoint3[2] - self.drone_position[2]
			
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
			if(self.drone_position[2] >= 2.9):
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
			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)
			print("done")
	if(count == 1):
		print("hi")

	
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
