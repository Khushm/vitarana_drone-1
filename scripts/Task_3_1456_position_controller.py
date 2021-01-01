#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix,LaserScan,Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
import cv2
import rospy
import time
import tf

global qr_chk
qr_chk=0
global pick
pick = 0
class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('Task_2_1456_position_controller')					# initializing ros node with name position_controller

        self.range = [0,0,0,0,0]
        self.drone_position = [18.999241138,  71.9998195496, 16.6600201637]
	    # self.drone_setpoint1 = [19.0009248718,71.9998318945 , 25.16]
        self.drone_setpoint2 = [0,0,0]	
        self.drone_setpoint3 = [18.9990965928, 72.0000664814, 10.75]	# [77.999997523, 11.000003582, 25.1599967919]
        self.altitude = 0.0
        self.altitude_setpoint = 25.16
        self.landing = 	0.31
        self.i_term = [0,0,0]

        self.state = 1

        # Kp, Ki, Kd values for z,x,y axis
        # self.throttle = [0,0,0]
        #self.throttle = [180,0.001,8300]
        self.throttle = [50,0.0001,9500]
        self.x_pid = [7000,0.003,7900000]
        self.y_pid = [7000,0.003,7900000]
	
	    # Declaring rc_cmd of message type edrone_cmd and initializing value
        self.rc_cmd = edrone_cmd()
        self.rc_cmd.rcRoll = 0.0
        self.rc_cmd.rcPitch = 0.0
        self.rc_cmd.rcYaw = 0.0
        self.rc_cmd.rcThrottle = 0.0

        self.prev_values = [0,0,0]
        self.error_sum = [0,0,0]
        self.error = [0,0,0]
        self.sample_time = 0.01

        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan ,self.range_finder_bottom)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan ,self.range_finder_top)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
        self.bridge = CvBridge()

    def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			image=self.img
			global qr_chk
			cv2.imshow('image',image)
			
			barcodes=pyzbar.decode(image)
			for barcode in barcodes:
				barcodeData=barcode.data.decode("utf-8")
				if(barcodeData!=0):
					qr_chk=barcodeData
					barcodeData=eval(barcodeData)
					barcodeData=[float(x) for x in barcodeData]
					self.drone_setpoint2[0]= barcodeData[0]
					self.drone_setpoint2[1]= barcodeData[1] 
					self.drone_setpoint2[2]= barcodeData[2]
					pick = 1
					print(self.drone_setpoint2)
					print(type(barcodeData))
					
			#print(qr_chk) 
			cv2.waitKey(1)
			
		except CvBridgeError as e:
			print(e)
			return

    def range_finder_bottom(self,msg):
        self.altitude = msg.ranges[0]
        print(self.altitude)

    def range_finder_top(self,msg):
        self.range[0] = msg.ranges[0]
        self.range[1] = msg.ranges[1]
        self.range[2] = msg.ranges[2]
        self.range[3] = msg.ranges[3]
        self.range[4] = msg.ranges[4]
        # print(self.range)
	
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude
 
    def altitude_set_pid(self, alt):
        self.throttle[0] = alt.Kp * 0.1
        self.throttle[1] = alt.Ki * 0.001
        self.throttle[2] = alt.Kd * 0.1

    def calc_throttle(self):
        self.error[2] = self.altitude_setpoint - self.drone_position[2]

        self.error_sum[2] = self.error_sum[2] + self.error[2]

        self.p_throttle = self.throttle[0] * self.error[2]
        self.i_throttle = self.throttle[1] * self.error_sum[2]
        self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
	
       	self.th_out = self.p_throttle + self.d_throttle + self.i_throttle

        self.prev_values[2] = self.error[2]
    
    def calc_landing(self):
        self.error[2] = self.landing - self.altitude

        self.error_sum[2] = self.error_sum[2] + self.error[2]

        self.p_throttle = self.throttle[0] * self.error[2]
        self.i_throttle = self.throttle[1] * self.error_sum[2]
        self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
	
       	self.th_out = self.p_throttle + self.d_throttle + self.i_throttle

        self.prev_values[2] = self.error[2]
    

    def calc_x_y(self):
        self.error[0] = self.drone_setpoint3[0] - self.drone_position[0]
        self.error[1] = self.drone_setpoint3[1] - self.drone_position[1]
        
        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]

        self.p_x = self.x_pid[0] * self.error[0]
        self.i_x = self.x_pid[1] * self.error_sum[0]
        self.d_x = self.x_pid[2] * (self.error[0] - self.prev_values[0])

        self.p_y = self.y_pid[0] * self.error[1]
        self.i_y = self.y_pid[1] * self.error_sum[1]
        self.d_y = self.y_pid[2] * (self.error[1] - self.prev_values[1])
        
        self.x_out = self.p_x + self.d_x + self.i_x		
        self.y_out = self.p_y + self.d_y + self.i_y

        self.prev_values[0] = self.error[0]
        self.prev_values[1] = self.error[1]
    
    def publish_val(self):
        self.zero_error_pub.publish(0)
        self.z_error_pub.publish(self.error[2])
        self.x_error_pub.publish(self.error[0])
        self.y_error_pub.publish(self.error[1])
        self.drone_cmd_pub.publish(self.rc_cmd)

    def assign_rc(self):
        self.rc_cmd.rcThrottle = self.th_out + 1500
        self.rc_cmd.rcRoll  = self.x_out + 1500.0 
        self.rc_cmd.rcPitch = self.y_out + 1500.0
        self.rc_cmd.rcYaw = 1500.0
    
    def limit_values(self):
        if (self.rc_cmd.rcThrottle > 2000):
            self.rc_cmd.rcThrottle = 2000
        if (self.rc_cmd.rcThrottle < 1000):
            self.rc_cmd.rcThrottle = 1000
        if (self.rc_cmd.rcRoll > 2000):
            self.rc_cmd.rcRoll = 2000
        if (self.rc_cmd.rcRoll < 1000):
            self.rc_cmd.rcRoll = 1000
        if (self.rc_cmd.rcPitch > 2000):
            self.rc_cmd.rcPitch = 2000
        if (self.rc_cmd.rcPitch < 1000):
            self.rc_cmd.rcPitch = 1000

    def pid(self):
        if(pick == 0):
            if not(self.drone_position[0] > (self.drone_setpoint3[0] - 0.0000000516) and self.drone_position[0] < (self.drone_setpoint3[0] + 0.0000000516) and self.drone_position[1] > (self.drone_setpoint3[1] - 0.0000004380) and self.drone_position[1] < (self.drone_setpoint3[1] + 0.0000004380)):
                self.calc_throttle()
                print("0")
                self.x_out, self.y_out = 0,0
                self.assign_rc()
                self.limit_values()
                self.publish_val()

                if(self.drone_position[2] > 24.9):
                    self.calc_x_y()
                    self.assign_rc()
                    self.limit_values()
                    self.publish_val()    
                    print("0xy")
            
        if(pick == 1): 
            pass

if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)	# rate in Hz 
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
