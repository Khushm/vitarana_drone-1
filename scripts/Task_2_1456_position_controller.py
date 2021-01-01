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

pick = 0
class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('Task_2_1456_position_controller')					# initializing ros node with name position_controller

        self.range = [0,0,0,0,0]
        self.drone_position = [19.0, 72.0, 0.31]				# current position of eDrone
        self.drone_setpoint1 = [110692.0702932625 * (19.0000451704 - 19),-105292.0089353767 * (72.0000000000 - 72), 3.00000000000]
        self.drone_setpoint2 = [0,0,0]
        self.drone_setpoint3 = [19.0007046575, 71.9998955286, 22.1599967919]	# [77.999997523, 11.000003582, 25.1599967919]
        self.altitude = 0.0
        self.altitude_setpoint = 25.16
        self.landing = 	0.31
        self.i_term = [0,0,0]

        self.state = 0

        # Kp, Ki, Kd values for z,x,y axis
        # self.throttle = [0,0,0]
        #self.throttle = [180,0.001,8300]
        self.throttle = [50,0.0001,9500]
        self.x_pid = [5,0.0,100]
        self.y_pid = [5,0.0,100]
	
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

    def lat_to_x(self,input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self,input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def range_finder_bottom(self,msg):
        self.altitude = msg.ranges[0]

    def range_finder_top(self,msg):
        self.range[0] = msg.ranges[0]
        self.range[1] = msg.ranges[1]
        self.range[2] = msg.ranges[2]
        self.range[3] = msg.ranges[3]
        self.range[4] = msg.ranges[4]
        # print(self.range)
	
    def gps_callback(self, msg):
        self.drone_position[0] = self.lat_to_x(msg.latitude)
        self.drone_position[1] = self.long_to_y(msg.longitude)
        self.drone_position[2] = msg.altitude
        print("x---",msg.latitude,self.drone_position[0])
 
    def altitude_set_pid(self, alt):
        self.throttle[0] = alt.Kp * 0.1
        self.throttle[1] = alt.Ki * 0.001
        self.throttle[2] = alt.Kd * 0.1

    def calc_throttle(self):
        self.error[2] = self.drone_setpoint1[2]-self.altitude

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
        self.error[0] = self.drone_setpoint1[0] - self.drone_position[0]
        self.error[1] = self.drone_setpoint1[1] - self.drone_position[1]
        
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
        if(True):
            print("throttle")
            self.calc_throttle()
            self.x_out, self.y_out = 0,0
            self.assign_rc()
            self.limit_values()
            self.publish_val()
            if(self.drone_position[2] > 3):
                self.state = 1

            if(self.state == 1):
                print("xy")
                self.calc_x_y()
                self.assign_rc()
                self.limit_values()
                self.publish_val()

if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()