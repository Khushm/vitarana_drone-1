#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int64
import rospy
import time
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')					# initializing ros node with name position_controller
	
        self.drone_position = [19.0, 72.0, 0.31]				# current position of eDrone
	self.drone_setpoint1 = [19.0000000000, 72.0000000000, 3.00000000000] 	# setpoint1 at altitude 3units
	self.drone_setpoint2 = [19.0000451704, 72.0000000000, 3.00000000000] 	# setpoint2 at altitude 3units
	self.landing = 0.31							# altitude setpoint for landing
	self.i_term = [0,0,0]							# integral term

	# Kp, Ki, Kd values for z,x,y axis
	self.throttle = [180,0.001,8300]
	self.x_pid = [10000,0.001,90000]
	self.y_pid = [10000,0.001,90000]
	
	# Declaring rc_cmd of message type edrone_cmd and initializing value
        self.rc_cmd = edrone_cmd()
        self.rc_cmd.rcRoll = 0.0
        self.rc_cmd.rcPitch = 0.0
        self.rc_cmd.rcYaw = 0.0
        self.rc_cmd.rcThrottle = 0.0

        self.prev_values = [0,0,0]		# storing previous errors in each axis [x, y, z]
	self.error = [0,0,0]			# storing updated errors in each axis [x, y, z]
        self.sample_time = 0.05			# this is the sample time in which you need to run pid.
	
	# ROS Publishers
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('/altitude_error/data', PidTune, queue_size=1)
	self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
	
	# ROS Subscribers
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
    
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
    # main function
    def pid(self):
	# is true untill drone reaches 19.0000451704 position in x(for reaching at 3 in z axis and travelling in x-axis)
	if(self.drone_position[0] < self.drone_setpoint2[0] - 0.0000047487):
		
		# Compute error in z axis
	        self.error[2] = self.drone_setpoint1[2] - self.drone_position[2]

		# Compute PID for z axis
	        self.p_throttle = self.throttle[0] * self.error[2]
		self.i_throttle = self.throttle[1] * self.error[2] + self.i_term[2]
	        self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
	
		# Calculate total pid output required for z
       		self.th_out = self.p_throttle + self.d_throttle + self.i_throttle

		# Update previous errors
		self.prev_values[2] = self.error[2]		

		# is true if drone is above 2.9units and below 3.1units to traverse in x axis
		if(self.drone_position[2] > (self.drone_setpoint1[2] - 0.1) and self.drone_position[2] < (self.drone_setpoint1[2] + 0.1)):
			# Compute error in x, y axis
			self.error[0] = self.drone_setpoint2[0] - self.drone_position[0]
			self.error[1] = self.drone_setpoint2[1] - self.drone_position[1]
			
			# Compute PID for x axis
			self.p_x = self.x_pid[0] * self.error[0]
			self.i_x = self.x_pid[1] * self.error[0] + self.i_term[0]
	       		self.d_x = self.x_pid[2] * (self.error[0] - self.prev_values[0])

			# Compute PID for y axis
			self.p_y = self.y_pid[0] * self.error[1]
			self.i_y = self.y_pid[1] * self.error[1] + self.i_term[1]
	       		self.d_y = self.y_pid[2] * (self.error[1] - self.prev_values[1])
			
			# Calculate total pid output required for x,y
			self.x_out = self.p_x + self.d_x + self.i_x		
			self.y_out = self.p_y + self.d_y + self.i_y

			# Update previous errors
			self.prev_values[0] = self.error[0]
			self.prev_values[1] = self.error[1]
		
			# Computing rc values
			self.rc_cmd.rcThrottle = self.th_out + 511
			self.rc_cmd.rcRoll = 1500.0 + self.x_out
        		self.rc_cmd.rcPitch = 1500.0 + self.y_out
        		self.rc_cmd.rcYaw = 1500.0
			
			# Publishing the values
			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)

		# resetting drone at 3units in z axis
		else:
			# Computing rc values
			self.rc_cmd.rcThrottle = self.th_out + 511
			self.rc_cmd.rcRoll = 1500.0
        		self.rc_cmd.rcPitch = 1500.0
        		self.rc_cmd.rcYaw = 1500.0
			
			# Publishing the values
			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)
	# to land the drone on the mark
	else:
		# Compute error in each axis
		self.error[0] = self.drone_setpoint1[0] - self.drone_position[0]
		self.error[1] = self.drone_setpoint1[1] - self.drone_position[1]
		self.error[2] = self.landing - self.drone_position[2]
		
		# Compute PID for z axis
		self.p_throttle = self.throttle[0] * self.error[2]
		self.i_throttle = self.throttle[1] * self.error[2] + self.i_term[2]
       		self.d_throttle = self.throttle[2] * (self.error[2] - self.prev_values[2])
		
		# Compute PID for x axis
		self.p_x = self.x_pid[0] * self.error[0]
		self.i_x = self.x_pid[1] * self.error[0] + self.i_term[0]
       		self.d_x = self.x_pid[2] * (self.error[0] - self.prev_values[0])

		# Compute PID for y axis
		self.p_y = self.y_pid[0] * self.error[1]
		self.i_y = self.y_pid[1] * self.error[1] + self.i_term[1]
       		self.d_y = self.y_pid[2] * (self.error[1] - self.prev_values[1])
		
		# Calculate total pid output
		self.throttle_out = self.p_throttle + self.d_throttle + self.i_throttle
		self.x_out = self.p_x + self.d_x + self.i_x		
		self.y_out = self.p_y + self.d_y + self.i_y
		
		# Update previous errors
		self.prev_values[0] = self.error[0]
		self.prev_values[1] = self.error[1]
		self.prev_values[2] = self.error[2]  
		if(self.drone_position[2] <= self.landing + 1.5):
			# Computing rc values
			self.rc_cmd.rcThrottle = self.throttle_out
			self.rc_cmd.rcRoll = 1500.0 + self.x_out
			self.rc_cmd.rcPitch = 1500.0 + self.y_out
			self.rc_cmd.rcYaw = 1500.0
			
			# Publishing the values
			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)
		else:
			# Computing rc values
			self.rc_cmd.rcThrottle = 100
			self.rc_cmd.rcRoll = 1500.0
			self.rc_cmd.rcPitch = 1500.0
			self.rc_cmd.rcYaw = 1500.0
			
			# Publishing the values
			self.zero_error_pub.publish(0)
			self.z_error_pub.publish(self.th_out)
			self.drone_cmd_pub.publish(self.rc_cmd)

		
        
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(20)	# rate in Hz 
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
