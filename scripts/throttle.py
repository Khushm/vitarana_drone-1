#!/usr/bin/env python

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
        rospy.init_node('throttle1')
	
	#ki_roll = 0.0002
	#kd_roll = 89
        self.drone_position= [19.0, 72.0, 0.31]
        self.drone_setpoint = [19.0, 72.0, 3.0]
	#self.throttle = [60,0.24,349]
	self.throttle = [180,0.001,8300]
        
	self.i_term = [0,0,0]

	self.x_pid = [10000,0.001,100000]
	self.y_pid = [10000,0.001,100000]

        self.rc_cmd = edrone_cmd()
        self.rc_cmd.rcRoll = 1500.0
        self.rc_cmd.rcPitch = 1500.0
        self.rc_cmd.rcYaw = 1500.0
        self.rc_cmd.rcThrottle = 0.0

	self.error = [0,0,0]
        self.prev_values = [0,0,0]
        self.max_values = [1024, 1024, 1024]
        self.min_values = [0, 0, 0]

        self.sample_time = 0.05
	
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
     #   self.throttle_error_pub = rospy.Publisher('/altitude_error/data', PidTune, queue_size=1)
	self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
	
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    def altitude_set_pid(self, alt):
        self.throttle[0] = alt.Kp * 1
        self.throttle[1] = alt.Ki * 0.1
        self.throttle[2] = alt.Kd * 2

    def pid(self):
        self.error[2] = self.drone_setpoint[2] - self.drone_position[2]

        self.p_thr = self.throttle[0] * self.error[2]
        self.d_thr = self.throttle[2] * (self.error[2] - self.prev_values[2])
        self.i_thr = self.throttle[1] * self.error[2] + self.i_term[2]
	
        self.th_out = self.p_thr + self.d_thr + self.i_thr
	
	self.prev_values[0] = self.error[0]
	self.prev_values[1] = self.error[1]
	
	self.error[0] = self.drone_setpoint[0] - self.drone_position[0]
	self.error[1] = self.drone_setpoint[1] - self.drone_position[1]
	
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

	self.rc_cmd.rcThrottle = self.th_out
	self.rc_cmd.rcRoll = 1500.0 + self.x_out
	self.rc_cmd.rcPitch = 1500.0 + self.y_out
	self.rc_cmd.rcYaw = 1500.0
	
	print("published1")
	self.zero_error_pub.publish(0)
	self.z_error_pub.publish(self.th_out)
	self.drone_cmd_pub.publish(self.rc_cmd)

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
