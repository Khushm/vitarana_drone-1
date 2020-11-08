#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import *
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int64
import rospy
import time
import tf

class Edrone():
	def __init__(self):
		rospy.init_node('position_controller')

		self.setpoint_euler = [0,0,0]
		self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.drone_orientation_euler = [0.0, 0.0, 0.0, 0.0]
		self.error = [0.0, 0.0, 0.0]
		self.drone_position = [0.0, 0.0, 0.0]
		self.setpoint_cmd = [1500.0, 1500.0, 1500.0] 
		self.prev_error= [0.0, 0.0, 0.0]
		self.iterm = [0.0, 0.0, 0.0]
		self.min_value = [0,0,0,0]
		self.max_value = [1024,1024,1024,1024]		
		self.Kp = [0,0,0]
		self.Kd = [0,0,0]
		self.Ki = [0,0,0]

		self.pwm_cmd = prop_speed()
        	self.pwm_cmd.prop1 = 512.0
        	self.pwm_cmd.prop2 = 512.0
        	self.pwm_cmd.prop3 = 512.0
        	self.pwm_cmd.prop4 = 512.0


        	#self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

	        self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1) 
	        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1) 
	        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
	        self.zero_error_pub = rospy.Publisher('/zero_error', Float64, queue_size=1)
	        self.z_error_pub = rospy.Publisher('/z_error', Float64, queue_size=1)
	        self.z_error_pub = rospy.Publisher('/throttle_error', Float64, queue_size=1)

	        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
	        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
	        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
	        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)

		self.sample_time = 0.060
        def roll_set_pid(self,roll):
        	self.Kp[0] = roll.Kp * 0.0006 
		self.Ki[0] = roll.Ki * 0.00008
		self.Kd[0] = roll.Kd * 0.003
		#print("working")

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.0006 
		self.Ki[1] = pitch.Ki * 0.00008
		self.Kd[1] = pitch.Kd * 0.003

	def drone_command_callback(self, msg):
		self.setpoint_cmd[0] = msg.rcRoll
		self.setpoint_cmd[1] = msg.rcPitch
		self.setpoint_cmd[2] = msg.rcYaw
		#self.cmd.rcRoll = 1500
		#self.cmd.rcPitch = 1500
		#self.cmd.rcYaw = 1500
		#print("drone")
		#rospy.sleep(1)

	def imu_callback(self, msg):
	        self.drone_orientation_quaternion[0] = msg.orientation.x
	        self.drone_orientation_quaternion[1] = msg.orientation.y
	        self.drone_orientation_quaternion[2] = msg.orientation.z
	        self.drone_orientation_quaternion[3] = msg.orientation.w
		#print("imu")
		#rospy.sleep(1)


	def pid(self):
		
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) =       			tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
		
		#print(self.drone_orientation_euler[0], self.drone_orientation_quaternion[0],self.drone_orientation_euler[1], self.drone_orientation_quaternion[1],self.drone_orientation_euler[2], self.drone_orientation_quaternion[2],self.drone_orientation_quaternion[3])
		
		self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
		self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
		self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
		# Complete the equations for pitch and yaw axis

		self.error[0] = self.drone_orientation_euler[0] -self.setpoint_euler[0] 
		self.error[1] = self.drone_orientation_euler[1] - self.setpoint_euler[1]
		self.error[2] = self.drone_orientation_euler[2] - self.setpoint_euler[2]

		#proportional error
		self.roll_kp = self.Kp[0] * self.error[0]
		self.pitch_kp = self.Kp[1] * self.error[1]
		self.yaw_kp = self.Kp[2] * self.error[2]

		#derivative error
		self.roll_kd = -self.Kd[0]*(self.error[0] - self.prev_error[0])   
		self.pitch_kd = self.Kd[1] *(self.error[1] - self.prev_error[1]) 
		self.yaw_kd = self.Kd[2] *(self.error[2] - self.prev_error[2])

		#integrated error
		self.iterm[0] = (self.iterm[0] + self.error[0])*self.Ki[0]
		self.iterm[1] = (self.iterm[1] + self.error[1])*self.Ki[1]
		self.iterm[2] = (self.iterm[2] + self.error[2])*self.Ki[2]

		self.out_roll = self.roll_kp + self.roll_kd + self.iterm[0]
		self.out_pitch = self.pitch_kp + self.pitch_kd + self.iterm[1]
		self.out_yaw = self.yaw_kp + self.yaw_kd + self.iterm[2]
		
		self.pwm_cmd.prop1 =self.out_roll + self.out_pitch - self.out_yaw
        	self.pwm_cmd.prop2 =-self.out_roll + self.out_pitch + self.out_yaw
        	self.pwm_cmd.prop3 =self.out_roll - self.out_pitch - self.out_yaw
        	self.pwm_cmd.prop4 =-self.out_roll - self.out_pitch + self.out_yaw
		self.pwm_pub.publish(self.pwm_cmd)

		if self.out_roll > self.max_value[0]:
			self.out_roll = self.max_value[0]
		if self.out_pitch > self.max_value[1]:
			self.out_pitch = self.max_value[1]
		if self.out_yaw > self.max_value[2]:
			self.out_yaw = self.max_value[2]


		if self.out_roll < self.min_value[0]:
			self.out_roll = self.min_value[0]
		if self.out_pitch < self.min_value[1]:
			self.out_pitch = self.min_value[1]
		if self.out_yaw < self.min_value[2]:
			self.out_yaw = self.min_value[2]

		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1]
		self.prev_error[2] = self.error[2]

		print(self.out_roll, self.out_pitch, self.out_yaw)		

# 1 = roll - pitch - yaw
# 2 = pitch + roll + yaw
# 3 = pitch - roll - yaw
# 4 = yaw - pitch - roll

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
