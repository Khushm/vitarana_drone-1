#!/usr/bin/env python

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
        def __init__(self):
                rospy.init_node('attitude_controller')				# initializing ros node with name attitude_controller

                self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]	# current orientation of eDrone in quaternion format [x,y,z,w]
                self.drone_orientation_euler = [0.0, 0.0, 0.0]			# current orientation of eDrone converted in euler angles form [r,p,y]

                self.setpoint_cmd = [0.0, 0.0, 0.0, 0.0]			 # setpoint that will be received from the drone_command [p_setpoint, r_setpoint, y_setpoint]
                self.setpoint_euler = [0.0, 0.0, 0.0]				 # setpoint of orientation in euler angles to stabilize the drone[p_setpoint, r_psetpoint, y_setpoint]

                # Declaring pwm_cmd of message type prop_speed and initializing value
                self.pwm_cmd = prop_speed()
                self.pwm_cmd.prop1 = 0.0
                self.pwm_cmd.prop2 = 0.0
                self.pwm_cmd.prop3 = 0.0
                self.pwm_cmd.prop4 = 0.0

                # initial setting of Kp, Kd and ki for [roll, pitch, yaw]
                self.Kp = [80, 80, 200.0]
                self.Ki = [0.0, 0.0, 0.0]
                self.Kd = [10500, 10500, 1470]

                self.prev_error = [0.0, 0.0, 0.0]				# storing previous errors in each axis [pitch,roll, yaw]
                self.error = [0.0, 0.0, 0.0]					# storing updated errors in each axis [pitch,roll, yaw]
                self.sum_error = [0.0, 0.0, 0.0]				# integral term

                self.out_throttle = 0
                self.out_roll = 0
                self.out_pitch = 0
                self.out_yaw = 0

                #  variables for limiting the values
                self.max_values = [1023, 1023, 1023, 1023]
                self.min_values = [0, 0, 0, 0]

                # This is the sample time in which you need to run pid.
                self.sample_time = 0.016 # in seconds

                # ROS Publishers
                self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
                self.roll_error_pub = rospy.Publisher('/roll_error',Float32, queue_size=1) 
                self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1) 
                self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
                self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
                
                # ROS Subscribers
                rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
                rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
                rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
                rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
                rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        
        # Imu callback function
        def imu_callback(self, msg):
                self.drone_orientation_quaternion[0] = msg.orientation.x
                self.drone_orientation_quaternion[1] = msg.orientation.y
                self.drone_orientation_quaternion[2] = msg.orientation.z
                self.drone_orientation_quaternion[3] = msg.orientation.w

        # edrone_cmd values getting subscribe from position_controller.py
        def drone_command_callback(self, msg):
                self.setpoint_cmd[0] = msg.rcRoll
                self.setpoint_cmd[1] = msg.rcPitch
                self.setpoint_cmd[2] = msg.rcYaw
                self.setpoint_cmd[3] = msg.rcThrottle

        # Callback function for /pid_tuning_roll
        def roll_set_pid(self, roll):
                self.Kp[0] = roll.Kp * 1 
                self.Ki[0] = roll.Ki * 0.01
                self.Kd[0] = roll.Kd * 2
        # Callback function for /pid_tuning_pitch
        def pitch_set_pid(self, pitch):
                self.Kp[1] = pitch.Kp * 1 
                self.Ki[1] = pitch.Ki * 0.01
                self.Kd[1] = pitch.Kd * 2
        # Callback function for /pid_tuning_yaw
        def yaw_set_pid(self, yaw):
                self.Kp[2] = yaw.Kp * 1
                self.Ki[2] = yaw.Ki * 0.1
                self.Kd[2] = yaw.Kd * 1

        def calc_setpoint_euler(self):
                # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
                self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
                self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
                self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        def calc_error(self):
                self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
                self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
                self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

        def calc_sum_error(self):
                self.sum_error[0] = self.sum_error[0] + self.error[0]
                self.sum_error[1] = self.sum_error[1] + self.error[1]
                self.sum_error[2] = self.sum_error[2] + self.error[2]

        def calc_proportional(self):
                #proportional error [pitch,roll, yaw]
                self.roll_kp = self.Kp[0] * self.error[0]
                self.pitch_kp = self.Kp[1] * self.error[1]
                self.yaw_kp = self.Kp[2] * self.error[2]

        def calc_derivative(self):
                #derivative error [pitch,roll, yaw]
                self.roll_kd = self.Kd[0] * (self.error[0] - self.prev_error[0])   
                self.pitch_kd = self.Kd[1] * (self.error[1] - self.prev_error[1]) 
                self.yaw_kd = self.Kd[2] * (self.error[2] - self.prev_error[2])

        def calc_integral(self):
                #integrated error [pitch,roll, yaw]
                self.roll_ki = self.sum_error[0] * self.Ki[0]
                self.pitch_ki = self.sum_error[1] * self.Ki[1]
                self.yaw_ki = self.sum_error[2] * self.Ki[2]

        def calc_pid_output(self):
                # Calculate total pid output required for each axis
                self.out_roll = self.roll_kp + self.roll_kd + self.roll_ki
                self.out_pitch = self.pitch_kp + self.pitch_kd + self.pitch_ki
                self.out_yaw = self.yaw_kp + self.yaw_kd + self.yaw_ki
                self.out_throttle = ((self.setpoint_cmd[3]/1000)-1) * 1023        #1000-2000 map 0-1023
        
        def calc_pwm_prop(self):
                self.pwm_cmd.prop1 = self.out_throttle - self.out_roll + self.out_pitch - self.out_yaw
                self.pwm_cmd.prop2 = self.out_throttle - self.out_roll - self.out_pitch + self.out_yaw
                self.pwm_cmd.prop3 = self.out_throttle + self.out_roll - self.out_pitch - self.out_yaw
                self.pwm_cmd.prop4 = self.out_throttle + self.out_roll + self.out_pitch + self.out_yaw

        def limit_prop_speed(self):
                if self.pwm_cmd.prop1 > self.max_values[0]:
                        self.pwm_cmd.prop1 = self.max_values[0]
                if self.pwm_cmd.prop2 > self.max_values[1]:
                        self.pwm_cmd.prop2 = self.max_values[1]
                if self.pwm_cmd.prop3 > self.max_values[2]:
                        self.pwm_cmd.prop3 = self.max_values[2]
                if self.pwm_cmd.prop4 > self.max_values[3]:
                        self.pwm_cmd.prop4 = self.max_values[3]

                if self.pwm_cmd.prop1 < self.min_values[0]:
                        self.pwm_cmd.prop1 = self.min_values[0]
                if self.pwm_cmd.prop2 < self.min_values[1]:
                        self.pwm_cmd.prop2 = self.min_values[1]
                if self.pwm_cmd.prop3 < self.min_values[2]:
                        self.pwm_cmd.prop3 = self.min_values[2]
                if self.pwm_cmd.prop4 < self.min_values[3]:
                        self.pwm_cmd.prop4 = self.min_values[3]

        def calc_prev_error(self):
                self.prev_error[0] = self.error[0]
                self.prev_error[1] = self.error[1]
                self.prev_error[2] = self.error[2]

        def publish_val(self):
                self.pwm_pub.publish(self.pwm_cmd)
                self.roll_error_pub.publish(self.error[0])
                self.pitch_error_pub.publish(self.error[1])
                self.yaw_error_pub.publish(self.error[2])
                self.zero_error_pub.publish(0)

    # main function 
        def pid(self):
                # Converting quaternion to euler angles
                (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
                
                self.calc_setpoint_euler()
                self.calc_error()
                self.calc_sum_error()
                self.calc_proportional()
                self.calc_derivative()
                self.calc_integral()
                self.calc_pid_output()
                self.calc_pwm_prop()
                self.limit_prop_speed()      # Limit the output value between the maximum(0) and minimum(1024)range
                self.calc_prev_error()       # Update previous errors
                self.publish_val()           # Publishing the values  

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time) 
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
