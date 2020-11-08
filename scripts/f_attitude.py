#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf

class Edrone():
    def __init__(self):

        rospy.init_node('attitude_controller')

        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]
        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 510.0
        self.pwm_cmd.prop2 = 510.0
        self.pwm_cmd.prop3 = 510.0
        self.pwm_cmd.prop4 = 510.0

        self.sample_time = 0.060

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
    def roll_set_pid(self,roll):
        self.Kp[0] = roll.Kp * 0.1
        self.Kd[1] = roll.Kp * 0.02
        self.Ki[2] = roll.Kp * 0.001

    def roll_set_pid(self,roll):
        self.Kp[0] = roll.Kp * 0.1
        self.Kd[1] = roll.Kp * 0.02
        self.Ki[2] = roll.Kp * 0.001


    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        print("imu")		
        rospy.sleep(1)

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = 1500
        self.setpoint_cmd[1] = 1500
        self.setpoint_cmd[2] = 1500
        print("drone")		
        rospy.sleep(1)

    def pid(self):
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        rospy.loginfo("quaternion %0.2f",self.drone_orientation_quaternion[0])
        rospy.loginfo("conversioneular %0.2f",self.drone_orientation_euler[0])
        #print("setpoint",self.setpoint_cmd[0])
        if(self.altitude > 5):
            self.pwm_cmd.prop1 = 510.0
            self.pwm_cmd.prop2 = 510.0
            self.pwm_cmd.prop3 = 510.0
            self.pwm_cmd.prop4 = 510.0
            self.pwm_pub.publish(self.pwm_cmd)
         

if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()


