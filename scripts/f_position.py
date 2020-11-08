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




if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep(1)
