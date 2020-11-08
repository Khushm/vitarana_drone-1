from vitarana_drone.msg import *
from pid_tune.msg import *
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float64
import rospy
import time
import tf

class Edrone():
    def __init__(self):
        rospy.init_node('position controller')

        self.drone_position = [19.0,72.0,0.31]
        self.setpoint_cmd = [19.0, 72.0, 3.0]
        self.Kp = [0,0,0,0]
        self.Kd = [0,0,0,0]
        self.Ki = [0,0,0,0]

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500

        self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        self.sample_time = 0.060

        self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1) 
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1) 
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float64, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float64, queue_size=1)
        self.z_error_pub = rospy.Publisher('/throttle_error', Float64, queue_size=1)

        #rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_throttle', PidTune, self.throttle_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)

    def roll_set_pid(self,roll):
        self.Kp[0] = roll.Kp * 0.0006 
        self.Ki[0] = roll.Ki * 0.00008
        self.Kd[0] = roll.Kd * 0.003
	print("hhh")

    def pitch_set_pid(self,pitch):
        self.Kp[1] = pitch.Kp * 0.0006 
        self.Ki[1] = pitch.Ki * 0.00008
        self.Kd[1] = pitch.Kd * 0.003

    def yaw_set_pid(self,yaw):
        self.Kp[2] = yaw.Kp * 0.0006 
        self.Ki[2] = yaw.Ki * 0.00008
        self.Kd[2] = yaw.Kd * 0.003

    def throttle_set_pid(self,yaw):
        self.Kp[3] = yaw.Kp * 0.0006 
        self.Ki[3] = yaw.Ki * 0.00008
        self.Kd[3] = yaw.Kd * 0.003


    def pid():
        pass





if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        e_drone.pid()
        # r.sleep()
