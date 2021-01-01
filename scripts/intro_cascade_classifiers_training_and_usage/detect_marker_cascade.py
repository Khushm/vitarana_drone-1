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
import cv2
from matplotlib import pyplot as plt

class Edrone():
    self.sample_time = 0.01
    rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
    self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            logo_cascade = cv2.CascadeClassifier('/home/khush/catkin_ws/src/vitarana_drone/scripts/intro_cascade_classifiers_training_and_usage/data/cascade.xml')
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            image = self.img
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
            for (x, y, w, h) in logo:
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
                plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                plt.show()
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
            return
        
if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()


# img = cv2.imread('/home/khush/catkin_ws/src/vitarana_drone/scripts/intro_cascade_classifiers_training_and_usage/test_3.png')  # Source image



