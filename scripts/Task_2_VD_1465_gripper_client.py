#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String,Float32
from vitarana_drone.msg import value

def callback_service_client(activate):
    rospy.wait_for_service('/edrone/activate_gripper')
    resp_error_pub = rospy.Publisher("resp_error", value, queue_size=1)
    try:
        gripper_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
	obj_value = value()

        resp = gripper_activate(activate)
	obj_value.res = resp.result
	print(obj_value.res)

	resp_error_pub.publish(obj_value)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    while not rospy.is_shutdown():
        try:
	    rospy.init_node('gripper_client')
            callback_service_client(activate=True)
        except rospy.ROSInterruptException:
            rospy.logerr("Shtdown Req")
if __name__ == "__main__":
    main()
