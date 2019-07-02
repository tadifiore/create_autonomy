#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class ImuConverter(object):

    def __init__(self):
        rospy.init_node("imu_converter", log_level=rospy.INFO)
        self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.__imu_cb__)
        self.sub_angle = rospy.Subscriber("/angle", PoseWithCovarianceStamped, self.__angle_cb__)
        rospy.spin()
    
    def __imu_cb__(self, msg):
        q = msg.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        rospy.loginfo("[IMU] X:{:.2f} Y:{:.2f} Z:{:.2f}".format(math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2])))

    def __angle_cb__(self, msg):
        q = msg.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        rospy.loginfo("[ANGLE] X:{:.2f} Y:{:.2f} Z:{:.2f}".format(math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2])))

if __name__ == "__main__":
    ImuConverter()