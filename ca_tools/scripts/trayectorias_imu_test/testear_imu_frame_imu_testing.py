#!/usr/bin/env python  
import roslib
import rospy
import tf
import turtlesim.msg
from sensor_msgs.msg import Imu
import math

def handler_imu(msg):
    #print msg.orientation
    # Publicamos la traslacion y rotacion como una transformacion de imu_link a testing_link
    br = tf.TransformBroadcaster()
    br.sendTransform(
        # tf2_msgs/TFMessage
        # Traslacion x, y, z
        (0, 0, 0),
        # Rotacion cuaternion x, y, z, w
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        # time stamp
        rospy.Time.now(),
        # child_frame_id
        'testing_link',
        # frame_id
        'imu_link')

if __name__ == '__main__':
    rospy.init_node('probador_imu')
    # No subscribimos al topic /turtle#/pose. Los parametros son(nombre del topic, tipo de mensaje, handler, argumentos adicionales para el handler)
    rospy.Subscriber('/imu/data', Imu, handler_imu)
    
    #br = tf.TransformBroadcaster()
    # Se queda esperando mensajes
    rospy.spin()