#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

trayectorias = [[1,0], [1,0.5], [0,0.5], [0,0]]
puntos = []
for i in range(len(trayectorias)):
    p = Point()
    p.x = trayectorias[i][0]
    p.y = trayectorias[i][1]
    puntos.append(p)
print trayectorias
print puntos

class MoverRobot():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.r = rospy.Rate(4)
        #self.sub = rospy.Subscriber("/odom", Odometry, self.leer_odometria)
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.leer_odometria)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    def leer_odometria(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def mover(self, punto):
        inc_x = punto.x - self.x
        inc_y = punto.y - self.y
        vel = Twist()
        while abs(inc_x) > 0.05  or abs(inc_y) > 0.05:  
            #print("meta({},{}) Odom({},{})".format(punto.x, punto.y, self.x, self.y))
            
            angulo_meta = atan2(inc_y, inc_x)
            print (self.theta)
            print (angulo_meta)

            if (angulo_meta - self.theta) > 0.1:
                vel.linear.x = 0.0
                vel.angular.z = 0.3
            elif (angulo_meta - self.theta) < -0.1:
                vel.linear.x = 0.0
                vel.angular.z = -0.3
            else:
                vel.linear.x = 0.5
                vel.angular.z = 0.0
            self.pub.publish(vel)
            self.r.sleep()
            inc_x = punto.x - self.x
            inc_y = punto.y - self.y
        vel.linear.x = 0.0
        vel.angular.z = 0.0 
        self.pub.publish(vel)

if __name__ == "__main__":
    rospy.init_node("testear_odometria")
    mb = MoverRobot()
    for i in range(len(puntos)):
        print("Yendo a la coordenada {},{}".format(puntos[i].x, puntos[i].y))
        mb.mover(puntos[i])
    #while not rospy.is_shutdown():
    rospy.spin()
        


       