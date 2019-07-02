#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

# 0.1 metros por segundo
VEL_LIN = 0.1
# 10 grados por segundo
VEL_ANG = 0.174527

class MoverRobot():
    def __init__(self):
        rospy.init_node("node_name")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def avanzar(self):
        self.pub.publish(Twist(Vector3(VEL_LIN, 0, 0), Vector3(0, 0, 0)))

    def retroceder(self):
        self.pub.publish(Twist(Vector3(-VEL_LIN, 0, 0), Vector3(0, 0, 0)))
    
    def rotar_izq(self):
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, VEL_ANG)))
        
    def rotar_der(self):
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -VEL_ANG)))
        
    def parar(self):
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

if __name__ == "__main__":
    contador = 0
    mb = MoverRobot()
    rate = rospy.Rate(10)   # 100ms
    while not rospy.is_shutdown() and contador<630:
        #print(contador)
        if contador == 10:
            print('etapa1')
            mb.avanzar()
            #avanzo 1m
        elif contador == 110:
            print('etapa2')
            mb.parar()
            mb.rotar_izq()
            #roto 90 grados izquierda
        elif contador == 200:
            print('etapa3')
            mb.parar()
            mb.avanzar()
            #avanzo 0.5m
        elif contador == 250:
            print('etapa4')
            mb.parar() 
            mb.rotar_izq()
            #roto 90 grados izquierda
        elif contador == 340:
            print('etapa5')
            mb.parar()
            mb.avanzar()
            #avanzo 1m
        elif contador == 450:
            print('etapa6')
            mb.parar()
            mb.rotar_izq()
            #roto 90 grados izquierda
        elif contador == 540:
            print('etapa7')
            mb.parar()
            mb.avanzar()
            #avanzo 0.1m
        contador += 1
        rate.sleep()
    mb.parar()