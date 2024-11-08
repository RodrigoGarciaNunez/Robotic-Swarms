#!/usr/bin/env python3
#hola
import os
import rclpy
from rclpy import executors
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range  #este tiene que cambiar a LaserScan
from sensor_msgs.msg import LaserScan
import random as rd
from threading import Thread
from gazebo_msgs.msg import ModelStates

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64MultiArray, Int64


#entradas: s1(todos los rayitos), s2, s3, s4, x, y
#salidas: vel_lineal_x, vel_angular_z

#este modulo se encarga de enviar las instrucciones a las ruedas del robot a través del plugin del differential drive
class CortezaMotoraPrimaria(Node):

    msg=Twist()
    #constructor
    def __init__(self,i,tipo):
        super().__init__("corteza_motora_primaria_"+str(i)+tipo)
        
        self.ignoreFlag = 0
        self.mensajes_recibidos=[]
        self.diccionario_sensores = {'link_sensor_center': 1, 'link_sensor_back': 2, 'link_sensor_right': 3, 'link_sensor_left': 4}

        self.publisher =  self.create_publisher(Twist, "robot"+str(i)+tipo+"/cmd_vel",10)

        self.suscriptorNN= self.create_subscription(Float64MultiArray,"robot"+str(i)+tipo+"/corteza_motora_secundariaNN", self.moverNN, 1)

        self.subsIgnoreFlag = self.create_subscription(Int64, "robot"+str(i)+tipo+"/ignoreFlag", self.SetIgnoreFlag, 10)

    def SetIgnoreFlag(self, mensaje:Int64):
        self.ignoreFlag = mensaje.data

    def moverNN(self, mensaje:Float64MultiArray):
        movimiento=Twist()

        if self.ignoreFlag == 1:
            movimiento.linear.x=0.0
            movimiento.angular.z=0.0
        else:
            movimiento.linear.x=mensaje.data[0]
            movimiento.angular.z=mensaje.data[1]
        
        self.publisher.publish(movimiento)


    def mover(self, mensaje:LaserScan):
       #este pass se comenta
        #print("a ver")
        self.mensajes_recibidos.append(mensaje)
        if len(self.mensajes_recibidos) < 4:   #si no ha recibido los 4 mensajes, no se mueve
            return 0
        
        sensor_menor_rango=10  #maximo artifial
        sensor_save:Range

        sensor_save=rd.choice(self.mensajes_recibidos)

        for mensaje in self.mensajes_recibidos:
            mensaje:Range

            if mensaje.range < sensor_menor_rango and mensaje.range>0.2:
                sensor_menor_rango = mensaje.range
                sensor_save=mensaje

        #print("me movi "+ self.get_name())
        #print(sensor_save.range)
            
            #elec = rd.randint(1,5)
        elec= self.diccionario_sensores[sensor_save.header.frame_id]

        if elec== 1:
            self.avanza()
        elif elec == 2: 
            self.atras()
        elif elec==3:
            self.derecha()
        elif elec==4:
            self.izquierda()
        elif elec ==5:
            self.parar()
        else:
            pass
            
        self.mensajes_recibidos.clear()
            #time.sleep(2)


    def avanza(self):

        #msg = Twist()
        self.msg.linear.x = 0.2  # velocidad lineal en el eje x
        self.msg.angular.z = 0.0  # velocidad angular en el eje z

        self.publisher.publish(self.msg)

    def derecha(self):

        #msg = Twist()
        self.msg.linear.x = 0.0  # velocidad lineal en el eje x
        self.msg.angular.z = -0.2  # velocidad angular en el eje z

        self.publisher.publish(self.msg)

    def izquierda(self):

        #msg = Twist()
        self.msg.linear.x = 0.0  # velocidad lineal en el eje x
        self.msg.angular.z = 0.2  # velocidad angular en el eje z  

        self.publisher.publish(self.msg)

    def atras(self):
        #msg = Twist()
        self.msg.linear.x = -0.2  # velocidad lineal en el eje x
        self.msg.angular.z = 0.0  # velocidad angular en el eje z

        self.publisher.publish(self.msg)

    def parar(self):
        #msg = Twist()
        self.msg.linear.x = 0.0  # velocidad lineal en el eje x
        self.msg.angular.z = 0.0  # velocidad angular en el eje z

        self.publisher.publish(self.msg)
    
    def teclado(self, e):  # la e es de evento
        if e.name == 'up':
            self.avanza() 
        elif e.name == 'down':
            self.parar()  
        elif e.name == 'right':
            self.derecha() 
        elif e.name == 'left':
            self.izquierda()


