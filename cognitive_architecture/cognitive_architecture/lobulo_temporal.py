#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import random as rd
from std_msgs.msg import Float32MultiArray
from arlo_interfaces.msg import EstadoArlo
from arlo_interfaces.msg import MatesOdom
from rclpy.executors import SingleThreadedExecutor

# def ejecutar_despues_del_constructor(cls):
#     original_init= cls.__init__
#     def wrapper(self,i, tipo, *args, **kwargs):
#             original_init(self,i, tipo, *args, **kwargs)
#             self.chequeo()  # Llama a la función chequeo después de la inicialización
#     cls.__init__ = wrapper
#     return cls

#@ejecutar_despues_del_constructor
class temporal_lobe(Node):
    
    def __init__(self,i, tipo, numMates):
        super().__init__("temporal_lobe_"+str(i)+tipo)

        self.i = i
        self.tipo = tipo
        self.numMates = numMates

        self.mensajes_recibidos_list=[0]* (4+numMates) #al 5 hay que sumarle la cantidad de compañeritos - 1
        self.mensajes_recibidos = 0
        self.mensajes_por_recibir = len(self.mensajes_recibidos_list)
        #self.posiciones_mates
        self.diccionario_sensores = {'link_sensor_center': 1, 'link_sensor_back': 2, 'link_sensor_right': 3, 'link_sensor_left': 4}

        self.publisher =  self.create_publisher(EstadoArlo,"robot"+str(i)+tipo+"/temporal_lobe_",10)
        self.suscriptor1 = self.create_subscription(LaserScan,"robot"+str(i)+tipo+"/ultrasonico1/out", self.procesa1, 10)
        self.suscriptor2 = self.create_subscription(LaserScan,"robot"+str(i)+tipo+"/ultrasonico2/out", self.procesa2, 10)
        self.suscriptor3 = self.create_subscription(LaserScan,"robot"+str(i)+tipo+"/ultrasonico3/out", self.procesa3, 10)
        self.suscriptor4 = self.create_subscription(LaserScan,"robot"+str(i)+tipo+"/ultrasonico4/out", self.procesa4, 10)
        self.suscriptorOdom = self.create_subscription(Odometry,"robot"+str(i)+tipo+"/odom", self.checaOdom, 10)
        self.susMates = self.create_subscription(MatesOdom,"MatesOdom", self.procesaMatesOdom, 10)
        self.pubMates =  self.create_publisher(MatesOdom,"MatesOdom",10)
        #self.nodoOdom = rclpy.create_node("oyente_odom_tp_"+str(i)+tipo)
        #suscriptorOdom = self.nodoOdom.create_subscription(Odometry,"robot"+str(i)+tipo+"/odom", self.checaOdom, 10)
        
        # self.executor = SingleThreadedExecutor()
        # self.executor.(self.nodoOdom)

        print("se hizo el constructor")


    def procesaMatesOdom(self, odoms:MatesOdom):
        
        split = list(odoms.name)
        
        #print(split[0])
        self.mensajes_recibidos_list[int(split[0])+3] = odoms.odom
        self.mensajes_recibidos +=1

        if(self.mensajes_recibidos == self.mensajes_por_recibir):
            self.procesaAll()
            self.mensajes_recibidos=0
    

    def checaOdom(self, odom:Odometry):
        estado = MatesOdom()
        estado.odom = odom
        estado.name = str(self.i)+str(self.tipo)
        self.pubMates.publish(estado)


    def procesa1(self, rango:LaserScan):  #link_sensor_center
        #print(rango.ranges)
        self.mensajes_recibidos_list[0]=rango
        self.mensajes_recibidos +=1
        if(self.mensajes_recibidos == self.mensajes_por_recibir):
            self.procesaAll()
            #self.mensajes_recibidos_list = [0] * 5
            self.mensajes_recibidos = 0


    def procesa2(self, rango:LaserScan):  #link_sensor_center
        #print(rango.ranges)
        self.mensajes_recibidos_list[1]=rango
        self.mensajes_recibidos +=1
        if(self.mensajes_recibidos == self.mensajes_por_recibir):
            self.procesaAll()
            #self.mensajes_recibidos_list = [0] * 5
            self.mensajes_recibidos = 0

    def procesa3(self, rango:LaserScan):  #link_sensor_center
        #print(rango.ranges)
        self.mensajes_recibidos_list[2]=rango
        self.mensajes_recibidos +=1
        if(self.mensajes_recibidos == self.mensajes_por_recibir):
            self.procesaAll()
            #self.mensajes_recibidos_list = [0] * 5
            self.mensajes_recibidos = 0
                

    def procesa4(self, rango:LaserScan):  #link_sensor_center
        #print(rango.ranges)
        self.mensajes_recibidos_list[3]=rango
        self.mensajes_recibidos +=1
        if(self.mensajes_recibidos == self.mensajes_por_recibir):
            self.procesaAll()
            #self.mensajes_recibidos_list = [0] * 5
            self.mensajes_recibidos = 0
    

    def calculaDistsPromedio(self):
        pass

    def procesaAll(self):
        estado = EstadoArlo()
        
        #rclpy.spin_once(self.nodoOdom)
        #self.executorspin_once()
        # aqui hay que poner un nodo en spin_once para obtener la posicion de arlo
        
        estado.sensor1 = self.mensajes_recibidos_list[0]
        estado.sensor2 = self.mensajes_recibidos_list[1]
        estado.sensor3 = self.mensajes_recibidos_list[2]
        estado.sensor4 = self.mensajes_recibidos_list[3]
        estado.odom= self.mensajes_recibidos_list[(int(self.i))+3]   # con self.i, obtenemos la posicion de la odometría que le corresponde a bot self

        # self.mensajes_recibidos_list = [0] * 5
        # self.mensajes_recibidos = 0

        # print("Estado a publicar:")
        # print("Sensor 1:", estado.sensor1)
        # print("Sensor 2:", estado.sensor2)
        # print("Sensor 3:", estado.sensor3)
        # print("Sensor 4:", estado.sensor4)
        print("Odometría:", estado.odom)

        # for mensaje in self.mensajes_recibidos_list:
        #     print(mensaje)
        
        self.publisher.publish(estado)
        
        # for i in range(0,5):
        #     if i == 4:
        #         estado.odom= self.mensajes_recibidos_list[i]
        #     else:
        #         estado.sensor1= self.mensajes_recibidos_list[i]
        #     self.publisher.publish(estado)
        
        
