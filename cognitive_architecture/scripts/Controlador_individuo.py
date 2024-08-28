#!/usr/bin/env python3

import os
import rclpy
from rclpy import executors
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import sys
import time
#import keyboard
import random as rd
from threading import Thread
from cognitive_architecture.lobulo_temporal import temporal_lobe as tl
from cognitive_architecture.corteza_motora_primaria import Nodo as ci 

#from arlo_interfaces.msg import PesosStruct

def main(args=None):
    rclpy.init(args=args) #inicia comunicaciones de ros2

    if args is None:
        args = sys.argv[1:] 
    
    if args[0]=="1":
        num_bots=3

    else:
        num_bots=int(args[1])

    nodos_control=[]
    nodos_tl=[]
    for i in range(1,num_bots+1):
        nodo = ci(i,args[0])
        lobulo_t = tl(i,args[0],num_bots)
        nodos_control.append(nodo)
        nodos_tl.append(lobulo_t)

    #print("perrrro voy bien")
    executor = executors.MultiThreadedExecutor()
    #print("perrrro voy bien 1")
    for i in range(len(nodos_control)):
        executor.add_node(nodos_control[i])
        executor.add_node(nodos_tl[i])

    #print("perrrro voy bien 2")
    hilo = Thread(target=executor.spin)
    #print("perrrro voy bien 3")
    hilo.start()
    #print("perrrro voy bien 4")
    hilo.join()
    #print("perrrro voy bien 5")

    rclpy.shutdown() #termina comunicaciones de ros2


if __name__ == '__main__':
    main()