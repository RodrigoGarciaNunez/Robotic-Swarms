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

def main(args=None):
    rclpy.init(args=args) #inicia comunicaciones de ros2

    nodos_control=[]
    nodos_tl=[]
    for i in range(1,3):
        nodo = ci(i)
        lobulo_t = tl(i)
        nodos_control.append(nodo)
        nodos_tl.append(lobulo_t)

    rclpy.spin(nodos_control[0])
    rclpy.spin(nodos_tl[0])


    rclpy.shutdown() #termina comunicaciones de ros2


if __name__ == '__main__':
    main()

