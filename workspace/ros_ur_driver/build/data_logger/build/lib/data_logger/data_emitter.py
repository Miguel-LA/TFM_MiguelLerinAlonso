#!/usr/bin/env python3

# Librerías de ros 2 a incluir.
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Librerías típicas de python.
import matplotlib.pyplot as plt 
import pandas as pd
import numpy as np


# Se define el nodo que crea una lista numérica en forma de seno y no para de sacarla por pantalla.
# Este nodo se puede asumir como un publisher al que se pueden suscribir otros nodos y operar con ello.
class senoNode(Node):

    def __init__(self):
        super().__init__('data_emitter')

        self.publisher_= self.create_publisher(String, 'Manolito', 10)
        self.timer_= self.create_timer(0.5, self.timer_callback)
        self.i=0

    def timer_callback(self):
        msg= String()
        msg.data= str(np.sin(self.i))
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando seno de t: %s' % msg.data)
        self.i+=1





def main(args=None):
    
    rclpy.init(args=args)

    node=senoNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()

# OBJETIVO: Voy a tratar de hacer una arquitectura básica en el que tenga un 
# sólo nodo que haga de publisher y luego otros dos subscribers. De los otros dos suscribers, uno simplemente leera 
# la salida y la sacará por pantalla mientras que el otro la multiplicará por 3 y la sacará por pantalla.