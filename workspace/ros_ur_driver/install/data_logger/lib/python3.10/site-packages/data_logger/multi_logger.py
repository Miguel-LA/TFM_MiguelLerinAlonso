#!/usr/bin/env python3

# DESCRIPCIÓN:
# Este es un supernodo que puede invocar a varios nodos de una misma vez. Como primera prueba voy a tratar
# de hacer uno que pueda invocar a los nodos de data_emmiter y data_reader. Si todo sale bien lo que hago es 
# modificarlo para que se puedan invocar instancias de los lectores especializadas.



# Librerías esenciales de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re

# Bibliotecas para la importación de nodos secundarios.
import os
import sys 
import time
from threading import Thread
import subprocess

# Se importan los nodos secundarios.
sys.path.append(os.path.dirname(__file__))
from data_logger.joint_reader import jointReader


class SuperLogger(Node):

    # Método constructor. Usa construcción explícita.
    def __init__(self):
        super().__init__('multi_logger')
        self.iniciar_nodos_secundarios()

    def iniciar_nodos_secundarios(self):
        # SE abre un hilo para iniciar el llgger de joints.
        self.thread_joint= Thread(target=self.ejecutar_nodo, args=('./src/data_logger/data_logger/joint_reader.py',))
        self.thread_joint.start()

        # SE abre un hilo para iniciar el llgger de joints.
        self.thread_analog= Thread(target=self.ejecutar_nodo, args=('./src/data_logger/data_logger/analog_reader.py',))
        self.thread_analog.start()

        # SE abre un hilo para iniciar el llgger de joints.
        self.thread_digital= Thread(target=self.ejecutar_nodo, args=('./src/data_logger/data_logger/digital_reader.py',))
        self.thread_digital.start()


    def ejecutar_nodo(self, nombre_nodo):
        subprocess.run(['python3', nombre_nodo])

        


def main(args=None):
    rclpy.init(args=args)

    node=SuperLogger()

    # rclpy.spin(node)
    # rclpy.shutdown()


if __name__== '__main__':
    main()