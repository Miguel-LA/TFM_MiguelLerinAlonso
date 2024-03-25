#!/usr/bin/env python3

# DESCRIPCIÓN:
# Este nodo es un suscriptor que cuando se arranca puede tomar un número concreto 
# de muestras y guardarlas en un fichero CSV. Los parámetros principales que usa son:
#
# * n_muestras: El número de muestras que se desea registrar. Por defecto se iniciará a 50.
# * ruta_guardado: La ruta del archivo donde se desea guardar el CSV resultado. Por defecto será
# '~/Desktop/datos_recopilados.csv'. 
# * continuar_: Es un flag booleano que indica si se desea continua con la ejecución del nodo después 
# de registrar las n_muestras. POr defecto se encuentra inicializado a False. En caso de inicializarse a True, 
# el nodo continuará ejecutándose indicando la medida que toma y el número que le corresponde, pero NO LA AÑADIRÁ
# AL CSV DE SALIDA. Se indica el estado de ejecución del nodo al pasar el punto límite de muestras.



# Librerías esenciales de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from sensor_msgs.msg._joint_state import JointState  # noqa: F401
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re


class lectorNode(Node):

    # Método constructor. Usa construcción explícita.
    def __init__(self):
        super().__init__('data_reader')

        # Se declaran los parámetros con su valor por defecto.
        self.declare_parameter('n_muestras', 50)
        self.declare_parameter('ruta_guardado', '~/Desktop/datos_recopilados.csv')
        self.declare_parameter('continuar', True)
        self.declare_parameter('representar', False)

        # Se arranca el nodo como suscriptor.
        self.subscription_= self.create_subscription(JointState
, '/joint_states', self.data_reader_callback, 10)
        self.subscription_

        # Atributos propios del nodo para trabajar. Se asocian los parámetros inicalizados.
        self.i=0    # Contador de muestras.
        self.mis_datos=[]   # Datos recopilados.
        self.numero_muestra=[]  # Número de muestra tomada desde que arranca el nodo.

        self.n_muestras=self.get_parameter('n_muestras').value  # Número muestras por registrar.
        self.ruta_guardado=self.get_parameter('ruta_guardado').value    # Ruta de guardado del CSV.
        self.continuar_=self.get_parameter('continuar').value   # Indicación de continuar la ejecución o no.
        self.representar= self.get_parameter('representar').value   # Indicar si se quieren representar en una gráfica los datos tomados o no.


    # Método de callback para tomar datos.
    def data_reader_callback(self, msg):
        self.get_logger().info('Estoy oyendo %s y tomo la muestra %d' %(msg.position, self.i)) # Como quiero saber la posición articular le pido la posición.


        self.coord=re.findall(r'[-+]?\d*\.\d+|\d+', str(msg.position))

        # self.coord= self.coord.strip('[]').split(', ')
        self.coord=[float(coor) for coor in self.coord]
      

        self.mis_datos.append(self.coord)
        self.numero_muestra.append(self.i)
        if self.i==self.n_muestras:
            print('Ya he tomado la muestra %d y lo guardo en un CSV' % self.i)


            self.mi_tabla=pd.DataFrame({'N_muestra':self.numero_muestra,
                                    'valor_muestra': self.mis_datos})
            self.mi_tabla.to_csv(self.ruta_guardado)

            if self.representar==True:  
                self.representar_muestras(self.mi_tabla)

            if self.continuar_==False:
                print('La recopilación de datos ha finalizado. Se destruirá el nodo.')
                self.destroy_node()
                rclpy.shutdown()
            else:
                print('La recopilación de datos ha finalizado pero se continúa con la escucha del nodo.')

        self.i+=1

         





def main(args=None):
    rclpy.init(args=args)

    node=lectorNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()