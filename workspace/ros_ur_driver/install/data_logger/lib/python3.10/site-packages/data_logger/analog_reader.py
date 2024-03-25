#!/usr/bin/env python3

# DESCRIPCIÓN:
# Este nodo es un suscriptor que cuando se arranca puede tomar un número concreto
# de muestras y guardarlas en un fichero CSV. Este nodo está especializado para tomar datos anañógicos
# Los parámetros principales que usa son:
#
# * n_muestras: El número de muestras que se desea registrar. Por defecto se iniciará a 50.
# * ruta_guardado: La ruta del archivo donde se desea guardar el CSV resultado. Por defecto será
# '~/Desktop/datos_recopilados.csv'.
# * continuar_: Es un flag booleano que indica si se desea continua con la ejecución del nodo después
# de registrar las n_muestras. POr defecto se encuentra inicializado a False. En caso de inicializarse a True,
# el nodo continuará ejecutándose indicando la medida que toma y el número que le corresponde, pero NO LA AÑADIRÁ
# AL CSV DE SALIDA. Se indica el estado de ejecución del nodo al pasar el punto límite de muestras.
# * analog_output_pin: Número del pin de salida analógico del UR que se quiere leer por defecto se encuentra inicializado al 0. 
# * analog_input_pin: Número del pin de entrada analógico del UR que se quiere leer por defecto se encuentra inicializado al 0. 
# * io: Para indicar si en la instacia del nodo se desea rastrear entradas o salidas. Por defecto se encuentra inicializado para una salida analógica. Las palabras clave son input/output.
# * freq: Frecuencia de muestreo deseada en Hz. Por defecto se incia a 1 Hz.



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
import time


class AnalogReader(Node):

    # Método constructor. Usa construcción explícita.
    def __init__(self):
        super().__init__('analog_reader')

        # Se declaran los parámetros con su valor por defecto.
        self.declare_parameter('n_muestras', 50)
        self.declare_parameter('ruta_guardado', '~/Desktop/datos_recopilados_analog.csv')
        self.declare_parameter('continuar', True)
        self.declare_parameter('io', 'output')
        self.declare_parameter('representar', False)
        self.declare_parameter('analog_output_pin', 0)
        self.declare_parameter('analog_input_pin', 0)
        self.declare_parameter('freq', 1)


        # Atributos propios del nodo para trabajar. Se asocian los parámetros inicalizados.
        self.i=0    # Contador de muestras.
        self.mis_datos=[]   # Datos recopilados.
        self.numero_muestra=[]  # Número de muestra tomada desde que arranca el nodo.
        self.timer_period=1
        self.read_pin_state_aux=True
        self.read_pin_state=[]
        self.timestamps=[]  # Vector para guardar las marcas de tiempo.
        self.timestamps_aux=[]  # Vector auxiliar para guardar las marcas de tiempo.

        self.n_muestras=self.get_parameter('n_muestras').value  # Número muestras por registrar.
        self.ruta_guardado=self.get_parameter('ruta_guardado').value    # Ruta de guardado del CSV.
        self.continuar_=self.get_parameter('continuar').value   # Indicación de continuar la ejecución o no.
        self.io= self.get_parameter('io').value # Para saber si queremos entradas/salidas.
        self.representar= self.get_parameter('representar').value   # Indicar si se quieren representar en una gráfica los datos tomados o no.
        self.analog_output_pin= self.get_parameter('analog_output_pin').value  
        self.analog_input_pin= self.get_parameter('analog_input_pin').value  
        self.frecuencia_muestreo= self.get_parameter('freq').value
        self.timer_period= 1/self.frecuencia_muestreo

        # Se define el periodo y se invoca el temporizador.
        self.create_timer(self.timer_period, self.timmer_callback)

        self.arrancar_suscriptor()

    def arrancar_suscriptor(self):
        # Se estudia si se quieren entradas y salidas y se arranca el suscriptor.
        if self.io=='output':
            self.subscription_= self.create_subscription(IOStates,
                                                     '/io_and_status_controller/io_states',
                                                     self.analog_output_reader, 10)
        elif self.io=='input':
            self.subscription_= self.create_subscription(IOStates,
                                                     '/io_and_status_controller/io_states',
                                                     self.analog_input_reader, 10)
            

        self.subscription_

    def timmer_callback(self):
        self.get_logger().info('Muestra Nº: %d'%self.i)


        # Registro de pines analógicos.
        x_pin_state= self.read_pin_state_aux
        self.read_pin_state.append(x_pin_state)
        # print(x_pin_state)

        # Registro de marcas de tiempo.
        x_timestamp=self.timestamps_aux
        self.timestamps.append(x_timestamp)


        self.numero_muestra.append(self.i)


        if self.i==self.n_muestras:
            print('Ya he tomado la muestra %d y lo guardo en un CSV' % self.i)
            self.mi_tabla=pd.DataFrame({'N_muestra':self.numero_muestra,
                                        'Marca_tiempo': self.timestamps,
                                    'AO_state:': self.read_pin_state,
                                    })
            self.mi_tabla.to_csv(self.ruta_guardado)

        self.i+=1


    # Método de callback para tomar datos de las salidas analógicas.
    def analog_output_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.analog_output_pin}, domain=0, state=([\d.]+)"
        resultado= re.search(patron, str(msg.analog_out_states))
        # print(resultado.group(1))
        if resultado:
            self.read_pin_state_aux= resultado.group(1)
        else:
            print('No se encuentra el valor.')

        # Se guarda la marca de tiempo de cada medida (sincornizada con el reloj del ordenador)
        self.timestamps_aux= time.time()
        self.timestamps_aux= self.formato_timestamp(self.timestamps_aux) # Esta línea formatea las marcas de tiempo para HH:mm:ss:decimasSegundo. Si se desea que no ocurra comentar esta línea.

    # Método de callback para tomar datos de las entradas analógicas.
    def analog_input_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.analog_input_pin}, domain=0, state=([\d.]+)"
        resultado= re.search(patron, str(msg.analog_in_states))

        if resultado:
            self.read_pin_state_aux= resultado.group(1)
        else:
            print('No se encuentra el valor.')

        
        # Se guarda la marca de tiempo de cada medida (sincornizada con el reloj del ordenador)
        self.timestamps_aux= time.time()
        self.timestamps_aux= self.formato_timestamp(self.timestamps_aux) # Esta línea formatea las marcas de tiempo para HH:mm:ss:decimasSegundo. Si se desea que no ocurra comentar esta línea.



    def formato_timestamp(self, timestamp_):

        # Se separa la marca de tiempo en la parte entera y la parte decimal.
        timestamp_int=int(timestamp_)
        timestamp_frac=timestamp_ - timestamp_int

        # Se convierte la la parte eentera de la marca de tiempo en formato universal UNIX a formato local.
        time_struct=time.localtime(timestamp_int)

        # Se define el formato de tiempo en el que se desea mostrar la marca de tiempo.
        formato_tiempo=time.strftime('%H:%M:%S', time_struct) + ':' + f".{10*timestamp_frac:.5f}"[1:]

        return formato_tiempo



def main(args=None):
    rclpy.init(args=args)

    node=AnalogReader()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()