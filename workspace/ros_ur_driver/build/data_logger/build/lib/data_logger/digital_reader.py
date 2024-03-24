#!/usr/bin/env python3

# DESCRIPCIÓN:
# Este nodo es un suscriptor que cuando se arranca puede tomar un número concreto 
# de muestras y guardarlas en un fichero CSV. Este nodo está especializado para leer datos digitales. 
# Los parámetros principales que usa son:
#
# * n_muestras: El número de muestras que se desea registrar. Por defecto se iniciará a 50.
# * ruta_guardado: La ruta del archivo donde se desea guardar el CSV resultado. Por defecto será
# '~/Desktop/datos_recopilados_digital.csv'. 
# * continuar_: Es un flag booleano que indica si se desea continua con la ejecución del nodo después 
# de registrar las n_muestras. POr defecto se encuentra inicializado a False. En caso de inicializarse a True, 
# el nodo continuará ejecutándose indicando la medida que toma y el número que le corresponde, pero NO LA AÑADIRÁ
# AL CSV DE SALIDA. Se indica el estado de ejecución del nodo al pasar el punto límite de muestras.
# * digital_output_pin: Número del pin de salida digital del UR que se quiere leer por defecto se encuentra inicializado al 4. 
# * digital_input_pin: Número del pin de entrada digital del UR que se quiere leer por defecto se encuentra inicializado al 4. 
# * io: Para indicar si en la instacia del nodo se desea rastrear entradas o salidas. Por defecto se encuentra inicializado para una salida digital. Las palabras clave son input/output.
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
import re   # Para hacer formateo de cadenas de texto
import time # Para implementar marcas de tiempo en el muestreo.


class DigitalReader(Node):

    # Método constructor. Usa construcción explícita.
    def __init__(self):
        super().__init__('digital_reader')

        # Se declaran los parámetros con su valor por defecto.
        self.declare_parameter('n_muestras', 50)
        self.declare_parameter('ruta_guardado', '~/Desktop/datos_recopilados_digital.csv')
        self.declare_parameter('continuar', True)
        self.declare_parameter('io', 'output')
        self.declare_parameter('digital_output_pin', 4)
        self.declare_parameter('digital_input_pin', 4)
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
        self.io=self.get_parameter('io').value   # Indicación de continuar la ejecución o no.
        self.digital_output_pin= self.get_parameter('digital_output_pin').value  
        self.digital_input_pin= self.get_parameter('digital_input_pin').value  
        self.frecuencia_muestreo= self.get_parameter('freq').value
        self.timer_period= 1/self.frecuencia_muestreo
        

        # Se define el periodo de muestreo y se llama al temporizador.
        self.create_timer(self.timer_period, self.timmer_callback)

        self.arrancar_suscriptor()

    def arrancar_suscriptor(self):
        # Se arranca el nodo como suscriptor para entradas/salidas según palabra clave inicializada.
        if self.io=='output':
            self.subscription_= self.create_subscription(IOStates, 
                                                     '/io_and_status_controller/io_states', 
                                                     self.digital_output_reader, 10)
        elif self.io=='input':
            self.subscription_= self.create_subscription(IOStates, 
                                                     '/io_and_status_controller/io_states', 
                                                     self.digital_input_reader, 10)
        self.subscription_

    # Función del temporizador
    def timmer_callback(self):
        self.get_logger().info('Muestra Nº: %d'%self.i)
       
       # Registro de estados del pin digital
        x_pin_state= self.read_pin_state_aux
        self.read_pin_state.append(x_pin_state)

        # Registro de marcas de tiempo.
        x_timestamp=self.timestamps_aux
        self.timestamps.append(x_timestamp)


        self.numero_muestra.append(self.i)

        # Aquií se guarda el CSV.
        if self.i==self.n_muestras:
            print('Ya he tomado la muestra %d y lo guardo en un CSV' % self.i)
            self.mi_tabla=pd.DataFrame({'N_muestra':self.numero_muestra,
                                        'Marca_tiempo': self.timestamps,
                                    'DO_state:': self.read_pin_state,
                                    })
            self.mi_tabla.to_csv(self.ruta_guardado)

        self.i+=1

    # Método de callback para tomar datos de las salidas digitales.
    def digital_output_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.digital_output_pin}, state=(True|False)"
        resultado= re.search(patron, str(msg.digital_out_states))
        resultado=resultado.group(0)


        # Para guardar resultado del pin
        patron=r'(True|False)'
        self.read_pin_state_aux=re.search(patron, str(resultado)).group() # Se invoca al método group para que se guarde sólo el valor solicitado.

        # Se guarda la marca de tiempo de cada medida (sincornizada con el reloj del ordenador)
        self.timestamps_aux= time.time()
        self.timestamps_aux= self.formato_timestamp(self.timestamps_aux) # Esta línea formatea las marcas de tiempo para HH:mm:ss:decimasSegundo. Si se desea que no ocurra comentar esta línea.
          
        

    # # Método de callback para leer las entradas digitales.
    def digital_input_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.digital_input_pin}, state=(True|False)"
        resultado= re.search(patron, str(msg.digital_in_states))
        resultado=resultado.group(0)


        # Para guardar resultado del pin
        patron=r'(True|False)'
        self.read_pin_state_aux=re.search(patron, str(resultado)).group() # Se invoca al método group para que se guarde sólo el valor solicitado.
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

    node=DigitalReader()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()