#!/usr/bin/env python3

# # DESCRIPCIÓN
# Paquete de ros2 para la lectura de datos directos del UR. Este paquete depende de la conexión del robot al computador mediante movit2 y rviz, ya que se sirve de muchos de los tipos de mensajes y servicios proporcionados. Este paquete puede registrar:
# * Coordenadas articulares (posición, velocidad y esfuerzo en N-m)
# * Valor de las entradas/salidas digitales deseadas
# * Valor de las entradas/salidas analógicas deseadas
# Recordar que por defecto sólo se registra UNA salida digital y otra analógica, en caso de necesitar otra salida, se debe modificar el parámetro del nodo necesario para ello. 

# Los parámetros de entrada del nodo son:

#  * n_muestras: El número de muestras que se desea registrar. Por defecto se iniciará a 50.
#  * ruta_guardado: La ruta del archivo donde se desea guardar el CSV resultado. Por defecto será
#  '~/Desktop/datos_super_logger.csv'. 
#  * continuar_: Es un flag booleano que indica si se desea continua con la ejecución del nodo después de registrar las n_muestras. POr defecto se encuentra inicializado a False. En caso de inicializarse a True, el nodo continuará ejecutándose indicando la medida que toma y el número que le corresponde, pero NO LA AÑADIRÁ
#  AL CSV DE SALIDA. Se indica el estado de ejecución del nodo al pasar el punto límite de muestras.
#  * digital_output_pin: Número del pin de salida digital del UR que se quiere leer por defecto se encuentra inicializado al 4. 
#  * digital_input_pin: Número del pin de entrada digital del UR que se quiere leer por defecto se encuentra inicializado al 4. 
#   * analog_output_pin: Número del pin de salida analógico del UR que se quiere leer por defecto se encuentra inicializado al 0. 
#  * analog_input_pin: Número del pin de entrada analógico del UR que se quiere leer por defecto se encuentra inicializado al 0. 
#  * io: Para indicar si en la instacia del nodo se desea rastrear entradas o salidas. Por defecto se encuentra inicializado para una salida digital. Las palabras clave son input/output.
#  * freq: Frecuencia de muestreo deseada en Hz. Por defecto se incia a 1 Hz.



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
from tqdm import tqdm

class SuperLogger(Node):

    # Método constructor. Usa construcción explícita.
    def __init__(self):
        super().__init__('super_logger')

        # Se declaran los parámetros con su valor por defecto.
        self.declare_parameter('n_muestras', 50)
        self.declare_parameter('ruta_guardado', '~/Desktop/datos_super_logger_ingenia.csv')
        self.declare_parameter('continuar', True)
        self.declare_parameter('io', 'output')

        # Definción de tipos de entradas
        self.declare_parameter('analog_output_pin', 0)
        self.declare_parameter('analog_input_pin', 0)
        self.declare_parameter('digital_output_pin', 4)
        self.declare_parameter('digital_input_pin', 4)

        self.declare_parameter('freq', 1)


        # Atributos propios del nodo para trabajar. Se asocian los parámetros inicalizados.
        self.i=0    # Contador de muestras.
        self.mis_datos=[]   # Datos recopilados.
        self.numero_muestra=[]  # Número de muestra tomada desde que arranca el nodo.
        self.timer_period=1

        # Lecturas de pines Analogicos y Digitales.
        self.read_pinA_state_aux=True
        self.read_pinA_state=[]
        self.read_pinD_state_aux=True
        self.read_pinD_state=[]

        # Atributos de las joints
        self.joint_pos=[]   # Posiciones articulares
        self.joint_vel=[]   # Velocidades articulares
        self.joint_eff=[]   # Esfuerzos articulares
        self.joint_pos_aux=[]   # Posiciones articulares
        self.joint_vel_aux=[]   # Velocidades articulares
        self.joint_eff_aux=[]   # Esfuerzos articulares
        self.timestamps_aux=[]  # Vector para guardar las marcas de tiempo.


        self.timestamps=[]  # Vector para guardar las marcas de tiempo.
        self.timestamps_aux=[]  # Vector auxiliar para guardar las marcas de tiempo.

        self.n_muestras=self.get_parameter('n_muestras').value  # Número muestras por registrar.
        self.ruta_guardado=self.get_parameter('ruta_guardado').value    # Ruta de guardado del CSV.
        self.continuar_=self.get_parameter('continuar').value   # Indicación de continuar la ejecución o no.
        self.io= self.get_parameter('io').value # Para saber si queremos entradas/salidas.
        
        self.analog_output_pin= self.get_parameter('analog_output_pin').value  
        self.analog_input_pin= self.get_parameter('analog_input_pin').value  
        self.digital_output_pin= self.get_parameter('digital_output_pin').value  
        self.digital_input_pin= self.get_parameter('digital_input_pin').value 
        
        self.frecuencia_muestreo= self.get_parameter('freq').value
        self.timer_period= 1/self.frecuencia_muestreo

        # Se define el periodo y se invoca el temporizador.
        self.create_timer(self.timer_period, self.timmer_callback)

        self.arrancar_suscriptor()


        self.barra_progreso=tqdm(total=self.n_muestras, desc='Procesando')

    def arrancar_suscriptor(self):

        # Se arranca la lectura de joitns.
        self.suscription_joints= self.create_subscription(JointState,
                                '/joint_states', 
                                self.joints_reader, 
                                10)

        # Se estudia si se quieren entradas y salidas y se arranca el suscriptor.
        if self.io=='output':
            self.subscription_analog= self.create_subscription(IOStates,
                                                     '/io_and_status_controller/io_states',
                                                     self.analog_output_reader, 10)
            self.subscription_digital= self.create_subscription(IOStates, 
                                                     '/io_and_status_controller/io_states', 
                                                     self.digital_output_reader, 10)
        elif self.io=='input':
            self.subscription_analog= self.create_subscription(IOStates,
                                                     '/io_and_status_controller/io_states',
                                                     self.analog_input_reader, 10)
            self.subscription_digital= self.create_subscription(IOStates, 
                                                     '/io_and_status_controller/io_states', 
                                                     self.digital_input_reader, 10)
            
        self.subscription_analog
        self.subscription_digital
        self.suscription_joints

    def timmer_callback(self):
        # self.get_logger().info('Muestra super logger Nº: %d'%self.i)

        # Registro de pines analógicos.
        x_pinA_state= self.read_pinA_state_aux
        # print(self.read_pinA_state_aux)
        self.read_pinA_state.append(x_pinA_state)
        # print(x_pin_state)

        # Registro de estados del pin digital
        x_pinD_state= self.read_pinD_state_aux
        self.read_pinD_state.append(x_pinD_state)


        # Registro de posición articular.
        x_pos=self.joint_pos_aux
        self.joint_pos.append(x_pos)

        # Registro de velocidad articular.
        x_vel=self.joint_vel_aux
        self.joint_vel.append(x_vel)

        # Registro de esfuerzo articular.
        x_eff=self.joint_eff_aux
        self.joint_eff.append(x_eff)


        # Registro de marcas de tiempo.
        x_timestamp=self.timestamps_aux
        self.timestamps.append(x_timestamp)


        self.numero_muestra.append(self.i)


        if self.i==self.n_muestras:
            print('FIN DEL PROCESO. PRECIONAR ctrl+C para cerrar de forma manual.')
            self.mi_tabla=pd.DataFrame({'N_muestra':self.numero_muestra,
                                        'Marca_tiempo': self.timestamps,
                                        'q_pos': self.joint_pos,
                                    'q_vel':self.joint_vel,
                                    'q_eff':self.joint_eff,
                                    'AO_state:': self.read_pinA_state,
                                    'DO_state:': self.read_pinD_state,
                                    })
                       
            # Se eliminan las filas con datos basura.
            # self.mi_tabla=self.mi_tabla[~self.mi_tabla.astype(str).apply(lambda x: x.str.contains('\[\]').any(), axis=1)]

            # Se guarda el csv
            self.mi_tabla.to_csv(self.ruta_guardado)

            self.barra_progreso.close()

        self.i+=1

        # sE inicia la barra de progreso
        self.barra_progreso.update(1)
        time.sleep(1/self.frecuencia_muestreo)


    # Método de callback para tomar datos de las joints.
    def joints_reader(self, msg):

        # Formateo del mensaje para que quede bonito.
        self.coord_pos=re.findall(r'[-+]?\d*\.\d+|\d+', str(msg.position))
        self.coord_vel=re.findall(r'[-+]?\d*\.\d+|\d+', str(msg.velocity))
        self.coord_eff=re.findall(r'[-+]?\d*\.\d+|\d+', str(msg.effort))

        self.coord_pos=[float(coor) for coor in self.coord_pos]
        self.coord_vel=[float(coor) for coor in self.coord_vel]
        self.coord_eff=[float(coor) for coor in self.coord_eff]
             
        # Se guardan los valores muestreados en atributos auxiliares.
        self.joint_pos_aux=self.coord_pos
        self.joint_vel_aux=self.coord_vel
        self.joint_eff_aux=self.coord_eff

        # Se guarda la marca de tiempo de cada medida (sincornizada con el reloj del ordenador)
        self.timestamps_aux= time.time()
        # self.timestamps_aux= self.formato_timestamp(self.timestamps_aux) # Esta línea formatea las marcas de tiempo para HH:mm:ss:decimasSegundo. Si se desea que no ocurra comentar esta línea.

    # Método de callback para tomar datos de las salidas analógicas.
    def analog_output_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.analog_output_pin}, domain=0, state=([\d.]+)"
        resultado= re.search(patron, str(msg.analog_out_states))
        # print(resultado.group(1))
        if resultado:
            self.read_pinA_state_aux= resultado.group(1)
        else:
            print('No se encuentra el valor.')

        # Se guarda la marca de tiempo de cada medida (sincornizada con el reloj del ordenador)
        # self.timestamps_aux= time.time()
        # self.timestamps_aux= self.formato_timestamp(self.timestamps_aux) # Esta línea formatea las marcas de tiempo para HH:mm:ss:decimasSegundo. Si se desea que no ocurra comentar esta línea.

    # Método de callback para tomar datos de las entradas analógicas.
    def analog_input_reader(self, msg):
        
        # Para buscar pin y resultado.
        patron= rf"pin={self.analog_input_pin}, domain=0, state=([\d.]+)"
        resultado= re.search(patron, str(msg.analog_in_states))

        if resultado:
            self.read_pinA_state_aux= resultado.group(1)
        else:
            print('No se encuentra el valor.')

        
 # Método de callback para tomar datos de las salidas digitales.
    def digital_output_reader(self, msg):

        # Para buscar pin y resultado.
        patron= rf"pin={self.digital_output_pin}, state=(True|False)"
        resultado= re.search(patron, str(msg.digital_out_states))
        resultado=resultado.group(0)


        # Para guardar resultado del pin
        patron=r'(True|False)'
        self.read_pinD_state_aux=re.search(patron, str(resultado)).group() # Se invoca al método group para que se guarde sólo el valor solicitado.


    # # Método de callback para leer las entradas digitales.
    def digital_input_reader(self, msg):
        

        # Para buscar pin y resultado.
        patron= rf"pin={self.digital_input_pin}, state=(True|False)"
        resultado= re.search(patron, str(msg.digital_in_states))
        resultado=resultado.group(0)


        # Para guardar resultado del pin
        patron=r'(True|False)'
        self.read_pinD_state_aux=re.search(patron, str(resultado)).group() # Se invoca al método group para que se guarde sólo el valor solicitado.
        

    def formato_timestamp(self, timestamp_):

        # Se separa la marca de tiempo en la parte entera y la parte decimal.
        timestamp_int=int(timestamp_)
        timestamp_frac=timestamp_ - timestamp_int

        # Se convierte la la parte entera de la marca de tiempo en formato universal UNIX a formato local.
        time_struct=time.localtime(timestamp_int)

        # Se define el formato de tiempo en el que se desea mostrar la marca de tiempo.
        formato_tiempo=time.strftime('%H:%M:%S', time_struct) + ':' + f".{10*timestamp_frac:.5f}"[1:]

        return formato_tiempo



def main(args=None):
    rclpy.init(args=args)

    node=SuperLogger()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()