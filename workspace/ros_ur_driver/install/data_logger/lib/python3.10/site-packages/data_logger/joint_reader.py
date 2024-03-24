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
import sys

from data_logger.data_reader import lectorNode  # Para hacer herencias (implementar en el futuro)
import time # Para implementar marcas de tiempo en el muestreo.
from ur_msgs.msg._robot_state_rt_msg import RobotStateRTMsg


class jointReader(Node):

    def __init__(self):
        self.name='joint_reader'
        
        super().__init__(self.name)

         # Se declaran los parámetros con su valor por defecto.
        self.declare_parameter('n_muestras', 50)
        self.declare_parameter('ruta_guardado', '~/Desktop/joints_data.csv')
        self.declare_parameter('continuar', True)
        self.declare_parameter('period', 1)   # En s


        # Atributos propios del nodo para trabajar. Se asocian los parámetros inicalizados.
        self.i=0    # Contador de muestras.
        self.joint_pos=[]   # Posiciones articulares
        self.joint_vel=[]   # Velocidades articulares
        self.joint_eff=[]   # Esfuerzos articulares
        self.numero_muestra=[]  # Número de muestra tomada desde que arranca el nodo.
        self.timer_period=1
        self.timestamps=[]  # Vector para guardar las marcas de tiempo.

        # Atributos auxiliares
        self.joint_pos_aux=[]   # Posiciones articulares
        self.joint_vel_aux=[]   # Velocidades articulares
        self.joint_eff_aux=[]   # Esfuerzos articulares
        self.timestamps_aux=[]  # Vector para guardar las marcas de tiempo.

        self.n_muestras=self.get_parameter('n_muestras').value  # Número muestras por registrar.
        self.ruta_guardado=self.get_parameter('ruta_guardado').value    # Ruta de guardado del CSV.
        self.continuar_=self.get_parameter('continuar').value   # Indicación de continuar la ejecución o no.
        self.timer_period=self.get_parameter('period').value

        # Se define el periodo
        self.create_timer(self.timer_period, self.timmer_callback)

        self.arrancar_suscriptor()
    
    def timmer_callback(self):
        self.get_logger().info('Muestra Nº: %d'%self.i)
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
            print('Ya he tomado la muestra %d y lo guardo en un CSV' % self.i)
            self.mi_tabla=pd.DataFrame({'N_muestra':self.numero_muestra,
                                        'Marca_tiempo': self.timestamps,
                                    'q_pos': self.joint_pos,
                                    'q_vel':self.joint_vel,
                                    'q_eff':self.joint_eff})
            self.mi_tabla.to_csv(self.ruta_guardado)

        self.i+=1
    

    def arrancar_suscriptor(self):

        # Se arranca el nodo como suscriptor.
        self.create_subscription(JointState,
                                '/joint_states', 
                                self.data_reader_callback, 
                                10)
        
       
        # self.create_subscription(RobotStateRTMsg,
        #                          '=',self.temperaturas_motor,
        #                          10)

    # Método de callback para tomar datos.
    def data_reader_callback(self, msg):

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

    def formato_timestamp(self, timestamp_):

        # Se separa la marca de tiempo en la parte entera y la parte decimal.
        timestamp_int=int(timestamp_)
        timestamp_frac=timestamp_ - timestamp_int

        # Se convierte la la parte eentera de la marca de tiempo en formato universal UNIX a formato local.
        time_struct=time.localtime(timestamp_int)

        # Se define el formato de tiempo en el que se desea mostrar la marca de tiempo.
        formato_tiempo=time.strftime('%H:%M:%S', time_struct) + ':' + f".{10*timestamp_frac:.5f}"[1:]

        return formato_tiempo
    
    def temperaturas_motor(self, msg):
        print()


def main(args=None):

    sys.path.append('data_logger')

    rclpy.init(args=args)

    node=jointReader()


    # if node.continuar_==False:
    #     print('La recopilación de datos ha finalizado. Se destruirá el nodo.')
    #     node.destroy_node()
    #     rclpy.shutdown()
    # else:
    #     print('La recopilación de datos ha finalizado pero se continúa con la escucha del nodo.')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()