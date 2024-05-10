# # DESCRIPCIÓN

# Paquete de ros2 para la ejecucución de trayectorias. Este nodo es capaz de leer las coordenadas cartesianas de un csv almacenado en la carpeta ./data y calcular
# los movimientos necesarios para poder efectuarla. Enesta versión, el robot se mueve calculando todas las poses en un sistema de coordanades lineales (cartesianas xyz). 
# Antes de iniciar la trayectoria, se recomienda poner al robot en una pose amigable para facilitar el trabajo del algoritmo del cálculo
# y evitar que pueda adoptar poses que puedan llevar a singularidades o poner en riesgo la integridad del equipo.

# Los parámetros de entrada al nodo son:

# * trayectoria_dato: Es el nombre del fichero CSV que contiene los puntos de la trayectoria a trazar. Debe estar obligatoriamente dentro de la carpeta ./data/



# Librerías de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from std_msgs.msg import Bool
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
from moveit_msgs.srv import GetPositionIK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose # Para manejar datos de las poses adoptadas en coordenadas articulares.
from moveit_msgs.msg import Constraints # Control de restricciones impuestas por el urdf o el entorno de movit.
from moveit_msgs.srv import GetCartesianPath # Algoritmo de cálculo de trayectorias en un espacio vectorial cartesiano.
from rclpy.action import ActionClient # Cliente  para efecturar acciones de ros2.
from moveit_msgs.action import ExecuteTrajectory # Acción de ejecución de trayectorias adaptada de moveit.
from builtin_interfaces.msg import Duration


# Otras librerías útiles.
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re
import time
from tqdm import tqdm
import math 
import csv
import os
import datetime
import git

# Esta clase es la respondable de calcular la trayectoria cartesiana.
class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client= self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def compute_cartesian_path(self, waypoints, factor_escala):
        request= GetCartesianPath.Request()
        request.header.stamp= self.get_clock().now().to_msg()
        request.group_name= 'ur_manipulator'
        request.waypoints= waypoints
        request.max_step= 0.05
        request.avoid_collisions= True
        # request.max_cartesian_speed= 0.5

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperado ...')
        
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        print('Servicio completado')

        if future.result() is not None:
            
            ###################################### PARTE NUEVA BORRAR/COMENTAR SI NECESARIO ##################
            destino= future.result().solution

            for j in range(len(future.result().solution.joint_trajectory.points)):
                x_= future.result().solution.joint_trajectory.points[j]

                for i in range(len(x_.velocities)):
                    x_.velocities[i]*= factor_escala
                    x_.accelerations[i]*= factor_escala

                print(x_.time_from_start)

                aux_nanosec= x_.time_from_start.nanosec
                aux_sec= x_.time_from_start.sec
                aux_duration= aux_sec*1e9+aux_nanosec
                aux_duration/=factor_escala

                segundos= aux_duration//1e9
                nanosegundos= aux_duration%1e9
                print(f'Tiempo con el factor de escala. Segundos {segundos} --- Nanosegundos {nanosegundos}')

                x_.time_from_start.sec= int(segundos)
                x_.time_from_start.nanosec= int(nanosegundos)



                # print(f"velocidades: {x_.velocities}")
                # print(f"aeleraciones: {x_.velocities}")

                destino.joint_trajectory.points[j]= x_

                # print(j)
            ##################################################################################################
            
            

            if factor_escala==1:
                print('Se ejecuta la solucion de moveit')
                print(f"Factor escala: {factor_escala}")
                trajectory= future.result().solution
            elif factor_escala<1:
                print('Se ejecuta la solución del control de velocidad')
                # print(destino)
                print(f"Factor escala < 1 : {factor_escala}")
                trajectory= destino
            else:
                print('Se ejecuta la solución del control de velocidad')
                # print(destino)
                print(f"Factor escala > 1 : {factor_escala}")
                trajectory= destino
            
            # print(future.result().fraction)
            return trajectory
            # return None
        else:
            self.get_logger().info('Se ha fallado calculando la solución en IK.')
            return None

# Clase responsable de comunicar las acciones de seguimiento y ejecución de trayectorias.
class MyActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node') 

        self.execute_client= ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def execute_trajectory(self, trajectory_solution):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('No está disponible el servidor de la acción /execute_trajectory')
            return
        
        goal_msg=ExecuteTrajectory.Goal()
        goal_msg.trajectory=trajectory_solution
        # print(goal_msg.trajectory[4])

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')


        rclpy.spin_until_future_complete(self, future)
        result=future.result()
   

# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrayectoryNodeL(Node):
    def __init__(self):
        super().__init__('trayectory_node_cartesian')

        self.declare_parameter('trayectoria_dato', '20240424_interpJunta0_cartesianspace_quat.csv')
        self.declare_parameter('arrancar_logger', False)
        self.declare_parameter('factor_escala', 1.00)
        self.arranca_logger=self.get_parameter('arrancar_logger').value

        self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
        

        self.cartesian_path_node= CartesianPathNode()
        self.action_client_node= MyActionClientNode()


        trayectoria=self.get_parameter('trayectoria_dato').value
        file_path= self.get_data_path() + trayectoria
        self.positions= self.read_positions_from_file(file_path)

        self.factor_escala=self.get_parameter('factor_escala').value

        print('Iniciando trayectoria ...\n')

        self.goal_names=[]
        self.calculo_trayectoria()
        print(f"Estoy corriendo el archivo de trayectoria {trayectoria}")


    def calculo_trayectoria(self):

        for position in self.positions:
            # print('Estoy leyendo poses del csv')
            # print(position)
            poses=Pose()
            # Aquí abajo pongo orientatio porque es como estaba definido antes. Pero la documentación se refiere al campo orientation como quaternion
            poses.position.x, poses.position.y, poses.position.z, poses.orientation.x, poses.orientation.y, poses.orientation.z, poses.orientation.w = position
            # poses.orientation.w=1.0
            # print('Se asignan las poses en la nueva lectura del csv')
            self.goal_names.append(poses)

        trajectory_solution= self.cartesian_path_node.compute_cartesian_path(self.goal_names, self.factor_escala)

        if trajectory_solution:
            print('Se ha calculado la trayectoria con éxito, ejecutando ...')
            self.action_client_node.execute_trajectory(trajectory_solution)
        else:
            print('Fallo en el cálculo de trayectoria')

        
        print('\n--- FIN DE TRAYECTORIA ---\n')
        # self.arranca_logger=True
        self.publish_arranca_logger()


    def read_positions_from_file(self, file_path):
        positions=[]
        with open(file_path, 'r') as file:
            csv_reader=csv.reader(file)
            for row in csv_reader:
                positions.append([float(value) for value in row])

            return positions
        
    def get_data_path(self):

        # Obtener la ruta al directorio actual
        current_dir = os.path.abspath(os.path.dirname(__file__))

        # Obtener la ruta al directorio del repositorio Git
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        
        data_path= git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/'

        return data_path
    
    def publish_arranca_logger(self):
        msg=Bool()
        msg.data=True
        self.publisher_.publish(msg)
        print('Se publica las señal de arrancar logger')
    
    
        

def main(args=None):
    rclpy.init(args=args)
    print('Esto es el move_l_new')
    node=TrayectoryNodeL()

    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()