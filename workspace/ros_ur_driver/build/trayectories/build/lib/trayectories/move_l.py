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

# Esta clase es la respondable de calcular la trayectoria cartesiana.
class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client= self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def compute_cartesian_path(self, waypoints):
        request= GetCartesianPath.Request()
        request.header.stamp= self.get_clock().now().to_msg()
        request.group_name= 'ur_manipulator'
        request.waypoints= waypoints
        request.max_step= 0.5
        request.avoid_collisions= True

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servico no disponible, esperado ...')
        
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        print('Servicio completado')

        if future.result() is not None:
            trajectory= future.result().solution
            print(future.result().fraction)
            return trajectory
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

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')

        rclpy.spin_until_future_complete(self, future)
        result=future.result()
   

# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrayectoryNodeL(Node):
    def __init__(self):
        super().__init__('trayectory_node_cartesian')

        self.declare_parameter('trayectoria_dato', 'points_hel.csv')
        

        self.cartesian_path_node= CartesianPathNode()
        self.action_client_node= MyActionClientNode()


        trayectoria=self.get_parameter('trayectoria_dato').value
        file_path= '/home/alvaro/Desktop/workspace/ros_ur_driver/src/trayectories/data/' + trayectoria
        self.positions= self.read_positions_from_file(file_path)

        print('Iniciando trayectoria ...\n')

        self.goal_names=[]
        self.calculo_trayectoria()

    def calculo_trayectoria(self):

        for position in self.positions:
            poses=Pose()
            poses.position.x, poses.position.y, poses.position.z = position
            poses.orientation.w=1.0
            self.goal_names.append(poses)

        trajectory_solution= self.cartesian_path_node.compute_cartesian_path(self.goal_names)

        if trajectory_solution:
            print('Se ah calculado la trayectoria con éxito, ejecutando ...')
            self.action_client_node.execute_trajectory(trajectory_solution)
        else:
            print('Fallo en el cálculo de trayectoria')

        
        print('\n--- FIN DE TRAYECTORIA ---\n')

    def read_positions_from_file(self, file_path):
        positions=[]
        with open(file_path, 'r') as file:
            csv_reader=csv.reader(file)
            for row in csv_reader:
                positions.append([float(value) for value in row])

            return positions
        

def main(args=None):
    rclpy.init(args=args)

    node=TrayectoryNodeL()

    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()