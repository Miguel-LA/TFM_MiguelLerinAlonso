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
from moveit_msgs.srv import GetPositionIK, GetPositionFK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose # Para manejar datos de las poses adoptadas en coordenadas articulares.
from moveit_msgs.msg import Constraints, RobotState, RobotTrajectory # Control de restricciones impuestas por el urdf o el entorno de movit.
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


# Nodo responsable de calcular una trayectoria articular. Emplea el servicio computeFK
class JointPathNode_computeFK(Node):
    def __init__(self):
        super().__init__('joint_path_node')
        self.compute_fk_client= self.create_client(GetPositionFK, '/compute_fk')

        self.compute_fk_client
        
        self.cuenta = 0
        # Asegurarse que las variables de tiempo siempre sean enteras
        self.tiempo_sec = 6
        self.tiempo_nanosec = 0
        self.last_sec = 0
        self.last_nanosec = 0
        
    
    def compute_joint_path(self, joint_values):

        
        print('Estoy en compute FK joint trajectory')
        # trajectory= JointTrajectory()
        # trajectory.joint_names= ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        # "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] 

        carthesian_path= GetPositionFK.Response()

        trajectory= RobotTrajectory()
        trajectory.joint_trajectory.joint_names= ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] 
        trajectory.joint_trajectory.header.stamp=  self.get_clock().now().to_msg()

        last_position = None
        last_time = 0
        last_velocity = None

        for position in joint_values:

            while not self.compute_fk_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Servicio no disponible, esperado ...')

            # request= GetPositionFK.Request() # Me sale un error diciendo que no lo puedo instanciar.
            request = GetPositionFK.Request()
            request.header.stamp= self.get_clock().now().to_msg()
            request.fk_link_names = ["wrist_3_link"]  # Cambia a tu eslabón final deseado
            request.robot_state.joint_state.name = trajectory.joint_trajectory.joint_names
            request.robot_state.joint_state.position = position

            # request= RobotState()
            # request.joint_state.position=position
            
        
            future = self.compute_fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            carthesian_path.pose_stamped.append(response)


            if response is not None and len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose

                point = JointTrajectoryPoint()
                point.positions = position
           
                point.time_from_start = Duration(sec=last_time, nanosec=0)
        
                point.velocities = [0.0] * len(trajectory.joint_trajectory.joint_names)  # Define tus velocidades
                point.accelerations = [0.0] * len(trajectory.joint_trajectory.joint_names)  # Define tus aceleraciones
                if last_position is not None and last_time is not None:
                    duration = 1    #  20240423 Me quedo aquí. REvisar.
                    velocities = [(p2 - p1) / duration for p1, p2 in zip(last_position, position)]
                    point.velocities = velocities
                    if last_velocity is not None:
                        accelerations = [(v2 - v1) / duration for v1, v2 in zip(last_velocity, velocities)]
                        point.accelerations = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
                    point.accelerations = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
                    # point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

                trajectory.joint_trajectory.points.append(point)

                last_position = position
                last_time += 1
                # last_velocity = velocities

        print('SE ha generado una trayectoria con computefk')
        # print(carthesian_path)
        # print(carthesian_path)
        # Se devuelve una trayectoria definida como <class 'trajectory_msgs.msg._joint_trajectory.JointTrajectory'>
        # Necesito devolver una tryectoria definida como <class 'moveit_msgs.msg._robot_trajectory.RobotTrajectory'>
        return carthesian_path

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
            self.get_logger().info('Servicio no disponible, esperado ...')
        
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        print('Servicio completado')

        if future.result() is not None:
            trajectory= future.result().solution
            # print(future.result().fraction)
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
        # print(goal_msg.trajectory[4])

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')


        rclpy.spin_until_future_complete(self, future)
        result=future.result()
   

# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrayectoryNodeJ(Node):
    def __init__(self):
        super().__init__('trayectory_node_cartesian')

        self.declare_parameter('trayectoria_dato', 'tray_hel_joints.csv')
        self.declare_parameter('arrancar_logger', False)
        self.arranca_logger=self.get_parameter('arrancar_logger').value

        self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
        


        self.joint_path_node= JointPathNode_computeFK()
        self.cartesian_path_node= CartesianPathNode()
        self.action_client_node= MyActionClientNode()


        trayectoria=self.get_parameter('trayectoria_dato').value
        file_path= self.get_data_path() + trayectoria
        self.positions= self.read_positions_from_file(file_path)

        print('Iniciando trayectoria ...\n')

        
        self.goal_names=[]
        self.calculo_trayectoria()
        print(f"Estoy corriendo el archivo de trayectoria {trayectoria}")

        # self.joint_path_node.compute_joint_path(self.positions)


    def calculo_trayectoria(self):

        x= self.joint_path_node.compute_joint_path(self.positions)
        print(x.pose_stamped)
        
        # for position in self.positions:
        #     # print('Estoy leyendo poses del csv')
        #     # print(position)
        #     poses=Pose()
        #     # Aquí abajo pongo orientatio porque es como estaba definido antes. Pero la documentación se refiere al campo orientation como quaternion
        #     poses.position.x, poses.position.y, poses.position.z, poses.orientation.x, poses.orientation.y, poses.orientation.z, poses.orientation.w = position
        #     # poses.orientation.w=1.0
        #     # print('Se asignan las poses en la nueva lectura del csv')
        #     self.goal_names.append(poses)

        # trajectory_solution= self.cartesian_path_node.compute_cartesian_path(self.goal_names)

        # if trajectory_solution:
        #     print('Se ha calculado la trayectoria con éxito, ejecutando ...')
        #     self.action_client_node.execute_trajectory(trajectory_solution)
        # else:
        #     print('Fallo en el cálculo de trayectoria')

        
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
    node=TrayectoryNodeJ()

    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()