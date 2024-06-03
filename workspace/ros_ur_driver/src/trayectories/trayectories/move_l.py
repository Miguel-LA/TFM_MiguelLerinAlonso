# # DESCRIPCIÓN

# Paquete de ros2 para la ejecucución de trayectorias. Este nodo es capaz de leer las coordenadas cartesianas de un csv almacenado en la carpeta ./data y calcular
# los movimientos necesarios para poder efectuarla. En esta versión, el robot se mueve calculando todas las poses en un sistema de coordanades lineales (cartesianas xyz). 
# Antes de iniciar la trayectoria, se recomienda poner al robot en una pose amigable para facilitar el trabajo del algoritmo del cálculo
# y evitar que pueda adoptar poses que puedan llevar a singularidades o poner en riesgo la integridad del equipo.

# Los parámetros de entrada al nodo son:

# * trayectoria_dato: Es el nombre del fichero CSV que contiene los puntos de la trayectoria a trazar. Debe estar obligatoriamente dentro de la carpeta ./data/
# * factor_escala: Es el factor de escala que permite definir la velocidad de ejecución del movimiento. Se deja al controlador de Moveit que la tarea
# de obtener las configuraciones cinemáticas necesarias. Este factor de escala lo que hace es aplicar un corrector para aumentar o disminuir la velocidad base de moveit.
# Se recomienda ensayar en el rango de 0.01-2.00 veces el valor de velocidad base.


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
# Dependiendo del número de puntos, complejidad de la trayectoria y pose inicial del robot puede demeorarse más o menos tiempo.
# Se recomienda realizar unos ensayos previos a un bajo factor de escala para conocer la velocidad de ejecución en cada caso.
class CartesianPathNode(Node):
    # Constructor de la clase de cálculo de trayectoria.
    def __init__(self):
        super().__init__('cartesian_path_node')
        # Se crea un cliente que calculará la trayectoria a partir de los datos cartesianos. El cliente es nativo de moveit y ros2.
        self.compute_cartesian_path_client= self.create_client(GetCartesianPath, '/compute_cartesian_path')

    # Método de cálculo de trayectoria
    def compute_cartesian_path(self, waypoints, factor_escala):
        # Se crea una solicitud para que el servicio calcule la trayectoria
        # Se asignan marcas de tiempo, ancho de paso, puntos de cálculo y que se limiten las colisiones con el entorno virtual definido por moveit.
        request= GetCartesianPath.Request()
        request.header.stamp= self.get_clock().now().to_msg()
        request.group_name= 'ur_manipulator'
        request.waypoints= waypoints
        request.max_step= 0.05
        request.avoid_collisions= True

        print(f'Se mete como dato {len(waypoints)} waypoints a calcular')

        # Aviso de que el servicio puede tardar un poco en obtener un resultado final.
        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperado ...')
        

        # Solución de trayectoria calculada
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        print('Servicio completado')
        print(f'Future tiene un total de {len(future.result().solution.joint_trajectory.points)}')

        # Con una trayectoria previa calculada por el controlador, se define el control de velocidad mediante factor de escala.
        if future.result() is not None:
            
            # Copia de la trayectoria de moveit con la que se operará
            destino= future.result().solution
            # Modificación de velocidades y aceleraciones articulares de la trayectoria resultado.
            # ïdem para los tiempos relativo de ejecución. Se hace punto a punto para asegurar un buen resultado.
            for j in range(len(future.result().solution.joint_trajectory.points)):
                x_= future.result().solution.joint_trajectory.points[j]

                # Operación de velocidades y aceleraciones
                for i in range(len(x_.velocities)):
                    x_.velocities[i]*= factor_escala
                    x_.accelerations[i]*= factor_escala

                # print(x_.time_from_start)

                # Operación de tiempos de reloj
                aux_nanosec= x_.time_from_start.nanosec
                aux_sec= x_.time_from_start.sec
                aux_duration= aux_sec*1e9+aux_nanosec
                aux_duration/=factor_escala

                segundos= aux_duration//1e9
                nanosegundos= aux_duration%1e9
                # print(f'Tiempo con el factor de escala. Segundos {segundos} --- Nanosegundos {nanosegundos}')

                x_.time_from_start.sec= int(segundos)
                x_.time_from_start.nanosec= int(nanosegundos)



                # print(f"velocidades: {x_.velocities}")
                # print(f"aeleraciones: {x_.velocities}")

                # Asignación de los resultados modificados al resultado original. Se hace punto a punto.
                destino.joint_trajectory.points[j]= x_

                if j==len(future.result().solution.joint_trajectory.points)-1:
                    print(f'Tiempo de ejecución aproximado: {segundos/60:.2f} minutos')
                    print(f'Se calculan {len(future.result().solution.joint_trajectory.points)} puntos en la solución')
            
            
            # Depenediendo del factor de escala aplicado se pasa a ROS2 una solución a otra.
            # Solución original de moveit si =1. Solución modificada si >1 o <1.
            if factor_escala==1:
                print('Se ejecuta la solucion de moveit')
                print(f"Factor escala: {factor_escala}")
                trajectory= future.result().solution
            elif factor_escala<1:
                print('Se ejecuta la solución del control de velocidad')
                print(f"Factor escala < 1 : {factor_escala}")
                trajectory= destino
            else:
                print('Se ejecuta la solución del control de velocidad')
                print(f"Factor escala > 1 : {factor_escala}")
                trajectory= destino
            
            # Se guarda la trayectoria calculada por moveit en un excel.
            self.save_trajectory(trajectory=trajectory)

            return trajectory
        else:
            self.get_logger().info('Se ha fallado calculando la solución en IK.')
            return None
        

    def save_trajectory(slef, trajectory):
        print('Se procede a guardar la trayectoria calculada en un csv')
        joint_names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points

        data = {name: [] for name in joint_names}
        data['time_from_start'] = []

        for point in points:
            for i, name in enumerate(joint_names):
                data[name].append(point.positions[i])
            time_in_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            data['time_from_start'].append(time_in_sec)


        # Se guarda la trayectoria resultado del cálculo de moveit en la carpeta data/saved_trajectories
        # Se formatea el título para que incluya la fecha y hora correspondientes. Son las de creación de la gráfica.
        fecha_hora_actual=datetime.datetime.now()

        # Se guarda la imagen indicando la fecha de actualizacion.
        fecha_str=fecha_hora_actual.strftime("%Y%m%d_%H%M")
        titulo_tabla=f"{fecha_str}_trajectory.csv"

        # Obtener la ruta al directorio actual
        current_dir = os.path.abspath(os.path.dirname(__file__))

        # Obtener la ruta al directorio del repositorio Git
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        
        ruta_guardado= git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/saved_trajectories/'


        
        ruta_guardado=os.path.join(ruta_guardado, titulo_tabla)

        # self.mi_tabla.to_csv(ruta_guardado)


        df = pd.DataFrame(data)
        df.to_csv(ruta_guardado)
        print(f'Trayectoria guardada en {ruta_guardado}')

    def get_data_path(self):

        # Obtener la ruta al directorio actual
        current_dir = os.path.abspath(os.path.dirname(__file__))

        # Obtener la ruta al directorio del repositorio Git
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        
        data_path= git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/'

        return data_path

# Clase responsable de comunicar las acciones de seguimiento y ejecución de trayectorias.
class MyActionClientNode(Node):
    
    # Método constructor
    def __init__(self):
        super().__init__('action_client_node') 

        self.execute_client= ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    # Método de ejecución de trayectorias calculadas por CartesianPathNode
    def execute_trajectory(self, trajectory_solution):
        # Se llama al servidor y se espera el tiempo necesario.
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('No está disponible el servidor de la acción /execute_trajectory')
            return
        
        # Se crea una orden de ejecución de trayectoria y se carga la solución de CartesianPathNode
        goal_msg=ExecuteTrajectory.Goal()
        goal_msg.trajectory=trajectory_solution

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')

        # Repetir proceso hasta que se complelte la trayectoria o se paralice por moveit
        rclpy.spin_until_future_complete(self, future)
        result=future.result()
   

# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrayectoryNodeL(Node):
    # Constructor
    def __init__(self):
        super().__init__('trayectory_node_cartesian')

        # Parámetros de arranque para el nodo.
        self.declare_parameter('trayectoria_dato', '20240424_interpJunta0_cartesianspace_quat.csv')
        self.declare_parameter('arrancar_logger', False)
        self.declare_parameter('factor_escala', 1.00)
        self.arranca_logger=self.get_parameter('arrancar_logger').value

        self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
        
        # Lectura de la trayectoria dato desde el csv.
        trayectoria=self.get_parameter('trayectoria_dato').value
        file_path= self.get_data_path() + trayectoria
        self.positions= self.read_positions_from_file(file_path)

        self.factor_escala=self.get_parameter('factor_escala').value

        # Se invocan las clases de cálculo de trayectoria y mecanismo acción-cliente.
        self.cartesian_path_node= CartesianPathNode()
        self.action_client_node= MyActionClientNode()

        print('Iniciando trayectoria ...\n')

        self.goal_names=[]
        self.calculo_trayectoria()

        # Se indica el archivo de trayectoria dato que se ha tomado.
        print(f"Estoy corriendo el archivo de trayectoria {trayectoria}")


    def calculo_trayectoria(self):

        # Asignación de posiciones dato desde el csv.
        for position in self.positions:
            poses=Pose()
            # Aquí abajo pongo orientatio porque es como estaba definido antes. Pero la documentación se refiere al campo orientation como quaternion
            poses.position.x, poses.position.y, poses.position.z, poses.orientation.x, poses.orientation.y, poses.orientation.z, poses.orientation.w = position
            self.goal_names.append(poses)

        # Se solicita una trayectoria articular solución.
        print(f'El tamaño del vector de entrada es {len(self.goal_names)}')
        trajectory_solution= self.cartesian_path_node.compute_cartesian_path(self.goal_names, self.factor_escala)

        # Cuando se tiene la trayectoria solución se manda ejecutar.
        if trajectory_solution:
            print('Se ha calculado la trayectoria con éxito, ejecutando ...')
            self.action_client_node.execute_trajectory(trajectory_solution)
        else:
            print('Fallo en el cálculo de trayectoria')

        
        print('\n--- FIN DE TRAYECTORIA ---\n')

        # Mensaje interno de arrancar el logger.
        # self.arranca_logger=True
        self.publish_arranca_logger()

    # Método de lectura automática del csv
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
    
    # Método para indicar que se arranque el logger.
    def publish_arranca_logger(self):
        msg=Bool()
        msg.data=True
        self.publisher_.publish(msg)
        print('Se publica las señal de arrancar logger')
    
    
        
# Función main del archivo.
def main(args=None):
    rclpy.init(args=args)
    node=TrayectoryNodeL()

    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()