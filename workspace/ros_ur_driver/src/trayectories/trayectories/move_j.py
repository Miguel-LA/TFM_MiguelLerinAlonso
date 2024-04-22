# # DESCRIPCIÓN

# Paquete de ros2 para la ejecución de trayectorias. Este nodo es capaz de leer las coordenadas cartesianas de un csv almacenado en la carpeta ./data y calcular
# los movimientos necesarios para poder efectuarla. En esta versión, el robot se mueve calculando todas las poses en un sistema de coordanades articulaes (thetha_1, theta_2, etc.). 
# Antes de iniciar la trayectoria, se recomienda poner al robot en una pose amigable para facilitar el trabajo del algoritmo del cálculo
# y evitar que pueda adoptar poses que puedan llevar a singularidades o poner en riesgo la integridad del equipo.

# Los parámetros de entrada al nodo son:

# * trayectoria_dato: Es el nombre del fichero CSV que contiene los puntos de la trayectoria a trazar. Debe estar obligatoriamente dentro de la carpeta ./data/



import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import time
import csv
import os
import datetime
import git
# import Event


# # Clase que hace transformación de coordenadas cartesianas a articulares.
# class IKTransformNode(Node):
#     def __init__(self):
#         super().__init__('ik_transform_node')
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.last_state = RobotState()

#         self.last_state_joints=None
#         self.suscriptor_joint_state= self.create_subscription(JointState,
#                                 '/joint_states', 
#                                 self.data_reader_callback, 
#                                 10)
        
#         self.suscriptor_joint_state

#         print('fakin spin 1')
#         rclpy.spin_once(self)
#         print('fakin spin')
#         # self.create_subscription(RobotStateRTMsg,
#         #                          '=',self.temperaturas_motor,
#         #                          10)

#     # Método de callback para tomar datos.
#     def data_reader_callback(self, msg):
        
#         self.last_state_joints= msg

    
       
#     def transform_xyz_to_joint_positions(self, goal_names):
#         print('Estoy en transform xyz')
#         request = GetPositionIK.Request()
#         request.ik_request.group_name = "ur_manipulator"
#         print('7777')

#         # print(self.last_state_joints)
#         # while self.last_state_joints==None:
#         #     print(self.last_state)
#         #     # Eve.wait(1)

#         request.ik_request.robot_state.joint_state= self.last_state_joints
#         request.ik_request.pose_stamped = goal_names
#         request.ik_request.avoid_collisions = True
#         # print(request)
#         future = self.ik_client.call_async(request)
#         print('Antes del spint')
#         rclpy.spin_once(self)
#         # rclpy.spin_until_future_complete(self, future=future, timeout_sec=10.0)
        
#         # sourceFile= open('/home/alvaro/Desktop/demo_request_spin.txt', 'w')
#         # print(future, file=sourceFile)
#         # sourceFile.close()

#         rclpy.spin_until_future_complete(self, future)
#         # print(future.result)
#         print('Despues del spin')

#         if future.result() is not None:
#             print('Me meto al future')
#             joint_positions = future.result().solution.joint_state.position
#             print('1')
#             self.last_state = future.result().solution
#             print('2')
#             return joint_positions
#         else:
#             print("Failed to calculate IK solution")
#             return None


# Clase que publica la trayectoria calculada en coordenadas articulares.
class publisher_joint_trajectory_controller(Node):

    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.cuenta = 0
        # Asegurarse que las variables de tiempo siempre sean enteras
        self.tiempo_sec = 6
        self.tiempo_nanosec = 0
        self.last_sec = 0
        self.last_nanosec = 0

    def AjustarTiempo(self, punto1, punto2):
        # Definir Velocidad de impresión
        velocidad = 0.08
        distancia = math.sqrt((punto2[0] - punto1[0]) ** 2 +
                              (punto2[1] - punto1[1]) ** 2 +
                              (punto2[2] - punto1[2]) ** 2)

        tiempo = distancia/velocidad

        # Para "puntos alejados", trabajamos en segundos, y aproximamos al entero superior
        if (tiempo >= 0.75):
            self.tiempo_sec = math.ceil(tiempo)
            self.tiempo_nanosec = 0
        # Para "puntos cercanos", trabajamos en nanosegundos, con un margen del 30%
        else:
            self.tiempo_sec = 0
            self.tiempo_nanosec = int(tiempo * 1.3 * 1e9)

    def publish_trajectory(self, all_coord_articulares):
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # Lista de nombres de las articulaciones
        

        for i in range(len(all_coord_articulares)):
            point = JointTrajectoryPoint()
            # point.velocities = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
            point.positions = all_coord_articulares[i]
            point.velocities= [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
            point.accelerations= [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

            if i != 0:
                self.last_sec += self.tiempo_sec
                self.last_nanosec += self.tiempo_nanosec

                if self.last_nanosec >= 1e9:
                    extra_seconds = self.last_nanosec // 1e9
                    self.last_sec += int(extra_seconds)
                    self.last_nanosec %= 1e9
                    self.last_nanosec = int(self.last_nanosec)

                self.AjustarTiempo(
                    all_coord_articulares[i], all_coord_articulares[i-1])

            point.time_from_start = Duration(
                sec=self.tiempo_sec + self.last_sec, nanosec=self.tiempo_nanosec + self.last_nanosec)
            msg.points.append(point)

            
        # print(msg)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publicando: %s' % msg)


# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrajectoryNodeJ(Node):
    def __init__(self):
        try:
            super().__init__('trayectory_node_joints')

            self.declare_parameter('trayectoria_dato', 'tray_hel_joints.csv')
            self.declare_parameter('arrancar_logger', False)
            self.arranca_logger=self.get_parameter('arrancar_logger').value

            self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
            self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)


            # self.ik_node = IKTransformNode()
            self.joint_trajectory_publisher = publisher_joint_trajectory_controller()
            self.aux_node = rclpy.create_node("aux_node")

            trayectoria=self.get_parameter('trayectoria_dato').value
            file_path= self.get_data_path() + trayectoria
            self.positions_generator = self.read_positions_from_file(file_path)
            
            # print(self.positions_generator)

            print(f"Estoy corriendo el archivo de trayectoria {trayectoria}")
            print("Iniciando trayectoria...\n")
            self.all_joint_positions = []

            self.calcular_trayectoria()
        except Exception as exception:
            print('Except del try catch')
            print(exception)

    def calcular_trayectoria(self):
        for position in self.positions_generator:

            # goal_names = PoseStamped()
            # goal_names.header.frame_id = "base_link"
            # goal_names.header.stamp = self.aux_node.get_clock().now().to_msg()
            # goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position

            # goal_names.pose.orientation.w = 1.0

            # print('Antes de la transformación')
            # joint_position = self.ik_node.transform_xyz_to_joint_positions(goal_names)
            # print('se ha hecho la transformación')
            joint_position=position

            if len(joint_position) == 6:
                # print(f"Esto es joint_position: {joint_position}")
                self.all_joint_positions.append(joint_position)

        # print(self.all_joint_positions)
        self.joint_trajectory_publisher.publish_trajectory(self.all_joint_positions)
        
        print("\n ---  TRAYECTORIA EJECUTANDOSE CON ÉXITO   ---\n")
        

        self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
        self.publish_arranca_logger()
        # self.joint_trajectory_publisher.destroy_node()

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

    node=TrajectoryNodeJ()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
