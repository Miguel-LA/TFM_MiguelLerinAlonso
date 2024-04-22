# # DESCRIPCIÓN

# Paquete de ros2 para la ejecución de trayectorias. Este nodo es capaz de leer las coordenadas cartesianas de un csv almacenado en la carpeta ./data y calcular
# los movimientos necesarios para poder efectuarla. En esta versión, el robot se mueve calculando todas las poses en un sistema de coordanades articulaes (thetha_1, theta_2, etc.). 
# Antes de iniciar la trayectoria, se recomienda poner al robot en una pose amigable para facilitar el trabajo del algoritmo del cálculo
# y evitar que pueda adoptar poses que puedan llevar a singularidades o poner en riesgo la integridad del equipo.

# Los parámetros de entrada al nodo son:

# * trayectoria_dato: Es el nombre del fichero CSV que contiene los puntos de la trayectoria a trazar. Debe estar obligatoriamente dentro de la carpeta ./data/



import rclpy
import math
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState, RobotTrajectory
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient # Cliente  para efecturar acciones de ros2.
from moveit_msgs.action import ExecuteTrajectory # Acción de ejecución de trayectorias adaptada de moveit.
from control_msgs.action import FollowJointTrajectory

import time
import csv
import os
import datetime
import git
# import Event


# Nodo responsable de calcular una trayectoria articular. NO usa ningún tipo de servicio y lo hace a lo bruto.
class JointPathNode_bruto(Node):
    def __init__(self):
        super().__init__('joint_path_node')
#         self.compute_fk_client= self.create_client(GetPositionFK, '/compute_fk')

        self.cuenta = 0
        # Asegurarse que las variables de tiempo siempre sean enteras
        self.tiempo_sec = 6
        self.tiempo_nanosec = 0
        self.last_sec = 0
        self.last_nanosec = 0
        
    
    def compute_joint_path(self, joint_values):

        
        print('Estoy en compute joint trajectory')
        trajectory= JointTrajectory()
        trajectory.joint_names= ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] 

        for i in range(len(joint_values)):
            point= JointTrajectoryPoint()
            point.positions= joint_values[i]
            point.velocities= [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
            point.accelerations= [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
            # point.time_from_start = Duration(sec=1+self.cuenta, nanosec=0)
            # self.cuenta+= k1

            if i != 0:
                self.last_sec += self.tiempo_sec
                self.last_nanosec += self.tiempo_nanosec

                if self.last_nanosec >= 1e9:
                    extra_seconds = self.last_nanosec // 1e9
                    self.last_sec += int(extra_seconds)
                    self.last_nanosec %= 1e9
                    self.last_nanosec = int(self.last_nanosec)

                self.AjustarTiempo(
                    joint_values[i], joint_values[i-1])

            point.time_from_start = Duration(
                sec=self.tiempo_sec + self.last_sec, nanosec=self.tiempo_nanosec + self.last_nanosec)

            trajectory.points.append(point)

        return trajectory
        # print(trajectory.points)
        # print('Voy a entrar en execute_trajectory')

   

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
        trajectory= JointTrajectory()
        trajectory.joint_names= ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] 

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
            request.robot_state.joint_state.name = trajectory.joint_names
            request.robot_state.joint_state.position = position

            # request= RobotState()
            # request.joint_state.position=position
            
        
            future = self.compute_fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()


            if response is not None and len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose

                point = JointTrajectoryPoint()
                point.positions = position
           
                point.time_from_start = Duration(sec=last_time, nanosec=0)
        
                point.velocities = [0.0] * len(trajectory.joint_names)  # Define tus velocidades
                point.accelerations = [0.0] * len(trajectory.joint_names)  # Define tus aceleraciones
                if last_position is not None and last_time is not None:
                    duration = 1    #  20240423 Me quedo aquí. REvisar.
                    velocities = [(p2 - p1) / duration for p1, p2 in zip(last_position, position)]
                    point.velocities = velocities
                    if last_velocity is not None:
                        accelerations = [(v2 - v1) / duration for v1, v2 in zip(last_velocity, velocities)]
                        point.accelerations = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
                    point.accelerations = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
                    # point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

                trajectory.points.append(point)

                last_position = position
                last_time += 1
                # last_velocity = velocities

        print('SE ha generado una trayectoria con computefk')
        
        return trajectory
        # print(trajectory.points)
        # print('Voy a entrar en execute_trajectory')




# Clase responsable de comunicar las acciones de seguimiento y ejecución de trayectorias mediante un mecanismo de acción cliente
class MyActionClientNode(Node):

    def __init__(self):
        super().__init__('action_client_node')
        self.execute_client= ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # self.publisher_ = self.create_publisher(
        #     JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        

    def execute_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('No se pudo conectar al servidor.')
            return
        
        print('Estoy en execute trjectory')
        trajectory_=RobotTrajectory()
        trajectory_.joint_trajectory=trajectory

        goal_msg= ExecuteTrajectory.Goal()
        goal_msg.trajectory= trajectory_
        # print(goal_msg.trajectory)
        future= self.execute_client.send_goal_async(goal=goal_msg)
        print('Estoy despues del future')

        self.get_logger().info('Ejecutando trayectoria articular de move_j_action')
        rclpy.spin_until_future_complete(self, future)
        result=future.result()

    def publish_trajectory(self, all_coord_articulares):
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # Lista de nombres de las articulaciones

        for i in range(len(all_coord_articulares)):
            point = JointTrajectoryPoint()
            # point.velocities = [0.08, 0.08, 0.08, 0.08, 0.08, 0.08]
            point.positions = all_coord_articulares[i]

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
            self.action_client_node = MyActionClientNode()
            # self.joint_path_node= JointPathNode_bruto()
            self.joint_path_node= JointPathNode_computeFK()
            self.aux_node = rclpy.create_node("aux_node")

            trayectoria=self.get_parameter('trayectoria_dato').value
            file_path= self.get_data_path() + trayectoria
            self.positions_generator = self.read_positions_from_file(file_path)
            
            # print(self.positions_generator)

            print(f"Estoy corriendo el archivo de trayectoria {trayectoria}")
            print("Iniciando trayectoria...\n")
            self.all_joint_positions = []

            for position in self.positions_generator:

                joint_position=position

                if len(joint_position) == 6:
                    # print(f"Esto es joint_position: {joint_position}")
                    self.all_joint_positions.append(joint_position)

            # print(self.all_joint_positions)
            trajectory_solution= self.joint_path_node.compute_joint_path(self.all_joint_positions)
            # print(trajectory_solution)
            self.action_client_node.execute_trajectory(trajectory_solution)
            print('Voy a ejecutar el action client node')
            # self.action_client_node.execute_trajectory(self.all_joint_positions)

            print("\n ---  TRAYECTORIA EJECUTANDOSE CON ÉXITO   ---\n")
            

            self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
            self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
            self.publish_arranca_logger()
            # self.joint_trajectory_publisher.destroy_node()


        except Exception as exception:
            print('Except del try catch')
            print(exception)

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
