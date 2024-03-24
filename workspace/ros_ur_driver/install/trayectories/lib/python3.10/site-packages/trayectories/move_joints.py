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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import time
import csv


# Clase que hace transformación de coordenadas cartesianas a articulares.
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.last_state = RobotState()

    def transform_xyz_to_joint_positions(self, goal_names):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.robot_state = self.last_state
        request.ik_request.pose_stamped = goal_names
        request.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_positions = future.result().solution.joint_state.position
            self.last_state = future.result().solution
            return joint_positions
        else:
            print("Failed to calculate IK solution")
            return None


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
        self.get_logger().info('Publicando: %s' % msg)


# Clase principal responsable de instanciar a las dos primeras y leer los datos del csv.
class TrajectoryNodeJ(Node):
    def __init__(self):
        super().__init__('trayectory_node_joints')

        self.declare_parameter('trayectoria_dato', 'points.csv')


        self.ik_node = IKTransformNode()
        self.joint_trajectory_publisher = publisher_joint_trajectory_controller()
        self.aux_node = rclpy.create_node("aux_node")

        trayectoria=self.get_parameter('trayectoria_dato').value
        file_path= '/home/alvaro/Desktop/workspace/ros_ur_driver/src/trayectories/data/' + trayectoria
        self.positions_generator = self.read_positions_from_file(file_path)
        # print(self.positions_generator)

        print("Iniciando trayectoria...\n")
        self.all_joint_positions = []

        self.calcular_trayectoria()

    def calcular_trayectoria(self):

        for position in self.positions_generator:

            goal_names = PoseStamped()
            goal_names.header.frame_id = "base_link"
            goal_names.header.stamp = self.aux_node.get_clock().now().to_msg()
            goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position

            goal_names.pose.orientation.w = 1.0

            joint_position = self.ik_node.transform_xyz_to_joint_positions(goal_names)

            if len(joint_position) == 6:
                self.all_joint_positions.append(joint_position)

        self.joint_trajectory_publisher.publish_trajectory(self.all_joint_positions)

        print("\n ---  TRAYECTORIA EJECUTANDOSE CON ÉXITO   ---\n")
        self.joint_trajectory_publisher.destroy_node()

    def read_positions_from_file(self, file_path):
        positions=[]
        with open(file_path, 'r') as file:
            csv_reader=csv.reader(file)
            for row in csv_reader:
                positions.append([float(value) for value in row])

            return positions


def main(args=None):
    rclpy.init(args=args)

    node=TrajectoryNodeJ()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
