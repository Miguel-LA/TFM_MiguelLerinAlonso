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


# ESTE NODO SE ENCARGA DE TRANSFORMAS LAS COORDENADAS DE XYZ A POSICIONES ARTICULARES
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

# ESTE NODO PUBLICA LAS COORDENADAS ARTICULARES AL TOPIC INDICAD0


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

            # print("Tiempo_sec:", self.tiempo_sec)
            # print("Tiempo_nanosec", self.tiempo_nanosec)
            # print("last_sec", self.last_sec)
            # print("last nanosec:", self.last_nanosec)

            point.time_from_start = Duration(
                sec=self.tiempo_sec + self.last_sec, nanosec=self.tiempo_nanosec + self.last_nanosec)
            msg.points.append(point)

            # print("Voy a publicar")
        self.publisher_.publish(msg)
        # print("Terminé de publicar")
        self.get_logger().info('Publicando: %s' % msg)


# FUNCION USADA PARA LEER LAS POSICONES XYZ DESDE UN ARCHIVO .csv
# De esta manera, se almacena en la variable positions TODOS los puntos
# del archivo. En caso de dar problemas, investigar la opción "yield"
# para almacenar los puntos uno a uno


def read_positions_from_file(file_path):
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            yield [float(value) for value in row]


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKTransformNode()
    joint_trajectory_publisher = publisher_joint_trajectory_controller()

    node = rclpy.create_node("trayectoria_ingenia")

    file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'

    positions_generator = read_positions_from_file(file_path)
    print(positions_generator)

    print("Iniciando trayectoria...\n")
    all_joint_positions = []

    while True:

        try:
            position = next(positions_generator)
        except StopIteration:
            break

        print("Punto leído desde el archivo CSV:", position)
        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"
        goal_names.header.stamp = node.get_clock().now().to_msg()
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position

        goal_names.pose.orientation.w = 1.0

        joint_position = ik_node.transform_xyz_to_joint_positions(goal_names)
        print(joint_position)

        if len(joint_position) == 6:
            all_joint_positions.append(joint_position)

    joint_trajectory_publisher.publish_trajectory(all_joint_positions)

    # rclpy.spin(joint_trajectory_publisher)
    print("\n ###   TRAYECTORIA EJECUTANDOSE CON ÉXITO   ###\n")
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
