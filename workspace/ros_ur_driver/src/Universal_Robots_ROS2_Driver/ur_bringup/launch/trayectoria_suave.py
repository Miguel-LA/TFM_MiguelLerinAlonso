import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import csv


# ESTE NODO SE ENCARGA DE TRANSFORMAS LAS COORDENADAS DE XYZ A POSICIONES ARTICULARES
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def transform_xyz_to_joint_positions(self, goal_names):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.avoid_collisions=True
        request.ik_request.pose_stamped = goal_names
        future = self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_positions = future.result().solution.joint_state.position
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
            self.tiempo_nanosec = int(tiempo * 1e9)

    def publish_trajectory(self, coord_articulares):
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # Lista de nombres de las articulaciones
        point = JointTrajectoryPoint()
        point.positions = coord_articulares
        # Tiempo desde el inicio de la trayectoria
        point.time_from_start = Duration(
            sec=self.tiempo_sec, nanosec=self.tiempo_nanosec)

        msg.points.append(point)

        self.publisher_.publish(msg)
        # self.get_logger().info('Publicando: %s' % msg)

# ESTE NODO RECIBE LA POSICIÓN ARTICULAR DEL ROBOT
# Función asíncrona que envía una bandera para el cerrar el bucle de control del main


class JointStatesListener(Node):
    def __init__(self):
        super().__init__('joint_states_feedback')

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.callbackState, 10)

        self.joint_positions_target = []
        self.current_positions = []
        self.reached_target = False
        self.next_goal = False

    def callbackState(self, msg):

        if self.next_goal is True:
            # Ajustar precision
            tolerance = 0.001
            self.current_positions = msg.position

            # El orden de las articulaciones en /joint_states no es la misma que /joint_trajectory_controller/joint_trajectory
            # Para poder comparar cada articulación una a una, es necesario ordenar el vector de articulaciones
            last_element = self.current_positions[-1]

            for i in range(len(self.current_positions) - 1, 0, -1):
                self.current_positions[i] = self.current_positions[i - 1]

            self.current_positions[0] = last_element

            if all(abs(curr - target) < tolerance for curr, target in zip(self.current_positions, self.joint_positions_target)):
                self.next_goal = False
                self.reached_target = True


# FUNCION USADA PARA LEER LAS POSICONES XYZ DESDE UN ARCHIVO .csv
# De esta manera, se almacena en la variable positions TODOS los puntos
# del archivo. En caso de dar problemas, investigar la opción "yield"
# para almacenar los puntos uno a uno


def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKTransformNode()
    joint_trajectory_publisher = publisher_joint_trajectory_controller()
    joint_states = JointStatesListener()

    node = rclpy.create_node("trayectoria_ingenia")

    file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    print("Iniciando trayectoria...\n\n")

    for i, position in enumerate(positions):
        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"
        goal_names.header.stamp = node.get_clock().now().to_msg()
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position
        goal_names.pose.orientation.w = 1.0
        joint_position = ik_node.transform_xyz_to_joint_positions(goal_names)

        if joint_trajectory_publisher.cuenta != 0:
            joint_trajectory_publisher.AjustarTiempo(
                positions[joint_trajectory_publisher.cuenta - 1], positions[joint_trajectory_publisher.cuenta])

        joint_trajectory_publisher.publish_trajectory(joint_position)
        joint_states.joint_positions_target = joint_position
        joint_states.next_goal = True

        while not joint_states.reached_target:
            rclpy.spin_once(joint_states)

            # Comienza a calcular el siguiente punto cuando el robot está cerca del punto actual
            if not joint_states.reached_target and i < len(positions) - 1:
                next_position = positions[i + 1]
                distance_to_next = math.sqrt((next_position[0] - position[0]) ** 2 +
                                             (next_position[1] - position[1]) ** 2 +
                                             (next_position[2] - position[2]) ** 2)
                if distance_to_next < 0.01:  # Puedes ajustar este valor según lo cerca que desees que esté el robot del siguiente punto
                    next_goal_names = PoseStamped()
                    next_goal_names.header.frame_id = "base_link"
                    next_goal_names.header.stamp = node.get_clock().now().to_msg()
                    next_goal_names.pose.position.x, next_goal_names.pose.position.y, next_goal_names.pose.position.z = next_position
                    next_goal_names.pose.orientation.w = 1.0
                    next_joint_position = ik_node.transform_xyz_to_joint_positions(next_goal_names)
                    joint_trajectory_publisher.publish_trajectory(next_joint_position)
                    joint_states.joint_positions_target = next_joint_position
                    joint_states.next_goal = True
                    break

        porcentaje_completado = (
            joint_trajectory_publisher.cuenta / len(positions)) * 100
        num_caracteres_llenos = int(porcentaje_completado // 2)
        num_caracteres_vacios = 50 - num_caracteres_llenos
        barra_llena = '█' * num_caracteres_llenos
        barra_vacia = '░' * num_caracteres_vacios

        print(
            f"Progreso trayectoria: [{barra_llena}{barra_vacia}]\n")

        joint_states.reached_target = False
        joint_trajectory_publisher.cuenta += 1

    print("\n ###   FINAL DE TRAYECTORIA   ###\n")
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()