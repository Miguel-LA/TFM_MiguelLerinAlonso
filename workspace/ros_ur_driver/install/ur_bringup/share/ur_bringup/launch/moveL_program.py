import rclpy
import math
import time
import csv
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')

    def compute_cartesian_path(self, waypoints):
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = "ur_manipulator"
        # request.start_state = ???
        request.waypoints = waypoints
        request.max_step = 0.5
        request.avoid_collisions = True

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')

        future = self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info("Esperando servicio...")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Servicio completado")

        if future.result() is not None:
            trajectory = future.result().solution
            print(future.result().fraction)
            return trajectory
        else:
            self.get_logger().info("Failed to calculate IK solution")
            return None


class MyActionClientNode(Node):
    def __init__(self):
        super().__init__('my_action_client_node')

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

    def execute_trajectory(self, trajectory_solution):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            print("El servidor de la acción '/execute_trajectory' no está disponible")
            return

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory_solution

        future = self.execute_client.send_goal_async(goal_msg)

        print("Esperando resultado de la ejecución...")
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        # print(result)

        # if result:
        #     if result.status == 1:
        #         print("Trayectoria ejecutada exitosamente!")
        #     else:
        #         print("Fallo al ejecutar la trayectoria")
        # else:
        #     print("Fallo al ejecutar la trayectoria")


def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    print(positions)
    print('Hola')
    return positions


def main(args=None):
    rclpy.init(args=args)
    cartesian_path_node = CartesianPathNode()
    action_client_node = MyActionClientNode()

    file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_hel.csv'
    positions = read_positions_from_file(file_path)

    print("Iniciando trayectoria...\n")

    goal_names = []

    for position in positions:
        # print("Punto leído desde el archivo CSV:", position)
        poses = Pose()
        poses.position.x, poses.position.y, poses.position.z = position
        poses.orientation.w = 1.0
        goal_names.append(poses)

    trajectory_solution = cartesian_path_node.compute_cartesian_path(
        goal_names)

    # print(trajectory_solution)

    if trajectory_solution:
        print("Trayectoria calculada exitosamente, ejecutando...")
        action_client_node.execute_trajectory(trajectory_solution)
    else:
        print("Fallo al calcular la trayectoria")

    print("\n ###   FINAL DE TRAYECTORIA   ###\n")
    # cartesian_path_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
