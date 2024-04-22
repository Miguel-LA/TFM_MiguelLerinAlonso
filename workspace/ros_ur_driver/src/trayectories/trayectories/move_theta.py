#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs import FollowJointTrajectory
import csv
import time

class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.client = ActionClient(self, FollowJointTrajectory, 'execute_trajectory')
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def read_csv(self, filename):
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Ignorar la primera fila (encabezado)
            for row in csv_reader:
                joint_values = [float(val) for val in row]
                self.execute_trajectory(joint_values)
                time.sleep(1)  # Pausa entre cada punto de la trayectoria

    def execute_trajectory(self, joint_values):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start = rclpy.duration.Duration(seconds=1)  # Duración de 1 segundo para cada punto
        goal_msg.trajectory.points = [point]

        self.get_logger().info('Enviando meta de la trayectoria...')
        self.client.wait_for_server()

        self.client.send_goal_async(goal_msg)

        self.client.wait_for_result()

        result = self.client.get_result()

        return result

def main(args=None):
    rclpy.init(args=args)

    ur_node = URControlNode()

    ur_node.read_csv("path_to_your_csv_file.csv")

    rclpy.spin(ur_node)

    ur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
