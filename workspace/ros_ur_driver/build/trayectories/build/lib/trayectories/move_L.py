# Librerías esenciales de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
from moveit_msgs.srv import GetPositionIK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Otras librerías útiles.
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re
import time
from tqdm import tqdm


class moveJoints(Node):
    def __init__(self):
        super().__init__('move_l')
        self.get_logger().info('Hola soy el nodo de move_L')


# Nodo que transforma las coordenadas cartesianas a coordenadas articulares.
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client= self.create_client(GetPositionIK, '/compute_ik')

    def transform_cartesian_to_joint_positions(self, goal_names):
        request=GetPositionIK.Request()
        request.ik_request.group_name= 'ur_manipulator'
        request.ik_request.avoid_collisions=True
        request.ik_request.pose_stamped=goal_names
        future= self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_positions= future.result().solution.joint_state_position
            return joint_positions
        else:
            print('Se ha fallado calculando las posiciones IK.')
            return None
        
# Nodo que envía el vector de poses en coordenadas articulares al topic indicado.
class publisher_joint_trajectory_controller(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher= self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        


def main(args=None):
    rclpy.init(args=args)
    
    node=moveJoints()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()