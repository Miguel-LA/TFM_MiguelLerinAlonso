import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time
import csv

#ESTE NODO ENVIA UNA SOLICITUD AL SERVICIO /compute_ik  para calcular las posiciones articualres
# class MoveItExampleNode(Node):
        
#     def __init__(self):
#         super().__init__('moveit_example_node')
#         self.ik_client= self.create_client(GetPositionIK, '/compute_ik')
       
#     def send_goal(self, goal_names):
#         request = GetPositionIK.Request()
#         request.ik_request.group_name = "ur_manipulator"
#         request.ik_request.pose_stamped , goal_names
#         future = self.ik_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)

#         if future.result() is not None and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
#             self.get_logger().info('IK Solution found')
            
#         else:
#             self.get_logger().error('IK Solution failed')

#ESTE NODO PUBLICA LAS POSICIONES ARTICUALRES EN FORMA DE MENSAJES EN EL NODO  /joint_trajectory. 
#PARA ELLO USAMOS EL METODO publish_trajectory() Y LE PASAMOS LAS POSCICIONES COMO ARGUMENTO    
class publisher_joint_trajectory_controller(Node):
        
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)  #este es el tamaño del buffer. 
        #self.timer = self.create_timer(1, self.publish_trajectory())
        self.count = 0

    def publish_trajectory(self, posiciones_art):
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint","elbow_joint","wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] # Lista de nombres de las articulaciones
        point = JointTrajectoryPoint()

        # Aquí redefine las posiciones de las articulaciones redondeandolas a 2 decimales
        #posiciones_art_red = [round(componente,2) for componente in posiciones_art]
        point.positions = posiciones_art #posiciones_art_red
        point.time_from_start = Duration(sec=6, nanosec=0) # Tiempo desde el inicio de la trayectoria
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando: %s' % msg)
        self.count += 1

#FUNCION USADA PARA LEER LAS POSICONES XYZ DESDE UN ARCHIVO .csv 
def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions

#ESTE NODO SE ENCARGA DE TRANSFORMAS LAS COORDENADAS DE XYZ A POSICIONES ARTICULARES
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def transform_xyz_to_joint_positions(self, goal_names):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator" 
        request.ik_request.pose_stamped = goal_names
        future = self.ik_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            joint_positions= future.result().solution.joint_state.position
            return joint_positions
        else:
            print("Failed to calculate IK solution")
            return None

def main(args=None):
    rclpy.init(args=args)
    #moveit_node = MoveItExampleNode()
    ik_node = IKTransformNode()
    
    node = rclpy.create_node("state_publisher")
    
    file_path = '/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    for position in positions:
        print("Punto leído desde el archivo CSV:", position)
        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"  
        goal_names.header.stamp = node.get_clock().now().to_msg()  # Aquí configuramos el campo 'stamp'
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position
        goal_names.pose.orientation.w = 0.0


        joint_positions = [] 
        joint_positions = ik_node.transform_xyz_to_joint_positions(goal_names)
        
        if joint_positions is not None:
            print("Joint positions:", list(joint_positions)) 

        # Envía la posición articulada al nodo de MoveIt para planificar el movimiento
        #moveit_node.send_goal(joint_positions)
    
    
        joint_trajectory_publisher = publisher_joint_trajectory_controller()
        joint_trajectory_publisher.publish_trajectory(joint_positions)
        
            #ESTO ES PARA EVITAR QUE LEA EL SIGUIENTE PUNTO ANTES DE ALCANZARLO
            #PARA SER MENOS CUTRES, HAY QUE HACER QUE ESTE TIEMPO VARIE Y UNA VEZ ALCANCE EL PUNTO, LEA EL SIGUIENTE
            #TIP DE PEDRO. USAR /tf QUE DA INFORMACION SOBRE EL ESTADO DEL ROBOT CADA NANOSEGUNDO
        time.sleep(0.01)
        #loop_rate.sleep()

    rclpy.spin(joint_trajectory_publisher)
    
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#MENSAJES CORRECTOS. NO SE MUEVE.
    # REVISAR LA PLANIFICACION DEL ROBOT -->  SI LA PLANIFICACION DE LA TRAYECTORIA FALLA EL ROBOT NO RECIBE ORDENES DE MOVIMIENTO VALIDAS
    # NODO PLANIFICACION --> moveit_node

    # en el nodo /joint_state_broadcaster
    # hay un servicio /plan_kinematic_path