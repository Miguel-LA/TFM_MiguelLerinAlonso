import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from subprocess import Popen

class NodeLauncher(Node):
    def __init__(self):
        super().__init__('node_launcher')

        self.logger_process = None
        self.mover_process = None

        # self.create_subscription(
        #     String,
        #     '/launch_nodes',
        #     self.launch_nodes_callback,
        #     10
        # )


        # print('Se lanza el suscriptor -----')
        self.start_nodes()

        # self.stop_nodes()

    # def launch_nodes_callback(self, msg):
    #     if msg.data == 'start':
    #         self.start_nodes()
    #         print('Se arrancan los nodos +++++++++')
    #     elif msg.data == 'stop':
    #         self.stop_nodes()

    def arranca_logger_callback(self, msg):
        print('Estoy en el callback de arrancar logger')
        print(msg.Data)
        if msg.Data==True:
            self.logger_process = Popen(['ros2', 'run', 'data_logger', 'super_logger', '--ros-args', '-p', 'n_muestras:=500'])

            

    def start_nodes(self):

        self.create_subscription(
            String,
            '/arranca_logger_topic',
            self.arranca_logger_callback,
            10
        )

        # self.arranca_logger_callback()
        self.mover_process = Popen(['ros2', 'run', 'trayectories', 'move_l'])

        # self.logger_process = Popen(['ros2', 'run', 'data_logger', 'super_logger', '--ros-args', '-p', 'n_muestras:=500'])
        self.logger_process= None
        # self.arranca_logger_callback()
        
    
        self.get_logger().info('Nodes started.')
    

    def stop_nodes(self):
        if self.logger_process:
            self.logger_process.terminate()
        if self.mover_process:
            self.mover_process.terminate()
        self.get_logger().info('Nodes stopped.')

def main(args=None):
    rclpy.init(args=args)
    node_launcher = NodeLauncher()
    rclpy.spin(node_launcher)
    node_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
