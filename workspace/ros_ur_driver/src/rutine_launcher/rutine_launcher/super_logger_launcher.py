# # DESCRIPCIÓN
# Este nodo permite define y ejecuta una rutina consistente en el cálculo de una trayectoria y la ejecución del super_logger. 
# Lo primero que hace este nodo es calcular la trayectoria y quedarse escuchando a la recepción de un mensaje de continuación, 
# en ese momento manda correr el nodo del super_logger con las instrucciones deseadas.
#
# Para poder correr ambos nodos con las instrucciones deseadas se recomienda abrir el código y modificar las strings que contienen 
# el comando de terminal equivalente. 
#
# Este nodo no requiere de argumentos previos.


import rclpy
from rclpy.node import Node         # Para emplear nodos de ros2 con python
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from std_msgs.msg import Bool       # ïdem para datos tipo Bool
from subprocess import Popen        # Biblioteca de python que permite correr procesos en paralelo co un aterminal virtualizada


# El único nodo del fichero de nodo
class NodeLauncher(Node):
    # Constructor del nodo
    def __init__(self):
        super().__init__('super_logger_launcher')

        # Se definen las variables que alojarań cada proceso de forma previa por motivos de seguridad
        self.logger_process = None
        self.mover_process = None

        # Se definen las instrucciones que entenderá nuestro nodo por correr --> PONER DATOS DE RUTINA AQUÍ
        self.string_trayectoria= 'ros2 run trayectories move_l --ros-args -p factor_escala:=0.5 -p trayectoria_dato:=20240506_medio_estrella_posesROSquat_miguel.csv'
        self.string_super_logger= 'ros2 run data_logger super_logger --ros-args -p n_muestras:=500 -p freq:=100'

        # Se arrancan los nodos partícipes
        self.start_nodes()  

    # Arranque de los nodos partícipes
    def start_nodes(self):
        # Se arrnca el nodo de mover el robot con la trayectoria y luego se carga en la variable que representa dicho proceso.
        self.mover_process = Popen(self.string_trayectoria.split())

        # Se crea un nodo suscriptor a move_l que será responsable de actualizar la señal de arranque del nodo del logger gracias a su callback
        self.suscription_move_l= self.create_subscription(Bool,
                                '/arranca_logger_topic', 
                                self.arranca_logger_callback, 
                                10)

        self.suscription_move_l
        
    
        self.get_logger().info('Los nodos han arrancado')

    # Callback para arrancar el proceso del nodo del logger
    def arranca_logger_callback(self, msg):
        if msg.data==True:
            # Se arranca el nodo de super_logger en cuanto el nodo de la trayectoria indica que la va a comenzar.
            self.logger_process = Popen(self.string_super_logger.split())

            self.logger_process
             
    # Método qenérico de detención de nodos.
    def stop_nodes(self):
        if self.logger_process:
            self.logger_process.terminate()
        if self.mover_process:
            self.mover_process.terminate()
        self.get_logger().info('Los nodos han parado.')


# Función main del nodo
def main(args=None):
    rclpy.init(args=args)
    node_launcher = NodeLauncher()
    rclpy.spin(node_launcher)
    node_launcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
