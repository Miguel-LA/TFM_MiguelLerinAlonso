import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time 

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_v2')
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto serie según tu configuración
        self.timer = self.create_timer(0.1, self.read_serial)
        self.enviar_consigna=0
        self.T_consigna= 77

        # En el valor de T_consigna es preferible declarar lo que queires como un valor numérico y si hay un problema en el fondo va a entender un string
        if isinstance(self.T_consigna, str) and self.i==0:
            print(f"No voy a meter esa borriqueria de {self.T_consigna}")
            print('Le mando un OFF')
            self.T_consigna= 50
            self.enviar_consigna=1
        else:
            print(f'T_consigna es un valor numérico: {self.T_consigna}')
            self.enviar_consigna= 1

    def read_serial(self):
        if self.serial_port.in_waiting > 0:

            if self.enviar_consigna>0:        
                self.serial_port.write(f"{self.T_consigna}".encode())
                print(self.T_consigna)
                time.sleep(0.1) 
                self.enviar_consigna=-1       # prueba para que la temperatura consigna se quede fija

            serial_data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = serial_data
            self.publisher_.publish(msg)
            print(msg.data)

            if msg.data.find("Sumatorio = 240")!= -1:
                print('El find funciona')
                self.serial_port.write("PARAR \n".encode())

            T_media= self.read_T_media(msg.data)

    def read_T_media(self, string_):
        if string_.find("Media = ")!=-1:
            print('He encontrado la temperatura media')
            indice_inicio_valor= string_.index("Media = ") + len("Media = ")
            T_media= float(string_[indice_inicio_valor:])
            print(f'El valor numérico de la T_media es {T_media}')

            return T_media



def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
