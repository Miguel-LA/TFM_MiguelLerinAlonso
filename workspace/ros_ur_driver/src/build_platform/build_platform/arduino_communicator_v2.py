import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import syllapy

class ArduinoCommunicator(Node):
    def __init__(self):
        super().__init__('arduino_communicator')
        
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto serie según tu configuración

        self.timer = self.create_timer(0.1, self.read_serial)
        self.enviar_consigna=0
        self.T_consigna= 65


    def send_message_to_arduino(self, message):
        self.serial_port.write(message.encode())
        time.sleep(0.1)  # Espera para asegurar que el mensaje se envíe completamente

    def read_serial(self):
        if self.serial_port.in_waiting > 0:

            # if self.enviar_consigna>0:        
            #     self.serial_port.write(f"{self.T_consigna}".encode())
            #     time.sleep(0.1) 
            #     self.enviar_consigna=-1       # prueba para que la temperatura consigna se quede fija

            serial_data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = serial_data
            self.publisher_.publish(msg)
            print(msg.data)

            T_media= self.read_T_media(msg.data)
            t_consigna= self.read_T_consigna(msg.data)
            Mensaje= self.read_Mensaje(msg.data)


    def read_T_media(self, string_):

        string_T= 'Temperatura media -> '

        if string_.find(string_T)!=-1:
            indice_inicio_valor= string_.index(string_T) + len(string_T)
            T= float(string_[indice_inicio_valor:])
            return T
        
    def read_T_consigna(self, string_):

        string_T= 'Temperatura consigna -> '

        if string_.find(string_T)!=-1:
            indice_inicio_valor= string_.index(string_T) + len(string_T)
            T= float(string_[indice_inicio_valor:])
            return T
        
    def read_Mensaje(self, string_):

        string_T= 'Mensaje -> '

        if string_.find(string_T)!=-1:
            indice_inicio_valor= string_.index(string_T) + len(string_T)
            T= string_[indice_inicio_valor:]
            return T

    def send_message_to_arduino(self, message):
        self.serial_port.write(message.encode())
        time.sleep(0.1) 

def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoCommunicator()

    while rclpy.ok():
        # Envía el mensaje "hola" al Arduino cada segundo
        arduino_node.send_message_to_arduino(f"{arduino_node.T_consigna}\n")
        time.sleep(1)

    # arduino_node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()