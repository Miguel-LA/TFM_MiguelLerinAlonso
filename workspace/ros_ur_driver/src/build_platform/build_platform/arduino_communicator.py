import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import syllapy

class ArduinoCommunicator(Node):
    def __init__(self):
        super().__init__('arduino_communicator')
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto serie según tu configuración

    def send_message_to_arduino(self, message):
        self.serial_port.write(message.encode())
        time.sleep(0.1)  # Espera para asegurar que el mensaje se envíe completamente

def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoCommunicator()
    temperatura_prueba='Esternocleidomastoideo'
    if len(temperatura_prueba)>3:
        print(f"Longitud palabra entrada: {len(temperatura_prueba)}")
        print(f"No voy a meter esa borriqueria de {temperatura_prueba}")
        print('Le mando un OFF')
        temperatura_prueba= 50


    while rclpy.ok():
        # Envía el mensaje "hola" al Arduino cada segundo
        arduino_node.send_message_to_arduino(f"{temperatura_prueba}\n")
        time.sleep(1)

    # arduino_node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
