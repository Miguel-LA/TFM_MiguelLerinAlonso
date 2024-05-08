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
        self.i=0

    def read_serial(self):
        if self.serial_port.in_waiting > 0:

            temperatura_prueba='Esternocleidomastoideo'
            if len(temperatura_prueba)>3 & self.i==0:
                print(f"Longitud palabra entrada: {len(temperatura_prueba)}")
                print(f"No voy a meter esa borriqueria de {temperatura_prueba}")
                print('Le mando un OFF')
                temperatura_prueba= 50
                self.i=1

            if self.i>0:        # Corregir error esternocleido 
                self.serial_port.write(f"{temperatura_prueba}".encode())
                print(temperatura_prueba)
                time.sleep(0.1) 


            serial_data = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = serial_data
            self.publisher_.publish(msg)
            print(msg.data)



            if msg.data.find("Estado = 1")!= -1:
                print('El find funciona')
                self.serial_port.write("PARAR \n".encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
