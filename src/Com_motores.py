#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class BluetoothPublisher(Node):
    def __init__(self):
        super().__init__('bluetooth_publisher')
        # Creamos un publisher para recibir los mensajes del tópico "/motor_speeds"
        self.subscription = self.create_subscription(
            String,
            '/motor_speeds',
            self.listener_callback,
            10
        )

        # Conexión serial para el Bluetooth
        self.message = ''
        self.bluetooth_connection = None
        self.connect_bluetooth()

    def connect_bluetooth(self):
        connected = False

        #conexión por bluetooth
        while not connected:
            try:
                print("...Connecting...")
                self.BT = serial.Serial('COM13', 115200) #revisa que los baudios sean correctos y que el COM sea el indicado
                print("Connected to mini WAM-V")
                connected = True
            except:
                print("Cannot connect to mini WAM-V") #entrará en loop hasta conectarse
                print("Trying again...")
                #exit()

        #loop principal
        while True:
            #message += '\n'  #innecesario, esta aca para que se imprima bien en el serial mntr. SACALO PARA TRABAJAR CON LA ESP
            #BT.write(message.encode('utf-8'))  #manda el mensaje, ya sea vel de motor o led
            print(self.message)
            self.BT.write(self.message.encode('utf-8'))

            
    def listener_callback(self, msg):
        self.message = msg

    def __del__(self):
        # Asegurarse de cerrar la conexión al final
        if self.bluetooth_connection and self.bluetooth_connection.is_open:
            self.bluetooth_connection.close()
            self.get_logger().info("Conexión Bluetooth cerrada.")


def main(args=None):
    rclpy.init(args=args)

    bluetooth_publisher_node = BluetoothPublisher()

    try:
        rclpy.spin(bluetooth_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        bluetooth_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
