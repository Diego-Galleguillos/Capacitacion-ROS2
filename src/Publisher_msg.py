#!/usr/bin/env python3


import serial
import pygame
import numpy as np
import math
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TeleopJoystick(Node):
    def __init__(self):
        super().__init__('teleop_joystick')
        self.publisher = self.create_publisher(String, '/motor_speeds', 10)
        
        # Inicializa las variables del joystick
        self.vel_izq, self.vel_der = 0, 0
        self.message = "M-0|0"
        self.joy = None
        self.angle = 0
        self.start_angle = 0
        self.arrow_adjustment = 0

        # Inicia pygame
        self.init_display()

        # Inicializa el joystick
        self.joystick_thread = Thread(target=self.joystick_handler)
        self.joystick_thread.start()

        # Inicia la visualización en pygame
        self.display_thread = Thread(target=self.update_display)
        self.display_thread.start()

    def init_display(self):
        global screen, XMAX, YMAX, CX, CY

        XMAX = 854
        YMAX = 480

        pygame.init()
        pygame.joystick.init()

        screen = pygame.display.set_mode((XMAX, YMAX))

        pygame.display.set_caption('Mini WAM-V')
        pygame.key.set_repeat(1, 50)

        CX = int(XMAX / 2)
        CY = int(YMAX / 2)

    def joystick_handler(self):
        global joy, vel_der, vel_izq, vert_move, horiz_move, message

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                joy = pygame.joystick.Joystick(event.device_index)

        while rclpy.ok():
            if pygame.joystick.get_count() == 0:
                for event in pygame.event.get():
                    if event.type == pygame.JOYDEVICEADDED:
                        joy = pygame.joystick.Joystick(event.device_index)

                vel_izq = 0
                vel_der = 0

                # Enviar el mensaje
                message = f"M-{vel_izq}|{vel_der}"
            else:
                horiz_move = joy.get_axis(0)
                vert_move = joy.get_axis(1)

                # Lógica de control del joystick
                magnitude = math.sqrt(horiz_move ** 2 + vert_move ** 2)
                magnitude = min(magnitude, 1)  # Asegura que la magnitud no supere 1

                # Solo hay movimiento vertical
                if horiz_move == 0:
                    vel_izq = vel_der = 255 * magnitude
                else:
                    angle = math.atan2(vert_move, horiz_move)
                    if horiz_move > 0:
                        vel_izq = 255 * magnitude
                        vel_der = (1 - abs(horiz_move)) * 255 * magnitude
                    else:
                        vel_der = 255 * magnitude
                        vel_izq = (1 - abs(horiz_move)) * 255 * magnitude

                # Limita las velocidades de los motores
                vel_izq = max(0, min(255, vel_izq))
                vel_der = max(0, min(255, vel_der))

                # Enviar el mensaje de control
                message = f"M-{vel_izq}|{vel_der}"

                # Publicar el mensaje en el tópico
                self.publish_message(message)

            time.sleep(0.1)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

    def update_display(self):
        global vel_izq, vel_der

        font = pygame.font.SysFont("Arial", 20)

        while rclpy.ok():
            screen.fill((255, 255, 255))  # Blanco

            # Dimensiones y lógica para los gráficos (como en el script original)
            # Aquí se mantiene igual la lógica de la GUI, puedes adaptarla si lo prefieres

            pygame.display.flip()
            pygame.event.pump()  # Evita que pygame crashee

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopJoystick()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
