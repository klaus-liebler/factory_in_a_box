import rclpy

from rclpy.node import Node

import serial

import json

import time



class RobotPositionCycler(Node):

    def __init__(self):

        super().__init__('position_cycler')



        try:

            # Inicializa la conexión serial

            self.ser = serial.Serial("/dev/ttyUSB0", 115200)

            self.get_logger().info("Conexión serial establecida en /dev/ttyUSB0")

        except serial.SerialException as e:

            self.get_logger().error(f"Error al conectar con el puerto serial: {e}")

            raise



        # Definición de posiciones

        self.positions = [
        # Posiciones comunes:
        {"T": 3, "P1": 2340, "P2": 0, "P3": 2707, "P4": 1617, "P5": 2047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 0: posicion inicial
        {"T": 3, "P1": 240, "P2": 0, "P3": 2457, "P4": 1217, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 1: posicion post giro

        # Posiciones rojo:
        {"T": 3, "P1": 2160, "P2": 870, "P3": 2087, "P4": 897, "P5": 2857, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 2: posicion previa agarrar rojo
        {"T": 3, "P1": 2160, "P2": 1210, "P3": 2087, "P4": 897, "P5": 2857, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 3: posicion agarrar rojo
        {"T": 3, "P1": 2160, "P2": 1210, "P3": 2087, "P4": 897, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 4: posicion rojo agarrado
        {"T": 3, "P1": 2160, "P2": 870, "P3": 2087, "P4": 897, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 5: posicion post agarrar rojo
        {"T": 3, "P1": 2340, "P2": 0, "P3": 2457, "P4": 1217, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 6: posicion previa giro rojo

        # Posiciones verde:
        {"T": 3, "P1": 2320, "P2": 870, "P3": 2227, "P4": 1007, "P5": 2827, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 7: posicion previa agarrar verde
        {"T": 3, "P1": 2320, "P2": 1000, "P3": 2227, "P4": 1007, "P5": 2827, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 8: posicion agarrar verde
        {"T": 3, "P1": 2320, "P2": 1000, "P3": 2227, "P4": 1007, "P5": 3017, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 9: posicion verde agarrado
        {"T": 3, "P1": 2320, "P2": 870, "P3": 2227, "P4": 1007, "P5": 3017, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 10: posicion post agarrar verde
        {"T": 3, "P1": 2340, "P2": 0, "P3": 2457, "P4": 1217, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 11: posicion previa giro verde

        # Posiciones azul:
        {"T": 3, "P1": 2480, "P2": 740, "P3": 2307, "P4": 1087, "P5": 2797, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 12: posicion previa agarrar azul
        {"T": 3, "P1": 2480, "P2": 920, "P3": 2307, "P4": 1087, "P5": 2797, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 13: posicion agarrar azul
        {"T": 3, "P1": 2480, "P2": 920, "P3": 2307, "P4": 1087, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 14: posicion azul agarrado
        {"T": 3, "P1": 2480, "P2": 740, "P3": 2307, "P4": 1087, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 15: posicion post agarrar azul
        {"T": 3, "P1": 2340, "P2": 0, "P3": 2457, "P4": 1217, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 16: posicion previa giro azul

        # Posiciones amarillo:
        {"T": 3, "P1": 2420, "P2": 840, "P3": 2347, "P4": 1047, "P5": 2867, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 17: posicion previa agarrar amarillo
        {"T": 3, "P1": 2420, "P2": 1060, "P3": 2347, "P4": 1047, "P5": 2867, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 18: posicion agarrar amarillo
        {"T": 3, "P1": 2420, "P2": 1060, "P3": 2347, "P4": 1047, "P5": 3017, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 19: posicion amarillo agarrado
        {"T": 3, "P1": 2420, "P2": 840, "P3": 2347, "P4": 1047, "P5": 3017, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 20: posicion post agarrar amarillo
        {"T": 3, "P1": 2340, "P2": 0, "P3": 2457, "P4": 1217, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  # 21: posicion previa giro amarillo
        
        # Posiciones bloque bajo:
        {"T": 3, "P1": 240, "P2": 120, "P3": 2757, "P4": 1527, "P5": 3047, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #22 Posicion dejar bloque con bloque en pinza
        {"T": 3, "P1": 240, "P2": 120, "P3": 2757, "P4": 1527, "P5": 2727, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #23 Posicion dejar sin bloque
        {"T": 3, "P1": 240, "P2": 0, "P3": 2557, "P4": 1167, "P5": 2727, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #24 Posicion retroceso

        # Posiciones bloque medio:
        {"T": 3, "P1": 240, "P2": 10, "P3": 2717, "P4": 1527, "P5": 3057, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #25 Posicion previa soltar bloque
        {"T": 3, "P1": 240, "P2": 10, "P3": 2737, "P4": 1527, "P5": 2587, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #26 Posicion post soltar bloque
        {"T": 3, "P1": 240, "P2": 0, "P3": 2527, "P4": 1097, "P5": 2587, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #27 Posicion retroceso

        # Posiciones bloque alto:
        {"T": 3, "P1": 240, "P2": 280, "P3": 2457, "P4": 1397, "P5": 3137, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #28 Posicion medio camino
        {"T": 3, "P1": 240, "P2": 160, "P3": 2597, "P4": 1487, "P5": 3137, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #29 Posicion pre soltar bloque
        {"T": 3, "P1": 240, "P2": 160, "P3": 2597, "P4": 1487, "P5": 2807, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #30 Posicion post soltar bloque
        {"T": 3, "P1": 240, "P2": 0, "P3": 2507, "P4": 1087, "P5": 2807, "S1": 0, "S2": 0, "S3": 0, "S4": 0, "S5": 0, "A1": 60, "A2": 60, "A3": 60, "A4": 60, "A5": 60},  #31 Posicion retroceso
        
        ]

        self.get_logger().info("Secuencia de posiciones del tobogán izquierda configurada.")



    def send_position(self, position):

        """Envía una posición al brazo robótico."""

        try:

            data = json.dumps(position)

            self.ser.write(data.encode())

            self.get_logger().info(f"Enviando posición: {data}")

        except serial.SerialException as e:

            self.get_logger().error(f"Error enviando datos al robot: {e}")



    def execute_sequence(self,color1,color2,color3):
        
        self.send_position(self.positions[0]) # move the robotic arm to the initial position
        """Ejecuta la secuencia de movimiento en bucle."""
        #FIRST BLOCK (LOWER)
        if color1 == 'red':
                self.send_position(self.positions[2])
                time.sleep(2)
                self.send_position(self.positions[3])
                time.sleep(2)
                self.send_position(self.positions[4])
                time.sleep(2)
                self.send_position(self.positions[5])
                time.sleep(2)
                self.send_position(self.positions[6])
                time.sleep(2)
        elif color1 == 'green':
                self.send_position(self.positions[7])
                time.sleep(2)
                self.send_position(self.positions[8])
                time.sleep(2)
                self.send_position(self.positions[9])
                time.sleep(2)
                self.send_position(self.positions[10])
                time.sleep(2)
                self.send_position(self.positions[11])
                time.sleep(2)
        elif color1 == 'blue':
                self.send_position(self.positions[12])
                time.sleep(2)
                self.send_position(self.positions[13])
                time.sleep(2)
                self.send_position(self.positions[14])
                time.sleep(2)
                self.send_position(self.positions[15])
                time.sleep(2)
                self.send_position(self.positions[16])
                time.sleep(2)
        else:
                self.get_logger().warn(f"No hay acción definida para el color: {color1}")
                
        self.send_position(self.positions[1])
        time.sleep(2)
        self.send_position(self.positions[22])
        time.sleep(2)
        self.send_position(self.positions[23])
        time.sleep(2)
        self.send_position(self.positions[24])
        time.sleep(2)
        self.send_position(self.positions[0])
        time.sleep(2)
        
        #SECOND BLOCK (MID)
        if color2 == 'red':
                self.send_position(self.positions[2])
                time.sleep(2)
                self.send_position(self.positions[3])
                time.sleep(2)
                self.send_position(self.positions[4])
                time.sleep(2)
                self.send_position(self.positions[5])
                time.sleep(2)
                self.send_position(self.positions[6])
                time.sleep(2)
        elif color2 == 'green':
                self.send_position(self.positions[7])
                time.sleep(2)
                self.send_position(self.positions[8])
                time.sleep(2)
                self.send_position(self.positions[9])
                time.sleep(2)
                self.send_position(self.positions[10])
                time.sleep(2)
                self.send_position(self.positions[11])
                time.sleep(2)
        elif color2 == 'blue':
                self.send_position(self.positions[12])
                time.sleep(2)
                self.send_position(self.positions[13])
                time.sleep(2)
                self.send_position(self.positions[14])
                time.sleep(2)
                self.send_position(self.positions[15])
                time.sleep(2)
                self.send_position(self.positions[16])
                time.sleep(2)
        else:
                self.get_logger().warn(f"No hay acción definida para el color: {color1}")
                
        self.send_position(self.positions[1])
        time.sleep(2)
        self.send_position(self.positions[25])
        time.sleep(2)
        self.send_position(self.positions[26])
        time.sleep(2)
        self.send_position(self.positions[27])
        time.sleep(2)
        self.send_position(self.positions[0])
        time.sleep(2)
        
        if color3 == 'red':
                self.send_position(self.positions[2])
                time.sleep(2)
                self.send_position(self.positions[3])
                time.sleep(2)
                self.send_position(self.positions[4])
                time.sleep(2)
                self.send_position(self.positions[5])
                time.sleep(2)
                self.send_position(self.positions[6])
                time.sleep(2)
        elif color3 == 'green':
                self.send_position(self.positions[7])
                time.sleep(2)
                self.send_position(self.positions[8])
                time.sleep(2)
                self.send_position(self.positions[9])
                time.sleep(2)
                self.send_position(self.positions[10])
                time.sleep(2)
                self.send_position(self.positions[11])
                time.sleep(2)
        elif color3 == 'blue':
                self.send_position(self.positions[12])
                time.sleep(2)
                self.send_position(self.positions[13])
                time.sleep(2)
                self.send_position(self.positions[14])
                time.sleep(2)
                self.send_position(self.positions[15])
                time.sleep(2)
                self.send_position(self.positions[16])
                time.sleep(2)
        else:
                self.get_logger().warn(f"No hay acción definida para el color: {color1}")
                
        self.send_position(self.positions[1])
        time.sleep(2)
        self.send_position(self.positions[28])
        time.sleep(2)
        self.send_position(self.positions[29])
        time.sleep(2)
        self.send_position(self.positions[30])
        time.sleep(2)
        self.send_position(self.positions[31])
        time.sleep(2)
        self.send_position(self.positions[0])
        time.sleep(2)

        

        self.get_logger().info("Secuencia completa. Listo para repetir.")

        #while rclpy.ok():

            #for position in self.positions:

                #self.send_position(position)

                #time.sleep(2)

            #self.get_logger().info("Secuencia completa. Listo para repetir.")



def main(args=None):
    rclpy.init(args=args)

    # Crear nodo
    node = RobotPositionCycler()

    try:
        # Solicitar la entrada del usuario
        entrada = input("Select the tower to build: (red green blue) ")
    
        # Dividir la entrada en una lista de colores
        colores = entrada.split()
    
        # Asegurarse de que haya exactamente 3 colores
        if len(colores) != 3:
            print("Invalid input, the input must be 3 colours red green or blue separated by spaces")
            return
    
        # Asignar cada color a una variable diferente
        color1, color2, color3 = colores[0], colores[1], colores[2]
        node.execute_sequence(color1,color2,color3)  # Ejecutar la secuencia con los colores

    except KeyboardInterrupt:
        node.get_logger().info("Break.")

    finally:
        # Limpiar y cerrar el nodo
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()