"""
Control manual del bote desde el teclado: teclas -> /cmd_vel.

    ros2 run pinv_teleop teleop_teclado
    ros2 run pinv_teleop teleop_teclado --ros-args -p paso_lineal:=0.2

Basado en teleop_twist_keyboard, con dos cambios para el catamarán:

1. Modo "acelerador": cada tecla sube o baja la velocidad pedida y el nodo
   la publica a 10 Hz aunque no se toque nada. teleop_twist_keyboard solo
   publica al apretar, y con el watchdog de 500 ms del ESP32 el bote se
   frena entre pulsaciones.
2. Sin reversa: la velocidad lineal nunca baja de 0 (el firmware igual la
   recorta, acá se ve en pantalla).

Seguridad: espacio frena todo. Al salir (q o Ctrl+C) publica cero. Si el
nodo muere, deja de publicar y el watchdog del ESP32 frena en <= 0,5 s.

No usar con un objetivo de Nav2 activo: los dos publicarían en /cmd_vel.

Necesita una terminal real (ros2 run), no funciona dentro de un launch.
"""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


AYUDA = """
Control manual del ASV PINV  (publica /cmd_vel a {rate:.0f} Hz)
---------------------------------------------------------------
   w / flecha arriba     acelerar        (+{pl:.2f} m/s)
   s / flecha abajo      desacelerar     (-{pl:.2f} m/s, mínimo 0)
   a / flecha izquierda  girar a babor   (+{pa:.2f} rad/s)
   d / flecha derecha    girar a estribor (-{pa:.2f} rad/s)
   c                     timón al centro (giro = 0)
   ESPACIO / x           FRENAR TODO
   q / Ctrl+C            salir (frena antes)
Límites: v en [0, {vmax:.2f}] m/s, giro en [-{wmax:.2f}, {wmax:.2f}] rad/s
"""

# Secuencias de escape de las flechas -> tecla equivalente.
FLECHAS = {'\x1b[A': 'w', '\x1b[B': 's', '\x1b[D': 'a', '\x1b[C': 'd'}


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


def separar_teclas(texto):
    """
    Parte lo leído en teclas: 'ww' + flecha -> ['w', 'w', flecha].

    Con autorepetición o tipeando rápido, una sola lectura trae varias teclas.
    """
    teclas = []
    i = 0
    while i < len(texto):
        if texto[i] == '\x1b' and texto[i:i + 3] in FLECHAS:
            teclas.append(texto[i:i + 3])
            i += 3
        else:
            teclas.append(texto[i])
            i += 1
    return teclas


def aplicar_tecla(tecla, v, w, paso_v, paso_w, v_max, w_max):
    """Devuelve (v, w, salir) después de apretar una tecla."""
    tecla = FLECHAS.get(tecla, tecla.lower())
    if tecla == 'w':
        v += paso_v
    elif tecla == 's':
        v -= paso_v
    elif tecla == 'a':
        w += paso_w
    elif tecla == 'd':
        w -= paso_w
    elif tecla == 'c':
        w = 0.0
    elif tecla in (' ', 'x'):
        v, w = 0.0, 0.0
    elif tecla == 'q':
        return 0.0, 0.0, True
    # Redondear evita que 0.1 + 0.1 + ... deje restos como 0.30000000000000004.
    v = round(limitar(v, 0.0, v_max), 3)
    w = round(limitar(w, -w_max, w_max), 3)
    return v, w, False


def pulsos_estimados(v, w, separacion, v_max):
    """Misma cuenta que motors.cpp del firmware: (v, w) -> us izq / der."""
    def a_us(vel):
        # int(x + 0.5) como el firmware (round() de Python redondea al par).
        return int(1500 + 200 * limitar(vel, 0.0, v_max) / v_max + 0.5)
    return (a_us(v - w * separacion / 2), a_us(v + w * separacion / 2))


class TeleopTeclado(Node):

    def __init__(self):
        super().__init__('teleop_teclado')
        self.paso_v = self.declare_parameter('paso_lineal', 0.1).value
        self.paso_w = self.declare_parameter('paso_angular', 0.1).value
        self.v_max = self.declare_parameter('v_max', 2.0).value
        self.w_max = self.declare_parameter('w_max', 1.0).value
        self.rate = self.declare_parameter('frecuencia', 10.0).value
        # Solo para mostrar los pulsos; tienen que coincidir con config.h.
        self.separacion = self.declare_parameter('separacion_motores', 0.75).value
        self.v_max_esc = self.declare_parameter('v_max_firmware', 2.0).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.v = 0.0
        self.w = 0.0

    def publicar(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def estado(self):
        izq, der = pulsos_estimados(self.v, self.w, self.separacion, self.v_max_esc)
        return (f'\rv = {self.v:5.2f} m/s   giro = {self.w:+5.2f} rad/s   '
                f'ESC estimado: izq {izq} us  der {der} us   ')

    def avisar_otros_publicadores(self):
        # Dar tiempo al descubrimiento antes de contar.
        time.sleep(1.0)
        otros = self.count_publishers(self.pub.topic_name) - 1
        if otros > 0:
            self.get_logger().warn(
                f'Hay {otros} publicador(es) más en {self.pub.topic_name} '
                '(¿Nav2 corriendo?). No mandar objetivos mientras se usa el teclado.')

    def correr(self):
        fd = sys.stdin.fileno()
        print(AYUDA.format(rate=self.rate, pl=self.paso_v, pa=self.paso_w,
                           vmax=self.v_max, wmax=self.w_max))
        self.avisar_otros_publicadores()

        config_original = termios.tcgetattr(fd)
        periodo = 1.0 / self.rate
        proximo = time.monotonic()
        try:
            # cbreak: lee tecla por tecla pero Ctrl+C sigue funcionando.
            tty.setcbreak(fd)
            sys.stdout.write(self.estado())
            sys.stdout.flush()
            while rclpy.ok():
                espera = max(0.0, proximo - time.monotonic())
                listo, _, _ = select.select([fd], [], [], espera)
                if listo:
                    salir = False
                    leido = os.read(fd, 64).decode(errors='ignore')
                    for tecla in separar_teclas(leido):
                        self.v, self.w, salir = aplicar_tecla(
                            tecla, self.v, self.w, self.paso_v, self.paso_w,
                            self.v_max, self.w_max)
                        if salir:
                            break
                    sys.stdout.write(self.estado())
                    sys.stdout.flush()
                    if salir:
                        break
                if time.monotonic() >= proximo:
                    self.publicar(self.v, self.w)
                    proximo += periodo
        finally:
            self.v = self.w = 0.0
            # Varias veces por si alguno se pierde (QoS reliable, pero por las dudas).
            for _ in range(3):
                self.publicar(0.0, 0.0)
                time.sleep(0.05)
            termios.tcsetattr(fd, termios.TCSADRAIN, config_original)
            print('\nFrenado (cmd_vel = 0). Chau.')


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_teclado necesita una terminal: usar "ros2 run", no un launch.')
        return
    # Sin el manejador de Ctrl+C de rclpy: ese apaga el contexto antes del
    # finally y el cmd_vel = 0 de salida no llegaría a publicarse.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    nodo = TeleopTeclado()
    try:
        nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
