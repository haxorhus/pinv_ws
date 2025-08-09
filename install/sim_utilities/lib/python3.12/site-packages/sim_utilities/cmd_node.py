# thrust_converter.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToThrusters(Node):
    def __init__(self):
        super().__init__('thrust_converter')
        
        # Subscriber para cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publishers para los thrusters
        self.left_thrust_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/left/thrust', 
            10)
        
        self.right_thrust_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/right/thrust', 
            10)
        
        self.get_logger().info('Nodo convertidor cmd_vel a thrusters iniciado')

    def cmd_vel_callback(self, msg):
        # Conversión de cmd_vel a thrusters (modelo diferencial)
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Cálculo básico de thrust diferencial
        left_thrust = Float64()
        right_thrust = Float64()
        
        left_thrust.data = (linear_x - angular_z)*100
        right_thrust.data = (linear_x + angular_z)*100
        
        # Publicar los valores calculados
        self.left_thrust_pub.publish(left_thrust)
        self.right_thrust_pub.publish(right_thrust)
        
        self.get_logger().debug(f'Thrusters: L={left_thrust.data:.2f}, R={right_thrust.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    converter_node = CmdVelToThrusters()
    try:
        rclpy.spin(converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()