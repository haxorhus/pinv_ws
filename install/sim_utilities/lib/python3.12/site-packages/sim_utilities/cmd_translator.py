import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

class CmdVelNode(Node):
    def __init__(self):
        super().__init__('cmd_translator')
        
        # Subscriber para cmd_vel
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10)
        
        # Publishers para los thrusters
        self.new_cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        
        self.get_logger().info('Nodo convertidor cmd_vel iniciado')

    def cmd_vel_callback(self, msg):
        # Conversión de cmd_vel a thrusters (modelo diferencial)
        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z
        
        # nuevo cmd
        cmd_vel_new = Twist()
        cmd_vel_new.linear.x = linear_x
        cmd_vel_new.angular.z = angular_z
        
        # Publicar los valores calculados
        self.new_cmd_pub.publish(cmd_vel_new)
        

def main(args=None):
    rclpy.init(args=args)
    converter_node = CmdVelNode()
    try:
        rclpy.spin(converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()