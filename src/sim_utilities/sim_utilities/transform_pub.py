import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import cv2
import numpy as np
import yaml

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_pub')

        # Configuración de QoS para que sea TRANSIENT_LOCAL
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # Crear el broadcaster para la transformada estática
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("Nodo StaticTransformPublisher iniciado.")

        # Configurar la transformada estática entre `map` y `odom`
        self.publish_static_transform()
                

    def publish_static_transform(self):
        # Crear el mensaje de transformada
        transform_msg = TransformStamped()
        
        # Configurar los frames de referencia
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "map"
        transform_msg.child_frame_id = "odom"
        
        # Posición y orientación de la transformada estática
        transform_msg.transform.translation.x = 20.0
        transform_msg.transform.translation.y = 20.0
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0
        
        # Publicar la transformada estática
        self.tf_broadcaster.sendTransform(transform_msg)
        self.get_logger().info("Transformada estática map=>odom publicada.")

def main(args=None):
    rclpy.init(args=args)
    Transform_Publisher = TransformPublisher()
    rclpy.spin(Transform_Publisher)
    
    # Al cerrar el nodo
    Transform_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()