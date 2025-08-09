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

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # Configuración de QoS para que sea TRANSIENT_LOCAL
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Cargar el mapa de ocupación desde un archivo .pgm
        self.map_file = "/home/jose/pinv_ws/src/maps/map.pgm"
        self.map_yaml_file = "/home/jose/pinv_ws/src/maps/map.yaml"
        
        # Leer el archivo YAML para obtener metadata
        with open(self.map_yaml_file, 'r') as file:
            map_metadata = yaml.safe_load(file)
            self.resolution = map_metadata['resolution']
            self.origin = map_metadata['origin']
        
        # Crear un publicador de mapas
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)
                
        # Crear el broadcaster para la transformada estática
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Configurar la transformada estática entre `map` y `odom`
        self.publish_static_transform()
        
        # Timer para publicar el mapa a intervalos
        self.timer = self.create_timer(10.0, self.timer_callback)
        
        self.get_logger().info("Nodo MapPublisher iniciado. Publicando mapa...")

    def publish_static_transform(self):
        # Crear el mensaje de transformada
        transform_msg = TransformStamped()
        
        # Configurar los frames de referencia
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = "map"
        transform_msg.child_frame_id = "odom"
        
        # Posición y orientación de la transformada estática
        transform_msg.transform.translation.x = 0.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0
        
        # Publicar la transformada estática
        self.tf_broadcaster.sendTransform(transform_msg)
        self.get_logger().info("Transformada estática map=>odom publicada.")

    def timer_callback(self):
        # Crear el mensaje de mapa
        map_msg = OccupancyGrid()   
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        
        # Leer el mapa desde el archivo .pgm
        img = cv2.imread(self.map_file, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("Error al cargar el mapa. Verifica la ruta.")
            return
        
        # Procesar la imagen a un formato adecuado
        occupancy_values = np.where(img > 127, 0, 100).flatten()
        map_msg.data = occupancy_values.tolist()

        # Establecer dimensiones y resolución del mapa
        height, width = img.shape
        map_msg.info.resolution = self.resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = float(self.origin[0])
        map_msg.info.origin.position.y = float(self.origin[1])
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Publicar el mensaje
        self.publisher_.publish(map_msg)
        self.get_logger().debug("Mapa publicado.")

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    
    # Al cerrar el nodo
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()