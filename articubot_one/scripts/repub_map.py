import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
class TopicRenamer(Node):
    def __init__(self):
        super().__init__('topic_renamer')
        

        # Abone olunan orijinal konu
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/slamware_ros_sdk_server_node/map',  # Orijinal konu adı
            self.map_callback,
            10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/slamware_ros_sdk_server_node/scan',  # Orijinal konu adı
            self.scan_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/slamware_ros_sdk_server_node/odom',  # Orijinal konu adı
            self.odom_callback,
            10
        )

        # Yeni konuya yayın yap
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)


    def map_callback(self, msg):
        # Alınan mesajı yeni konuya yeniden yayınla
        self.map_publisher.publish(msg)

    def scan_callback(self, msg):
        # Alınan mesajı yeni konuya yeniden yayınla
        self.scan_publisher.publish(msg)
        
    def odom_callback(self, msg):
        # Alınan mesajı yeni konuya yeniden yayınla
        self.odom_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRenamer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()