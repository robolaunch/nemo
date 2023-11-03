import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.current_distance_ = 0.0
        self.linear_velocity_ = 0.2
        self.target_distance_ = 3.0
        # self.linear_velocity_ = 1.0
        # self.target_distance_ = 6.28318530718*5

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity_
        # msg.angular.z = self.linear_velocity_
        self.publisher_.publish(msg)
        self.current_distance_ += self.linear_velocity_ * 0.1

        if self.current_distance_ >= self.target_distance_:
            msg.linear.x = 0.0
            # msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Robot hareketi tamamlandı.')
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
