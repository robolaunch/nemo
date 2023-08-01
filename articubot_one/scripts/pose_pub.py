import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

class TFListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.01, self.get_tf_transform)
        self.robot_pose = PoseStamped()

        self.pose_publish = self.create_publisher(PoseStamped, "robot_position", 10)

    def get_tf_transform(self):
        try:
            # Wait for the transformation from "map" to "base_link"
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

            # Print the translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            print(f"Translation (x, y, z): {translation.x}, {translation.y}, {translation.z}")
            print(f"Rotation (x, y, z, w): {rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}")

            self.robot_pose.pose.position.x = translation.x
            self.robot_pose.pose.position.y = translation.y
            self.robot_pose.pose.position.z = translation.z

            self.robot_pose.pose.orientation.w = rotation.w
            self.robot_pose.pose.orientation.x = rotation.x
            self.robot_pose.pose.orientation.y = rotation.y
            self.robot_pose.pose.orientation.z = rotation.z
            
            self.pose_publish.publish(self.robot_pose)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f"Exception while looking up transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
