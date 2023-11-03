#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class IMUTransformerNode : public rclcpp::Node
{
public:
  IMUTransformerNode() : Node("imu_transformer_node")
  {
    // Create the subscriber for IMU messages
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "camera/imu", 10, std::bind(&IMUTransformerNode::imuCallback, this, std::placeholders::_1));

    // Create the publisher for the transformed IMU messages
    transformed_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "transformed_imu_topic", 10);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Create a new message for the transformed IMU
    auto transformed_imu = std::make_shared<sensor_msgs::msg::Imu>();

    // Copy the header from the original message
    transformed_imu->header = msg->header;

    // Copy the orientation fields from the original message
    transformed_imu->orientation = msg->orientation;
    transformed_imu->orientation_covariance = msg->orientation_covariance;

    // Create a new message for the transformed linear acceleration
    auto transformed_linear_acc = std::make_shared<geometry_msgs::msg::Vector3>();

    // Transform the linear acceleration values
    transformed_linear_acc->x =  msg->linear_acceleration.z;
    transformed_linear_acc->y = -msg->linear_acceleration.x;
    transformed_linear_acc->z = -msg->linear_acceleration.y;

    // Assign the transformed linear acceleration to the new message
    transformed_imu->linear_acceleration = *transformed_linear_acc;
    transformed_imu->linear_acceleration_covariance = msg->linear_acceleration_covariance;

    // Publish the transformed IMU message
    transformed_imu_publisher_->publish(*transformed_imu);
  }

  void calculateRotationMatrix(double roll, double pitch, double yaw, double R[3][3]) {
    // Convert angles to radians
    double rollRad  = roll  * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;
    double yawRad   = yaw   * M_PI / 180.0;

    // Calculate trigonometric values
    double cosRoll  = std::cos(rollRad);
    double sinRoll  = std::sin(rollRad);
    double cosPitch = std::cos(pitchRad);
    double sinPitch = std::sin(pitchRad);
    double cosYaw   = std::cos(yawRad);
    double sinYaw   = std::sin(yawRad);

    // Calculate individual elements of the rotation matrix
    R[0][0] = cosYaw * cosPitch;
    R[0][1] = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    R[0][2] = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;

    R[1][0] = sinYaw * cosPitch;
    R[1][1] = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
    R[1][2] = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;

    R[2][0] = -sinPitch;
    R[2][1] = cosPitch * sinRoll;
    R[2][2] = cosPitch * cosRoll;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    transformed_imu_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUTransformerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
