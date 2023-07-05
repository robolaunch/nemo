#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class RealsenseIMUConverterNode : public rclcpp::Node
{
public:
  RealsenseIMUConverterNode() : Node("imu_converter_node")
  {
    // Create the subscriber
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "camera/imu", 10, std::bind(&RealsenseIMUConverterNode::imuCallback, this, std::placeholders::_1));

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "converted_imu", 10);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // converted_imu->header.stamp = rclcpp::Clock().now();

    converted_imu->linear_acceleration.x = msg->linear_acceleration.z;
    // converted_imu->linear_acceleration.y = msg->linear_acceleration.x * -1;
    // converted_imu->linear_acceleration.z = msg->linear_acceleration.y * -1;

    // imu_publisher_->publish(*converted_imu);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  sensor_msgs::msg::Imu::SharedPtr converted_imu;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseIMUConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
