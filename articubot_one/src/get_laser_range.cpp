#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LaserScanSubscriber : public rclcpp::Node
{
public:
  LaserScanSubscriber() : Node("laser_scan_subscriber")
  {
    // Create a subscriber to receive laser scan messages
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_multi", 10,
        std::bind(&LaserScanSubscriber::laserScanCallback, this, std::placeholders::_1));
  }

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Retrieve laser scan data from the message
    std::vector<float> ranges = msg->ranges;
    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;

    // Specify the desired angle
    float desired_angle = 1.5;  // Example angle value in radians

    // Determine the index corresponding to the desired angle
    size_t index = static_cast<size_t>((desired_angle - angle_min) / angle_increment);

    // Retrieve the distance measurement at the specific angle
    float distance = ranges[index];
    printf("%.2f\n");
    // Use the distance measurement for further processing
    // ...
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}
