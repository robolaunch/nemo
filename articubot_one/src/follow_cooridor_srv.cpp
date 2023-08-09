#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <cmath>
#include <unistd.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


class FollowCooridor : public rclcpp::Node
{
public:
  FollowCooridor() : Node("follow_cooridor_srv")
  {
 
    cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();

    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    cb_group_1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_1_;




    subscription_front_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_filtered_front", 10, std::bind(&FollowCooridor::laserScanCallback_front, this, std::placeholders::_1),
            sub_options);

    subscription_rear_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_filtered_rear", 10, std::bind(&FollowCooridor::laserScanCallback_rear, this, std::placeholders::_1),
            sub_options);

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud_topic", 10);

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&FollowCooridor::follow_path, this),
      cb_group_3_);
    
  }

private:


  void laserScanCallback_front(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // sensor_msgs::msg::PointCloud2::SharedPtr cloud = convertLaserScanToPointCloud(scan);
        this->scan_front = scan;
    }

  void laserScanCallback_rear(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        this->scan_rear = scan;
    }

  void follow_path()  // CHANGE
    {

        // printf("follow_path: %d\n",this->run);

        if(this->run){ 

            geometry_msgs::msg::TransformStamped transformStamped = buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);

            // Use the transform as needed
            translation_x = transformStamped.transform.translation.x;
            translation_y = transformStamped.transform.translation.y;
            translation_z = transformStamped.transform.translation.z;

            if(this->step_after_enterance == 0){
                RCLCPP_INFO(this->get_logger(), "Translation: (%.2f, %.2f, %.2f)", translation_x, translation_y, translation_z); 
                cooridor_enterance_x = translation_x;
                cooridor_enterance_y = translation_y;
                start_time = this->now();

                if(translation_x == 0){
                    geometry_msgs::msg::TransformStamped transformStamped = buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);

                    // Use the transform as needed
                    translation_x = transformStamped.transform.translation.x;
                    translation_y = transformStamped.transform.translation.y;

                    cooridor_enterance_x = translation_x;
                    cooridor_enterance_y = translation_y;
                    start_time = this->now();
                }else{
                    this->step_after_enterance++;
                    
                }
            }

            // print distance from enterance
            double distance_from_enterance = std::sqrt(std::pow(translation_x - cooridor_enterance_x, 2) + std::pow(translation_y - cooridor_enterance_y, 2));
            // RCLCPP_INFO(this->get_logger(), "Distance from enterance: %.2f");

            // time passed from enterance
            double time_passed = (this->now() - start_time).seconds();
            RCLCPP_INFO(this->get_logger(), "Time passed: %.2f Distance from enterance: %.2f", time_passed, distance_from_enterance);

            if(time_passed > 10 && distance_from_enterance < 0.3){
                is_cooridor_exit = true;
                this->run = false;
                RCLCPP_INFO(this->get_logger(), "COORIDOR EXITED\n\n");
                return;
            }

            sensor_msgs::msg::PointCloud2::SharedPtr cloud = convertLaserScanToPointCloud(direction ? this->scan_front : this->scan_rear);
            pointcloud_publisher_->publish(*cloud);

            
            std::vector<double> com_vec = calculatePointCloudCenter(cloud);
            double magnitude = std::sqrt(com_vec[0]*com_vec[0] + com_vec[1]*com_vec[1]);
            double p_angular = 1.0;

            
            double angle_rad = std::atan2(com_vec[1], com_vec[0]);
            int angle_rag_sign = angle_rad > 0 ? 1 : -1;

            if(!direction){
                angle_rad = M_PI - (angle_rag_sign * std::atan2(com_vec[1], com_vec[0]));
            }

            // printf("com_vec = %f",com_vec[0]);

            if(magnitude==0){
                magnitude = 0.3;
            }

            vel_x = vel_magnitude*(com_vec[0]/magnitude);
            vel_y = vel_magnitude*(com_vec[1]/magnitude);
            vel_z = angle_rad * p_angular;

            if(!direction){
                angle_rad = M_PI - (angle_rag_sign * std::atan2(com_vec[1], com_vec[0]));
                vel_z = -1 * angle_rag_sign * angle_rad * p_angular;
            }


            
            if(direction == 1){
                vel_magnitude = 0.3;
            }
            else if(direction == 0){
                vel_magnitude = -0.3;            }


            double angle_deg = angle_rad * 180.0 / M_PI; 


            // printf("angle error (deg): %.2f\n", angle_deg);
            // printf("angle error (rad): %.2f\n", angle_rad);

            // printf("vel x: %.3f\n",  cmd_vel->linear.x);
            // printf("vel y: %.3f\n",cmd_vel->linear.y);
            // printf("vel yaw: %.3f\n\n",cmd_vel->angular.z);
            // printf("Direction = %d\n",direction);

            cmd_vel->linear.x = vel_magnitude;
            cmd_vel->linear.y = vel_y;
            cmd_vel->angular.z = vel_z;


            cmd_publisher_->publish(*cmd_vel);
        }
                                    
    }

    sensor_msgs::msg::PointCloud2::SharedPtr convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());

        pclCloud->header.frame_id = scan->header.frame_id;
        pclCloud->width = scan->ranges.size();
        pclCloud->height = 1;
        pclCloud->points.resize(pclCloud->width * pclCloud->height);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            float angle = scan->angle_min + i * scan->angle_increment;

            pclCloud->points[i].x = range * cos(angle);
            pclCloud->points[i].y = range * sin(angle);
            pclCloud->points[i].z = 0.0f;
        }

        pcl::toROSMsg(*pclCloud, *cloud);
        return cloud;
    }

    std::vector<double> calculatePointCloudCenter(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_msg) {
        // Get the number of points in the cloud
        size_t num_points = cloud_msg->width * cloud_msg->height;

        // Calculate the byte size of each point
        size_t point_step = cloud_msg->point_step;

        // Get the byte offsets of the XYZ fields
        size_t x_offset = 0, y_offset = 0, z_offset = 0;
        for (const auto& field : cloud_msg->fields) {
            if (field.name == "x")
                x_offset = field.offset;
            else if (field.name == "y")
                y_offset = field.offset;
            else if (field.name == "z")
                z_offset = field.offset;
        }

        // Get the pointer to the point cloud data
        const uint8_t* data = cloud_msg->data.data();

        // Initialize the centroid coordinates and count of valid points
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        size_t valid_points = 0;

        // Compute the sum of coordinates for valid points
        for (size_t i = 0; i < num_points; ++i) {
            float x = *reinterpret_cast<const float*>(data + x_offset);
            float y = *reinterpret_cast<const float*>(data + y_offset);
            // float z = *reinterpret_cast<const float*>(data + z_offset);

            if (std::isfinite(x) && std::isfinite(y)) {
                sum_x += x;
                sum_y += y;
                // sum_z += z;
                valid_points++;
            }

            // Increment the data pointer to the next point
            data += point_step;
        }

        // Calculate the average of coordinates for valid points
        double avg_x = sum_x / valid_points;
        double avg_y = sum_y / valid_points;
        // printf("x distance = %.3f\n\n",avg_x);

        if(avg_x<0.60){
            direction=false;
        }
        

        std::vector<double> result = {avg_x, avg_y};

        return result;

    }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_front_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_rear_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_front;
  sensor_msgs::msg::LaserScan::SharedPtr scan_rear;
  geometry_msgs::msg::Twist::SharedPtr   cmd_vel;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  
  rclcpp::SubscriptionOptions sub_options;
  rclcpp::CallbackGroup::SharedPtr cb_group_1_;
  rclcpp::CallbackGroup::SharedPtr cb_group_2_;
  rclcpp::CallbackGroup::SharedPtr cb_group_3_;
  rclcpp::Time start_time;

  double vel_x;
  double vel_y;
  double vel_z;
  double vel_magnitude;

  double cooridor_enterance_x;
  double cooridor_enterance_y;

  bool run;
  bool is_cooridor_exit{false};
  bool direction = true;

  int step_after_enterance;

  double translation_x;
  double translation_y;
  double translation_z;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowCooridor>();

  // Create the executor and add the node to it
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Start the executor in a separate thread
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // Wait for the executor thread to finish (optional)
  executor_thread.join();
//   rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
