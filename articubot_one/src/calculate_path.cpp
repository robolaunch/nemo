#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
bool direction = true;

class LaserToPointCloudNode : public rclcpp::Node {
public:
    LaserToPointCloudNode() : Node("laser_to_pointcloud") {
        RCLCPP_INFO(this->get_logger(), "NODE CREATED");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_filtered_front", 10, std::bind(&LaserToPointCloudNode::laserScanCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud_topic", 10);

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);

        cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();
    }

private:

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud = convertLaserScanToPointCloud(scan);
        publisher_->publish(*cloud);
        std::vector<double> com_vec = calculatePointCloudCenter(cloud);
        double magnitude = std::sqrt(com_vec[0]*com_vec[0] + com_vec[1]*com_vec[1]);
        double p_angular = 1.0;

        double angle_rad = std::atan2(com_vec[1], com_vec[0]);
        double angle_deg = angle_rad * 180.0 / M_PI; 

        vel_x = vel_magnitude*(com_vec[0]/magnitude);
        vel_y = vel_magnitude*(com_vec[1]/magnitude);
        vel_z = angle_rad * p_angular;

        if(direction == 1){
        vel_magnitude = 0.3;
        }
        else if(direction == 0){
        vel_magnitude = -0.3;
        //vel_z = -1 * vel_z;
        }
        else{
        vel_magnitude = 0.0;
        }

        printf("angle error (deg): %.2f\n", angle_deg);
        printf("angle error (rad): %.2f\n", angle_rad);

        printf("vel x: %.3f\n",  cmd_vel->linear.x);
        printf("vel y: %.3f\n",cmd_vel->linear.y);
        printf("vel yaw: %.3f\n\n",cmd_vel->angular.z);
        printf("Direction = %d\n",direction);

        cmd_vel->linear.x = vel_x;
        cmd_vel->linear.y = vel_y;
        cmd_vel->angular.z = vel_z;


        cmd_publisher_->publish(*cmd_vel);

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
        printf("x distance = %.3f\n\n",avg_x);

        if(avg_x<0.60){
            direction=false;
        }
        

        std::vector<double> result = {avg_x, avg_y};

        return result;

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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    geometry_msgs::msg::Twist::SharedPtr cmd_vel;

    pcl::PointXYZ com; // center of mass point
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;

    double vel_x;
    double vel_y;
    double vel_z;
    double vel_magnitude;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<LaserToPointCloudNode>());

    // Start the executor in a separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Wait for the executor thread to finish (optional)
    executor_thread.join();
    // rclcpp::spin(std::make_shared<LaserToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
