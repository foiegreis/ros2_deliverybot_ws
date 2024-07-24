#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

//Doesn't work propertly. We will use the public package pointcloud_to_laserscan

class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
    PointCloudToLaserScanNode()
        : Node("pointcloud_to_laserscan_node"), target_height_(0.0)
    {
        this->declare_parameter("target_height", -2.32719); //-2.11588 = 0.0, -2.06588 = 5 cm
        this->get_parameter("target_height", target_height_);

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "fused_pointcloud", 10, std::bind(&PointCloudToLaserScanNode::pointCloudCallback, this, std::placeholders::_1));
        
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("fused_scan", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud to LaserScan node started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        sensor_msgs::msg::LaserScan laserscan_msg;
        laserscan_msg.header = msg->header;
        laserscan_msg.angle_min = 0; // 0 degrees
        laserscan_msg.angle_max = 6.28;  // 360 degrees
        laserscan_msg.angle_increment = (laserscan_msg.angle_max - laserscan_msg.angle_min) / 100;
        laserscan_msg.time_increment = 0.0;
        laserscan_msg.scan_time = 0.1;
        laserscan_msg.range_min = 0.10;
        laserscan_msg.range_max = 20.0;
        laserscan_msg.ranges.resize(100, std::numeric_limits<float>::infinity());

        for (const auto& point : pcl_cloud.points)
        {   
            
            // Filter points close to the target height with a small tolerance
            if (std::abs(point.z - target_height_) < 0.08) 
            {
                float angle = std::atan2(point.y, point.x);
                if (angle >= laserscan_msg.angle_min && angle <= laserscan_msg.angle_max)
                {
                    int index = static_cast<int>((angle - laserscan_msg.angle_min) / laserscan_msg.angle_increment);
                    float range = std::sqrt(point.x * point.x + point.y * point.y);
                    if (range >= laserscan_msg.range_min && range <= laserscan_msg.range_max)
                    {
                        if (range < laserscan_msg.ranges[index])
                        {
                            laserscan_msg.ranges[index] = range;
                        }
                    }
                }
            }
        }

        laserscan_pub_->publish(laserscan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    double target_height_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserScanNode>());
    rclcpp::shutdown();
    return 0;
}
