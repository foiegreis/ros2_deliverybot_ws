#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <memory>
#include <string>
#include <vector>

class LidarFusionNode : public rclcpp::Node
{
public:
    LidarFusionNode() : Node("lidar_fusion_node")
    {
        // Create subscribers for each LiDAR
        std::vector<std::string> lidar_topics = {
            "/lidar_front_left/points",
            "/lidar_front_right/points",
            "/lidar_rear_left/points",
            "/lidar_rear_right/points"
        };

        for (const auto& topic : lidar_topics) {
            lidar_subs_.push_back(
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic, 10, 
                    std::bind(&LidarFusionNode::lidarCallback, this, std::placeholders::_1, topic)
                )
            );
        }

        // Create publisher for fused point cloud
        fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_pointcloud", 10);

        // Set up TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& topic)
    {
        latest_clouds_[topic] = msg;
        fusePointClouds();
    }

    void fusePointClouds()
    {
        if (latest_clouds_.size() < 4) return; // Wait until we have data from all LiDARs

        pcl::PointCloud<pcl::PointXYZ> combined_cloud;

        for (const auto& [topic, cloud_msg] : latest_clouds_) {
            try {
                // Transform point cloud to base_link frame
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform("base_link", cloud_msg->header.frame_id, tf2::TimePointZero);

                sensor_msgs::msg::PointCloud2 cloud_transformed;
                tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);

                // Convert to PCL point cloud
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                pcl::fromROSMsg(cloud_transformed, pcl_cloud);

                // Add to combined cloud
                combined_cloud += pcl_cloud;
            }
            catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            }
        }

        // Convert combined cloud back to ROS message
        sensor_msgs::msg::PointCloud2 fused_msg;
        pcl::toROSMsg(combined_cloud, fused_msg);
        fused_msg.header.frame_id = "base_link";
        fused_msg.header.stamp = this->now();

        // Publish fused point cloud
        fused_pub_->publish(fused_msg);
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFusionNode>());
    rclcpp::shutdown();
    return 0;
}