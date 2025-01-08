#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarFusionNode : public rclcpp::Node
{
public:
  LidarFusionNode()
  : Node("lidar_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
  {
    RCLCPP_INFO(this->get_logger(), "Starting LidarFusionNode...");

    // Subscriptions for each LiDAR point cloud
    sub_front_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_front_left/points", 10,
      std::bind(&LidarFusionNode::lidarCallback, this, std::placeholders::_1));
    
    sub_front_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_front_right/points", 10,
      std::bind(&LidarFusionNode::lidarCallback, this, std::placeholders::_1));
    
    sub_rear_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_rear_left/points", 10,
      std::bind(&LidarFusionNode::lidarCallback, this, std::placeholders::_1));
    
    sub_rear_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_rear_right/points", 10,
      std::bind(&LidarFusionNode::lidarCallback, this, std::placeholders::_1));

    // Publisher for fused point cloud
    fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_pointcloud", 10);
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Store the latest point cloud keyed by its frame_id
    latest_clouds_[msg->header.frame_id] = *msg;
    fusePointClouds();
  }

  void fusePointClouds()
  {
    // Wait until we have data from all 4 LiDARs
    if (latest_clouds_.size() < 4) {
      return;
    }

    // Prepare a single PCL cloud to hold the merged data
    pcl::PointCloud<pcl::PointXYZ> combined_cloud;

    // Transform and combine each cloud into 'central_lidar_link'
    for (const auto & kv : latest_clouds_) {
      const std::string & frame_id = kv.first;
      const sensor_msgs::msg::PointCloud2 & cloud_in = kv.second;

      try {
        // Lookup the transform from the cloud's frame to 'central_lidar_link'
        geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_.lookupTransform("central_lidar_link", frame_id, tf2::TimePointZero);

        // Transform the cloud using tf2_sensor_msgs::doTransform(...)
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        tf2::doTransform(cloud_in, cloud_transformed, transform_stamped);

        // Convert to PCL for merging
        pcl::PointCloud<pcl::PointXYZ> pcl_in;
        pcl::fromROSMsg(cloud_transformed, pcl_in);

        // Append to the combined cloud
        combined_cloud += pcl_in;
      }
      catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not transform point cloud from '%s' to 'central_lidar_link': %s",
          frame_id.c_str(), ex.what());
      }
    }

    // If we have at least some points, publish fused cloud
    if (!combined_cloud.empty()) {
      sensor_msgs::msg::PointCloud2 fused_msg;
      pcl::toROSMsg(combined_cloud, fused_msg);

      // Use one of the existing headers as reference
      // (e.g. from the first entry in latest_clouds_)
      auto it = latest_clouds_.begin();
      fused_msg.header = it->second.header;
      fused_msg.header.frame_id = "central_lidar_link"; // Overwrite with target frame

      fused_pub_->publish(fused_msg);
    }
  }

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_right_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rear_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rear_right_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;

  // TF buffer and listener
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Store the latest point clouds by frame_id
  std::map<std::string, sensor_msgs::msg::PointCloud2> latest_clouds_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
