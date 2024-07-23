#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>


using namespace std::chrono_literals;

enum State {
    FREE = 0,
    WARNING,
    DANGER
};


class TwistmuxSafetyStop : public rclcpp::Node
{
public:
    TwistmuxSafetyStop() : Node("twistmux_safety_stop_node"),
                           is_first_msg_(true),
                           state_(State::FREE),
                           prev_state_(State::FREE)
    {
        declare_parameter<double>("danger_distance", 2.5); // m
        declare_parameter<double>("warning_distance", 3.5); // m
        declare_parameter<std::string>("pcl_topic", "fused_pointcloud");
        declare_parameter<std::string>("safety_stop_topic", "safety_stop");
        declare_parameter<std::string>("safety_warning_topic", "safety_warning");
        declare_parameter<std::string>("markers_reference_link", "base_link");


        danger_distance_ = get_parameter("danger_distance").as_double();
        warning_distance_ = get_parameter("warning_distance").as_double();
        pcl_topic_ = get_parameter("pcl_topic").as_string();
        safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
        safety_warning_topic_ = get_parameter("safety_warning_topic").as_string();
        markers_reference_link_ = get_parameter("markers_reference_link").as_string();

        pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(pcl_topic_, 10, std::bind(&TwistmuxSafetyStop::pclCallback, this, std::placeholders::_1));
        safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic_, 10);
        safety_warning_pub_ = create_publisher<std_msgs::msg::Bool>(safety_warning_topic_, 10);
        zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("zones", 10);
        min_distance_pub_ = create_publisher<std_msgs::msg::Float32>("min_obstacle_distance", 10);


        RCLCPP_INFO(get_logger(), "Twistmux safety stop node has started.");

        // Marker zones in rviz
        visualization_msgs::msg::Marker warning_zone;
        warning_zone.header.frame_id = markers_reference_link_;  //Zones will be placed with respect of this link
        warning_zone.id = 0;
        warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        warning_zone.action = visualization_msgs::msg::Marker::ADD;
        warning_zone.scale.x = warning_distance_ * 2;
        warning_zone.scale.y = warning_distance_ * 2;
        warning_zone.scale.z = 0.1;  // Height to the cylinder
        warning_zone.color.r = 1.0;
        warning_zone.color.g = 0.984;
        warning_zone.color.b = 0.0;
        warning_zone.color.a = 0.5;
        warning_zone.pose.position.z = 0.005;  // Height above ground
        zones_.markers.push_back(warning_zone);

        visualization_msgs::msg::Marker danger_zone = warning_zone;
        danger_zone.id = 1;
        danger_zone.scale.x = danger_distance_ * 2;
        danger_zone.scale.y = danger_distance_ * 2;
        danger_zone.color.r = 1.0;
        danger_zone.color.g = 0.0;
        danger_zone.color.b = 0.0;
        danger_zone.color.a = 0.5;
        danger_zone.pose.position.z = 0.008; //Slightly above warning zone
        zones_.markers.push_back(danger_zone);
    }

private:
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        State new_state = State::FREE;
        float min_distance = std::numeric_limits<float>::max();

        // Fallback to calculating distance from x, y, z
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        // Check distances
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float distance = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y) + (*iter_z) * (*iter_z));

            if (distance < danger_distance_) {
                new_state = State::DANGER;
                break;
            } else if (distance < warning_distance_) {
                new_state = State::WARNING;
            }
        }

        // Apply hysteresis
        if (new_state != state_) {
            if (new_state > state_) {
                // Immediately escalate to a more dangerous state
                state_ = new_state;
            } else {
                // Require multiple consecutive readings for de-escalation
                if (++state_change_counter_ >= 2) {
                    state_ = new_state;
                    state_change_counter_ = 0;
                }
            }
        } else {
            state_change_counter_ = 0;
        }

        // Logging and actions
        if(state_ != prev_state_)
        {
            std_msgs::msg::Bool is_safety_stop;
            std_msgs::msg::Bool is_safety_warning;

            if(state_ == State::WARNING)
            {
                is_safety_stop.data = false;
                is_safety_warning.data = true;
                zones_.markers.at(0).color.a = 1.0;
                zones_.markers.at(1).color.a = 0.5;
            }
            else if(state_ == State::DANGER)
            {
                is_safety_stop.data = true;
                is_safety_warning.data = true;
                zones_.markers.at(0).color.a = 1.0;
                zones_.markers.at(1).color.a = 1.0;
            }
            else if(state_ == State::FREE)
            {
                is_safety_stop.data = false;
                is_safety_warning.data = false;
                zones_.markers.at(0).color.a = 0.5;
                zones_.markers.at(1).color.a = 0.5;
            }

            prev_state_ = state_;
            safety_warning_pub_->publish(is_safety_warning);
            safety_stop_pub_->publish(is_safety_stop);
        }

        //Always publish safety stop message
        std_msgs::msg::Bool is_safety_stop;
        is_safety_stop.data = (state_ == State::DANGER);
        safety_stop_pub_->publish(is_safety_stop);

        //Publish minimum distance so we have a way to exit the safety stop when we increase the distance from the object
        std_msgs::msg::Float32 distance_msg;
        distance_msg.data = min_distance;
        min_distance_pub_->publish(distance_msg);


        if(is_first_msg_)
        {
            for(auto & zone : zones_.markers)
            {
                zone.header.frame_id = markers_reference_link_;
            }
            is_first_msg_ = false;
        }
        zones_pub_->publish(zones_);
    
    }



    double danger_distance_, warning_distance_, min_detection_distance_;
    std::string pcl_topic_, safety_stop_topic_, safety_warning_topic_, markers_reference_link_;
    bool is_first_msg_;
    State state_, prev_state_;
    int state_change_counter_ = 0;
    visualization_msgs::msg::MarkerArray zones_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_warning_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_pub_;


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistmuxSafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




