#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "custom_msg/msg/arm_positions.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MarkerArrayPub : public rclcpp::Node
{
public:
    MarkerArrayPub()
    : Node("marker_array_pub_node")
    {   
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        RCLCPP_INFO(this->get_logger(), "\033[1;32mMarkerArrayPub node has been initialized.\033[0m");

        pose_subscription_ = this->create_subscription<custom_msg::msg::ArmPositions>(
            "/prediction_pose", 10,
            std::bind(&MarkerArrayPub::poseCallback, this, std::placeholders::_1)
        );
    }

private:
    void poseCallback(const custom_msg::msg::ArmPositions::SharedPtr msg)
    {
        // Publish the markers using MarkerArray
        publishMarkerLine(msg->upper_arm, msg->lower_arm);
    }

    void publishMarkerLine(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& end_pose) {
        visualization_msgs::msg::MarkerArray marker_array;

        // Create upper arm marker
        visualization_msgs::msg::Marker upper_arm_marker = createMarker(
            start_pose,             // Pose
            "upper_arm",            // Namespace
            0,                      // Marker ID
            {1.0f, 0.0f, 0.0f, 1.0f}, // Color (red, fully opaque)
            rclcpp::Duration(0, 0)  // Lifetime (indefinite)
        );
        
        // Create lower arm marker
        visualization_msgs::msg::Marker lower_arm_marker = createMarker(
            end_pose,               // Pose
            "lower_arm",            // Namespace
            1,                      // Marker ID
            {0.0f, 1.0f, 0.0f, 1.0f}, // Color (green, fully opaque)
            rclcpp::Duration(0, 0)  // Lifetime (indefinite)
        );

        // Add both markers to the marker array
        marker_array.markers.push_back(upper_arm_marker);
        marker_array.markers.push_back(lower_arm_marker);
        
        // Publish the marker array
        marker_array_publisher_->publish(marker_array);
    }

    visualization_msgs::msg::Marker createMarker(
        const geometry_msgs::msg::Pose& pose,
        const std::string& ns,
        int marker_id,
        const std::array<float, 4>& color,
        const rclcpp::Duration& lifetime,
        const std::string& frame_id = "world") 
    {
        visualization_msgs::msg::Marker marker;

        // Set the frame ID and timestamp
        marker.header.frame_id = frame_id;
        marker.header.stamp = rclcpp::Node::now();

        // Set the namespace and marker ID
        marker.ns = ns;
        marker.id = marker_id;

        // Set the marker type (cylinder)
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker
        marker.pose = pose;

        // Set the scale of the marker
        marker.scale.x = 0.07;  // Diameter of the cylinder (x and y)
        marker.scale.y = 0.07;  // Diameter of the cylinder (x and y)
        marker.scale.z = 0.37;  // Height of the cylinder (z)

        // Set the color of the marker (RGBA)
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        // Set the marker lifetime
        marker.lifetime = lifetime;

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::Subscription<custom_msg::msg::ArmPositions>::SharedPtr pose_subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerArrayPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
