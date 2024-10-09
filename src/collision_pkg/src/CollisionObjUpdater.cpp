// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include "custom_msg/msg/arm_positions.hpp"

// class CollisionUpdater : public rclcpp::Node
// {
// public:
//     CollisionUpdater()
//     : Node("collision_updater_node")
//     {   
//         // Initialize the planning scene publisher
//         planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
//         // Initialize the MoveIt PlanningSceneInterface
//         planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

//         RCLCPP_INFO(this->get_logger(), "\033[1;32mCollisionUpdater node has been initialized.\033[0m");
//     }
//     void initializePlanningSceneMonitor()
//     {

//         // pose_subscription_ = this->create_subscription<custom_msg::msg::ArmPositions>(
//         //     "/optitrack_pose", 10,
//         //     std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
//         // );
//         pose_subscription_ = this->create_subscription<custom_msg::msg::ArmPositions>(
//             "/prediction_pose", 10,
//             std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
//         );

//     }

// private:

//     void poseCallback(const custom_msg::msg::ArmPositions::SharedPtr msg)
//     {
//         // RCLCPP_INFO(this->get_logger(), "shoulder: (%f, %f, %f), elbow: (%f, %f, %f), wrist: (%f, %f, %f)",
//         //     msg->shoulder.x, msg->shoulder.y, msg->shoulder.z,
//         //     msg->elbow.x, msg->elbow.y, msg->elbow.z,
//         //     msg->wrist.x, msg->wrist.y, msg->wrist.z);
//         // RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f), (%f, %f, %f, %f)",
//         //     msg->position.x, msg->position.y, msg->position.z,
//         //     msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//         // moveit_msgs::msg::CollisionObject upperArmCollisionObject = createCollisionObject(msg, "upper_arm_collision_object");
        
//         RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f), (%f, %f, %f, %f)",
//             msg->upper_arm.position.x, msg->upper_arm.position.y, msg->upper_arm.position.z,
//             msg->upper_arm.orientation.w, msg->upper_arm.orientation.x, msg->upper_arm.orientation.y, msg->upper_arm.orientation.z);
//         moveit_msgs::msg::CollisionObject upperArmCollisionObject = createCollisionObject(msg->upper_arm, "upper_arm_collision_object");
//         moveit_msgs::msg::CollisionObject lowerArmCollisionObject = createCollisionObject(msg->lower_arm, "lower_arm_collision_object");
//         moveit_msgs::msg::CollisionObject bodyCollisionObject = createBodyAndHeadCollisionObject(msg->upper_arm, "body_collision_object");
        
//         // // Apply the collision object to the planning scene
//         // // planning_scene_interface_->applyCollisionObject(collision_object);
        
//         // Publish the updated planning scene
//         moveit_msgs::msg::PlanningScene planning_scene_msg;
//         planning_scene_msg.is_diff = true;  // This tells MoveIt to only update the changed parts of the scene
//         // planning_scene_msg.world.collision_objects.push_back(upperArmCollisionObject);
//         planning_scene_msg.world.collision_objects.push_back(upperArmCollisionObject);
//         planning_scene_msg.world.collision_objects.push_back(lowerArmCollisionObject);
//         planning_scene_msg.world.collision_objects.push_back(bodyCollisionObject);



//         planning_scene_publisher_->publish(planning_scene_msg);

//         // planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
//         // planning_scene->processCollisionObjectMsg(collision_object);
//         // // Trigger the planning scene update event to publish the changes
//         // planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
        
//         // RCLCPP_INFO(this->get_logger(), "Updated collision object pose.");
//     }
    
//     moveit_msgs::msg::CollisionObject createCollisionObject(const geometry_msgs::msg::Pose msg , std::string id)
//         {
//             moveit_msgs::msg::CollisionObject collision_object;
//             collision_object.id = id ;  // The ID of the collision object you want to update

//             // Set the frame ID of the object (e.g., "world" or the frame you are working in)
//             collision_object.header.frame_id = "world";  // The frame ID of the object

//             // Define the shape and pose of the collision object
//             collision_object.primitives.resize(1);
//             collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//             collision_object.primitives[0].dimensions = {0.37, 0.07};

//             // // Update the pose of the collision object with the new pose from OptiTrack
//             // collision_object.primitive_poses.resize(1);
//             // collision_object.primitive_poses[0].position.x = msg->position.x;
//             // collision_object.primitive_poses[0].position.y = msg->position.y;
//             // collision_object.primitive_poses[0].position.z = msg->position.z;
//             // collision_object.primitive_poses[0].orientation.w = msg->orientation.w;
//             // collision_object.primitive_poses[0].orientation.x = msg->orientation.x;
//             // collision_object.primitive_poses[0].orientation.y = msg->orientation.y;
//             // collision_object.primitive_poses[0].orientation.z = msg->orientation.z;
//             // Update the pose of the collision object with the new pose from OptiTrack
//             collision_object.primitive_poses.resize(1);
//             collision_object.primitive_poses[0].position.x = msg.position.x;
//             collision_object.primitive_poses[0].position.y = msg.position.y;
//             collision_object.primitive_poses[0].position.z = msg.position.z;
//             collision_object.primitive_poses[0].orientation.w = msg.orientation.w;
//             collision_object.primitive_poses[0].orientation.x = msg.orientation.x;
//             collision_object.primitive_poses[0].orientation.y = msg.orientation.y;
//             collision_object.primitive_poses[0].orientation.z = msg.orientation.z;

//             // Set the operation to add or update the object in the planning scene
//             collision_object.operation = collision_object.ADD;

//             return collision_object;
//     }
//         moveit_msgs::msg::CollisionObject createBodyAndHeadCollisionObject(const geometry_msgs::msg::Pose msg , std::string id)
//         {
//             moveit_msgs::msg::CollisionObject collision_object;
//             collision_object.id = id ;  // The ID of the collision object you want to update

//             // Set the frame ID of the object (e.g., "world" or the frame you are working in)
//             collision_object.header.frame_id = "world";  // The frame ID of the object

//             // Define the shape and pose of the collision object
//             collision_object.primitives.resize(1);
//             collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
//             collision_object.primitives[0].dimensions = {0.4,0.2, 0.6};  // Size of the collision object (box)

//             // // Update the pose of the collision object with the new pose from OptiTrack
//             // collision_object.primitive_poses.resize(1);
//             // collision_object.primitive_poses[0].position.x = msg->position.x;
//             // collision_object.primitive_poses[0].position.y = msg->position.y;
//             // collision_object.primitive_poses[0].position.z = msg->position.z;
//             // collision_object.primitive_poses[0].orientation.w = msg->orientation.w;
//             // collision_object.primitive_poses[0].orientation.x = msg->orientation.x;
//             // collision_object.primitive_poses[0].orientation.y = msg->orientation.y;
//             // collision_object.primitive_poses[0].orientation.z = msg->orientation.z;
//             // Update the pose of the collision object with the new pose from OptiTrack
//             collision_object.primitive_poses.resize(1);
//             collision_object.primitive_poses[0].position.x = msg.position.x + 0.2;
//             collision_object.primitive_poses[0].position.y = msg.position.y - 0.25;
//             collision_object.primitive_poses[0].position.z = msg.position.z -0.1;
//             collision_object.primitive_poses[0].orientation.w = 1;
//             // collision_object.primitive_poses[0].orientation.x = msg.orientation.x;
//             // collision_object.primitive_poses[0].orientation.y = msg.orientation.y;
//             // collision_object.primitive_poses[0].orientation.z = msg.orientation.z;

//             // Set the operation to add or update the object in the planning scene
//             collision_object.operation = collision_object.ADD;

//             return collision_object;
//     }
    
//     rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
//     rclcpp::Subscription<custom_msg::msg::ArmPositions>::SharedPtr pose_subscription_;
//     // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
//     std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
//     // Initialize PlanningSceneMonitor
//     std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CollisionUpdater>();
//     node->initializePlanningSceneMonitor();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "custom_msg/msg/arm_positions.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
class CollisionUpdater : public rclcpp::Node
{
public:
    CollisionUpdater()
    : Node("collision_updater_node")
    {   
        // Initialize the planning scene publisher
        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);


        RCLCPP_INFO(this->get_logger(), "\033[1;32mCollisionUpdater node has been initialized.\033[0m");
    }
    void initializePlanningSceneMonitor()
    {
        pose_subscription_ = this->create_subscription<custom_msg::msg::ArmPositions>(
            "/prediction_pose", 10,
            std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
        );
    }

private:

    void poseCallback(const custom_msg::msg::ArmPositions::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f), (%f, %f, %f, %f)",
        //     msg->upper_arm.position.x, msg->upper_arm.position.y, msg->upper_arm.position.z,
        //     msg->upper_arm.orientation.w, msg->upper_arm.orientation.x, msg->upper_arm.orientation.y, msg->upper_arm.orientation.z);
        
        moveit_msgs::msg::CollisionObject upperArmCollisionObject = createCollisionObject(msg->upper_arm, "upper_arm_collision_object");
        moveit_msgs::msg::CollisionObject lowerArmCollisionObject = createCollisionObject(msg->lower_arm, "lower_arm_collision_object");
        // moveit_msgs::msg::CollisionObject bodyCollisionObject = createBodyAndHeadCollisionObject(msg->upper_arm, "body_collision_object");

        // Publish the updated planning scene
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(upperArmCollisionObject);
        planning_scene_msg.world.collision_objects.push_back(lowerArmCollisionObject);
        // planning_scene_msg.world.collision_objects.push_back(bodyCollisionObject);

        planning_scene_publisher_->publish(planning_scene_msg);
        
        // Publish both upper arm and lower arm markers using a MarkerArray
        // publishMarkerLine(msg->upper_arm, msg->lower_arm);

    }
    
    moveit_msgs::msg::CollisionObject createCollisionObject(const geometry_msgs::msg::Pose msg, std::string id)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = id;
        collision_object.header.frame_id = "world";

        // Define the shape and pose of the collision object (cylinder)
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        collision_object.primitives[0].dimensions = {0.37, 0.035};

        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0] = msg;

        collision_object.operation = collision_object.ADD;

        return collision_object;
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
        
        // Publish the marker array using the marker_array_publisher_
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
    moveit_msgs::msg::CollisionObject createBodyAndHeadCollisionObject(const geometry_msgs::msg::Pose msg, std::string id)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = id;
        collision_object.header.frame_id = "world";

        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_object.primitives[0].dimensions = {0.4, 0.2, 0.6};

        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position.x = msg.position.x ;
        collision_object.primitive_poses[0].position.y = msg.position.y - 0.25;
        collision_object.primitive_poses[0].position.z = msg.position.z - 0.1;
        collision_object.primitive_poses[0].orientation.w = 1;
        collision_object.primitive_poses[0].orientation.z = -1;


        collision_object.operation = collision_object.ADD;

        return collision_object;
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::Subscription<custom_msg::msg::ArmPositions>::SharedPtr pose_subscription_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionUpdater>();
    node->initializePlanningSceneMonitor();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
