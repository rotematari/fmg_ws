#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "custom_msg/msg/arm_positions.hpp"

class CollisionUpdater : public rclcpp::Node
{
public:
    CollisionUpdater()
    : Node("collision_updater_node")
    {   
        // Initialize the planning scene publisher
        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
        // Initialize the MoveIt PlanningSceneInterface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        RCLCPP_INFO(this->get_logger(), "CollisionUpdater node has been initialized.");
    }
    void initializePlanningSceneMonitor()
    {
        // // Initialize the PlanningSceneMonitor
        // planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_destion");

        // if (planning_scene_monitor_->getPlanningScene())
        // {
        //     planning_scene_monitor_->startStateMonitor("/joint_states");
        //     planning_scene_monitor_->startSceneMonitor();
        //     planning_scene_monitor_->startWorldGeometryMonitor();
        //     planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

        //     RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor initialized successfully.");
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Planning Scene Monitor could not be initialized.");
        // }
        pose_subscription_ = this->create_subscription<custom_msg::msg::ArmPositions>(
            "/optitrack_pose", 10,
            std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
        );
        // pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        //     "/optitrack_pose", 10,
        //     std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
        // );
    }

private:

    void poseCallback(const custom_msg::msg::ArmPositions::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "shoulder: (%f, %f, %f), elbow: (%f, %f, %f), wrist: (%f, %f, %f)",
        //     msg->shoulder.x, msg->shoulder.y, msg->shoulder.z,
        //     msg->elbow.x, msg->elbow.y, msg->elbow.z,
        //     msg->wrist.x, msg->wrist.y, msg->wrist.z);
        // RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f), (%f, %f, %f, %f)",
        //     msg->position.x, msg->position.y, msg->position.z,
        //     msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        // moveit_msgs::msg::CollisionObject upperArmCollisionObject = createCollisionObject(msg, "upper_arm_collision_object");
        
        moveit_msgs::msg::CollisionObject upperArmCollisionObject = createCollisionObject(msg->upper_arm, "upper_arm_collision_object");
        moveit_msgs::msg::CollisionObject lowerArmCollisionObject = createCollisionObject(msg->lower_arm, "lower_arm_collision_object");
        
        // // Apply the collision object to the planning scene
        // // planning_scene_interface_->applyCollisionObject(collision_object);
        
        // Publish the updated planning scene
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;  // This tells MoveIt to only update the changed parts of the scene
        // planning_scene_msg.world.collision_objects.push_back(upperArmCollisionObject);
        planning_scene_msg.world.collision_objects.push_back(upperArmCollisionObject);
        planning_scene_msg.world.collision_objects.push_back(lowerArmCollisionObject);


        planning_scene_publisher_->publish(planning_scene_msg);

        // planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
        // planning_scene->processCollisionObjectMsg(collision_object);
        // // Trigger the planning scene update event to publish the changes
        // planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
        
        // RCLCPP_INFO(this->get_logger(), "Updated collision object pose.");
    }
    
    moveit_msgs::msg::CollisionObject createCollisionObject(const geometry_msgs::msg::Pose msg , std::string id)
        {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = id ;  // The ID of the collision object you want to update

            // Set the frame ID of the object (e.g., "world" or the frame you are working in)
            collision_object.header.frame_id = "panda_link0";  // The frame ID of the object

            // Define the shape and pose of the collision object
            collision_object.primitives.resize(1);
            collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            collision_object.primitives[0].dimensions = {0.4, 0.1};  // Size of the collision object (box)

            // // Update the pose of the collision object with the new pose from OptiTrack
            // collision_object.primitive_poses.resize(1);
            // collision_object.primitive_poses[0].position.x = msg->position.x;
            // collision_object.primitive_poses[0].position.y = msg->position.y;
            // collision_object.primitive_poses[0].position.z = msg->position.z;
            // collision_object.primitive_poses[0].orientation.w = msg->orientation.w;
            // collision_object.primitive_poses[0].orientation.x = msg->orientation.x;
            // collision_object.primitive_poses[0].orientation.y = msg->orientation.y;
            // collision_object.primitive_poses[0].orientation.z = msg->orientation.z;
            // Update the pose of the collision object with the new pose from OptiTrack
            collision_object.primitive_poses.resize(1);
            collision_object.primitive_poses[0].position.x = msg.position.x;
            collision_object.primitive_poses[0].position.y = msg.position.y;
            collision_object.primitive_poses[0].position.z = msg.position.z;
            collision_object.primitive_poses[0].orientation.w = msg.orientation.w;
            collision_object.primitive_poses[0].orientation.x = msg.orientation.x;
            collision_object.primitive_poses[0].orientation.y = msg.orientation.y;
            collision_object.primitive_poses[0].orientation.z = msg.orientation.z;

            // Set the operation to add or update the object in the planning scene
            collision_object.operation = collision_object.ADD;

            return collision_object;
    }
    
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::Subscription<custom_msg::msg::ArmPositions>::SharedPtr pose_subscription_;
    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    // Initialize PlanningSceneMonitor
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
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
