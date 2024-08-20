#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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
        // Initialize the PlanningSceneMonitor
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");

        if (planning_scene_monitor_->getPlanningScene())
        {
            planning_scene_monitor_->startStateMonitor("/joint_states");
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->startWorldGeometryMonitor();
            planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

            RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor initialized successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning Scene Monitor could not be initialized.");
        }
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/optitrack_pose", 10,
            std::bind(&CollisionUpdater::poseCallback, this, std::placeholders::_1)
        );
    }

private:

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received new pose from OptiTrack: Position(x: %f, y: %f, z: %f), Orientation(x: %f, y: %f, z: %f, w: %f)",
            msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "moving_object";  // The ID of the collision object you want to update

        // Set the frame ID of the object (e.g., "world" or the frame you are working in)
        collision_object.header.frame_id = "panda_link0";  // The frame ID of the object

        // Define the shape and pose of the collision object

        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_object.primitives[0].dimensions = {1, 1, 1};  // Size of the collision object (box)
        
        // Update the pose of the collision object with the new pose from OptiTrack
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position.x = msg->position.x;
        collision_object.primitive_poses[0].position.y = msg->position.y;
        collision_object.primitive_poses[0].position.z = msg->position.z;
        collision_object.primitive_poses[0].orientation.w = 1.0;

        // Set the operation to add or update the object in the planning scene
        collision_object.operation = collision_object.ADD;

        // Apply the collision object to the planning scene
        // planning_scene_interface_->applyCollisionObject(collision_object);
        
        // Publish the updated planning scene
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;  // This tells MoveIt to only update the changed parts of the scene
        planning_scene_msg.world.collision_objects.push_back(collision_object);

        planning_scene_publisher_->publish(planning_scene_msg);

        // planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
        // planning_scene->processCollisionObjectMsg(collision_object);
        // // Trigger the planning scene update event to publish the changes
        // planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
        
        // RCLCPP_INFO(this->get_logger(), "Updated collision object pose.");
    }
    void updateDynamicObject(const geometry_msgs::msg::Pose::SharedPtr& msg)
    {
        // Get the latest planning scene
        // planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);

        // Create a CollisionObject message for the dynamic object
        moveit_msgs::msg::CollisionObject dynamic_object;
        dynamic_object.id = "dynamic_object";
        dynamic_object.header.frame_id = "world"; // or the relevant frame

        dynamic_object.primitives.resize(1);
        dynamic_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        dynamic_object.primitives[0].dimensions = {0.5, 0.5, 0.5};  // Example size

        dynamic_object.primitive_poses.resize(1);
        dynamic_object.primitive_poses[0].position.x = msg->position.x;
        dynamic_object.primitive_poses[0].position.y = msg->position.y;
        dynamic_object.primitive_poses[0].position.z = msg->position.z;
        dynamic_object.primitive_poses[0].orientation.w = 1.0;

        dynamic_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
        // planning_scene->processCollisionObjectMsg(dynamic_object);
    }
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
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
