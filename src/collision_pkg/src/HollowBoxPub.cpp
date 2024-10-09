#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class HollowBoxPub : public rclcpp::Node
{
public:
    HollowBoxPub()
        : Node("hollow_box_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;32mSetting Collision Box %s\033[0m", PLANNING_GROUP.c_str());

        // Initialize the planning scene publisher
        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    }

    void initialize()
    {
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        try
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
            RCLCPP_INFO(this->get_logger(), "\033[1;32mMoveGroupInterface initialized.\033[0m");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MoveGroupInterface instance: %s", e.what());
            return;
        }
        // Reset all collisions before creating the hollow box
        resetCollisions();
        // Call the function to create and publish the hollow box
        createHollowBox();
    }

private:
    void resetCollisions()
    {
        // Get the list of all known collision object names
        std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
        
        if (!object_ids.empty())
        {
            // Remove all objects from the planning scene
            planning_scene_interface_->removeCollisionObjects(object_ids);
            RCLCPP_INFO(this->get_logger(), "\033[1;32mRemoved %zu collision objects from the planning scene.\033[0m", object_ids.size());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mNo collision objects to remove.\033[0m");
        }
    }
    // Function to create a regular collision object
    moveit_msgs::msg::CollisionObject createCollisionObject(const geometry_msgs::msg::Pose& pose, const std::string& id, const std::array<double, 3>& dimensions)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = id;  // The ID of the collision object
        collision_object.header.frame_id = "world";  // Define the frame ID

        // Define the shape and pose of the collision object (rectangular box)
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;

        // Assign dimensions element by element
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = dimensions[0];
        collision_object.primitives[0].dimensions[1] = dimensions[1];
        collision_object.primitives[0].dimensions[2] = dimensions[2];

        // Update the pose of the collision object
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0] = pose;

        // Set the operation to add the object to the planning scene
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }

    // Function to create an attached collision object (for the bottom box)
    moveit_msgs::msg::AttachedCollisionObject createAttachedCollisionObject(const geometry_msgs::msg::Pose& pose, const std::string& id, const std::array<double, 3>& dimensions)
    {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.object.id = id;  // The ID of the attached object
        attached_object.object.header.frame_id = "panda_link0";  // Set the frame to which the object will be attached
        attached_object.touch_links.push_back("panda_link1");
        // Define the shape and pose of the attached object
        attached_object.object.primitives.resize(1);
        attached_object.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;

        // Assign dimensions element by element
        attached_object.object.primitives[0].dimensions.resize(3);
        attached_object.object.primitives[0].dimensions[0] = dimensions[0];
        attached_object.object.primitives[0].dimensions[1] = dimensions[1];
        attached_object.object.primitives[0].dimensions[2] = dimensions[2];

        attached_object.object.primitive_poses.resize(1);
        attached_object.object.primitive_poses[0] = pose;

        // Attach the object to a specific link
        attached_object.link_name = "panda_link0";
        attached_object.object.operation = attached_object.object.ADD;

        return attached_object;
    }
    // Function to create and apply the hollow box structure
    void createHollowBox()
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects;

        // Define the dimensions of the hollow box parts
        double length = 1.0;
        double width = 1.2;
        double height = 1.20;
        double thickness = 0.05;

        // Create the poses for each box part
        geometry_msgs::msg::Pose pose;
        
        // Bottom box (attached)
        pose.position.x = 0.6 - 0.3;
        
        pose.position.z = 0;
        attached_objects.push_back(createAttachedCollisionObject(pose, "bottom_box", {length, width, thickness}));

        // // Top box
        // pose.position.x = 0.6 - 0.3;
        // pose.position.z = height;
        // collision_objects.push_back(createCollisionObject(pose, "top_box", {length, width, thickness}));

        // front box
        pose.position.x = -length / 2.0 + 0.3;
        pose.position.y = - 0.3;
        pose.position.z = height/2;
        collision_objects.push_back(createCollisionObject(pose, "left_box", {thickness, width, height}));

        // back box
        pose.position.x = length / 2.0 + 0.3;
        collision_objects.push_back(createCollisionObject(pose, "right_box", {thickness, width, height}));

        // // Front box
        // pose.position.x = 0.15;
        // pose.position.y = width / 2.0 - 0.3;
        // collision_objects.push_back(createCollisionObject(pose, "front_box", {length, thickness, height}));

        // // Back box
        // pose.position.y = -width / 2.0 -0.3;
        // collision_objects.push_back(createCollisionObject(pose, "back_box", {length, thickness, height}));

        // Now apply the attached object (bottom)
        planning_scene_interface_->applyAttachedCollisionObject(attached_objects[0]);

        // Apply the remaining collision objects to the planning scene
        planning_scene_interface_->applyCollisionObjects(collision_objects);

        // Publish to the planning scene
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        for (const auto &obj : collision_objects)
        {
            planning_scene_msg.world.collision_objects.push_back(obj);
        }
        planning_scene_publisher_->publish(planning_scene_msg);

        RCLCPP_INFO(this->get_logger(), "\033[1;32mPublished all hollow box collision objects to the planning scene\033[0m");
    }

    const std::string PLANNING_GROUP = "panda_arm";
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HollowBoxPub>();
    node->initialize();

    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
