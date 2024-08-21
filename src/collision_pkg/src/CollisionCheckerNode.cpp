#include <rclcpp/rclcpp.hpp>
#include <memory> // Include the header for std::shared_ptr

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


class CollisionCheckerNode : public rclcpp::Node
{

public:
    CollisionCheckerNode()
    : Node("collision_checker_node")
    {
        // RCLCPP_INFO(this->get_logger(), "CollisionCheckerNode has been initialized.");
    }
    void initialize(){

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
        // // Load the robot model
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(),"robot_description");
        // robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        // robot_model_ = robot_model_loader_->getModel();
        robot_model_ = robot_model_loader.getModel();
        // Initialize the PlanningScene
        // planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
        RCLCPP_INFO(this->get_logger(), "Planning Scene Initialized."); 
        // Initialize the MoveGroupInterface for the Panda arm
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
        RCLCPP_INFO(this->get_logger(), "Move Group Initialized.");
        // Log the robot model information
        RCLCPP_INFO(this->get_logger(), "Robot Model: %s", robot_model_->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "Robot Model Root Link: %s", robot_model_->getRootLinkName().c_str());
        RCLCPP_INFO(this->get_logger(), "Robot Model Joint Model Group: %s", move_group_->getName().c_str());
        // Set up a timer to periodically check for collisions
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CollisionCheckerNode::checkForCollision, this)
        );
        RCLCPP_INFO(this->get_logger(), "Collision Checker Node Initialized.");

    }


private:

    void checkForCollision()
    {   
        RCLCPP_INFO(this->get_logger(), "Checking for collisions...");        
        // Get the latest planning scene
        planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);

        if (!planning_scene)
        {
            RCLCPP_WARN(this->get_logger(), "Unable to get the latest planning scene.");
            return;
        }
        // printCollisionObjects();
        // RCLCPP_INFO(this->get_logger(), "Checking for collisions...");
        // Get the current state of the robot
        planning_scene->getCurrentStateNonConst().update();
        
        // RCLCPP_INFO(this->get_logger(), "Current state updated.");
        // moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
        // RCLCPP_INFO(this->get_logger(), "Have current state.");
        
        // Check for collisions
        collision_detection::CollisionRequest collision_request;
        collision_request.contacts = true;
        collision_detection::CollisionResult collision_result;
        // collision_result.clear();
        // planning_scene->checkCollision(collision_request, collision_result, current_state);
        planning_scene->checkCollision(collision_request, collision_result);

        RCLCPP_INFO(this->get_logger(), "state is %s", planning_scene->isStateColliding()? "in collision" : "not in collision");
        if (collision_result.collision)
        {
            RCLCPP_WARN(this->get_logger(), "Collision detected! Stopping the robot.");
            RCLCPP_INFO(this->get_logger(), "%zu collision points found", collision_result.contact_count);
            move_group_->stop();
            // printCollisionContacts(this->get_logger(), collision_result);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No collision detected.");
        }
    }
    void printCollisionContacts(const rclcpp::Logger& logger, const collision_detection::CollisionResult& collision_result)
    {
        for (const auto& contact_pair : collision_result.contacts)
        {
            const std::pair<std::string, std::string>& object_pair = contact_pair.first;
            // const std::vector<collision_detection::Contact>& contacts = contact_pair.second;

            // Log the object names involved in the contact
            RCLCPP_INFO(logger, "Contact between: %s and %s", object_pair.first.c_str(), object_pair.second.c_str());

            // // Iterate over the contacts
            // for (const auto& contact : contacts)
            // {
            //     RCLCPP_INFO(logger, "  Contact point: [%.3f, %.3f, %.3f]", contact.pos.x(), contact.pos.y(), contact.pos.z());
            //     RCLCPP_INFO(logger, "  Normal: [%.3f, %.3f, %.3f]", contact.normal.x(), contact.normal.y(), contact.normal.z());
            //     RCLCPP_INFO(logger, "  Depth: %.3f", contact.depth);
            // }
        }
    }
    
    void printCollisionObjects()
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Get the names of all known collision objects
        std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();

        if (object_ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "No collision objects found in the planning scene.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu collision objects:", object_ids.size());

        // Iterate through all known objects and print their details
        for (const auto& object_id : object_ids)
        {
            RCLCPP_INFO(this->get_logger(), "Collision Object ID: %s", object_id.c_str());

            // Retrieve the actual collision object
            std::map<std::string, moveit_msgs::msg::CollisionObject> collision_objects = planning_scene_interface.getObjects({object_id});

            if (collision_objects.find(object_id) != collision_objects.end())
            {
                const moveit_msgs::msg::CollisionObject& obj = collision_objects[object_id];

                // Print the object's frame_id and poses
                RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", obj.header.frame_id.c_str());

                for (size_t i = 0; i < obj.primitive_poses.size(); ++i)
                {
                    
                    const auto& pose = obj.primitive_poses[i];
                    RCLCPP_INFO(this->get_logger(), "  Primitive Pose %zu: [x: %.3f, y: %.3f, z: %.3f, qw: %.3f, qx: %.3f, qy: %.3f, qz: %.3f]",
                                i, pose.position.x, pose.position.y, pose.position.z,
                                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "  Collision object with ID %s was not found.", object_id.c_str());
            }
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionCheckerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
