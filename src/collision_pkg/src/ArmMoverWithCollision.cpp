#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>




class ArmMoverWithCollision : public rclcpp::Node
{
public:
ArmMoverWithCollision()
: Node("collsion_checker_node")
{
    RCLCPP_INFO(this->get_logger(), "Planning group is: %s", PLANNING_GROUP.c_str());
}
void initialize()
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

    // Initialize the RobotModelLoader with a shared pointer
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");

    // Now you can use robot_model_loader_ as needed
    robot_model_ = robot_model_loader_->getModel();

    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    get_planning_plugin_names()
}
private:

void get_planning_plugin_names()
{
    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!this->get_parameter("move_group.planning_plugins", planner_plugin_names))
        RCLCPP_FATAL(this->get_logger(), "Could not find planner plugin names");

    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        RCLCPP_FATAL(this->get_logger(), "Exception while creating planning plugin loader %s", ex.what());
    }

    if (planner_plugin_names.empty())
    {
        RCLCPP_ERROR(this->get_logger(),
                    "No planner plugins defined. Please make sure that the planning_plugins parameter is not empty.");
        return -1;
    }
    // set planner name 
    const auto& planner_name = planner_plugin_names.at(0);

    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
        if (!planner_instance->initialize(robot_model, this,
                                        this->get_namespace()))
        RCLCPP_FATAL(this->get_logger(), "Could not initialize planner instance");
        RCLCPP_INFO(this->get_logger(), "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        RCLCPP_ERROR(this->get_logger(), "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_name.c_str(),
                    ex.what(), ss.str().c_str());
    }
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    
}
};


const std::string PLANNING_GROUP = "panda_arm";
std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
moveit::core::RobotModelPtr robot_model_;
moveit::core::RobotStatePtr robot_state_;
std::vector<std::string> planner_plugin_names;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMoverWithCollision>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}