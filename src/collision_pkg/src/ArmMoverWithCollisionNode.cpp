#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



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
    
    // const rclcpp::NodeOptions& options = this->get_node_options();
    // options.automatically_declare_parameters_from_overrides(true);
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

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_));
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    try
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create MoveGroupInterface instance: %s", e.what());
    }
    
    get_planning_plugin_names();
    RCLCPP_INFO(this->get_logger(), "Planning Scene Initialized.");
    set_right_pose_goal();
    RCLCPP_INFO(this->get_logger(), "Right pose goal set.");
    set_left_pose_goal();
    RCLCPP_INFO(this->get_logger(), "Left pose goal set.");
    set_collision_object();
    RCLCPP_INFO(this->get_logger(), "Collision object set.");
    set_move_group();
    RCLCPP_INFO(this->get_logger(), "Move group set.");


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ArmMoverWithCollision::move_from_right_to_left, this)
    );
}
private:
void set_right_pose_goal()
{
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    
    target_pose_right.position.x = 0.5;
    target_pose_right.position.y = -0.4;
    target_pose_right.position.z = 0.4;
    target_pose_right.orientation.x = 1;
    target_pose_right.orientation.y = 0;
    target_pose_right.orientation.z = 0;
    target_pose_right.orientation.w = 0;
}
void set_left_pose_goal()
{
    
    target_pose_left.position.x = 0.5;
    target_pose_left.position.y = 0.4;
    target_pose_left.position.z = 0.4;
    target_pose_left.orientation.x = 1;
    target_pose_left.orientation.y = 0;
    target_pose_left.orientation.z = 0;
    target_pose_left.orientation.w = 0;
}
void set_collision_object()
{
        // Table collision object
        // Create collision object for the robot to avoid
        auto const collision_object_1 = [frame_id =
                                            move_group_->getPlanningFrame()] {
        moveit_msgs::msg::AttachedCollisionObject collision_object;
        collision_object.link_name = "panda_link0";
        // collision_object.touch_links.push_back(frame_id);
        collision_object.touch_links.push_back("panda_link1");
        // moveit_msgs::msg::CollisionObject collision_object;
        // collision_object.header.frame_id = frame_id;
        // collision_object.id = "table";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.0;
        primitive.dimensions[primitive.BOX_Y] = 1.2;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.0;

        collision_object.object.primitives.push_back(primitive);
        
        collision_object.object.primitive_poses.push_back(box_pose);
        collision_object.object.operation = collision_object.object.ADD;

        return collision_object;
    }();
    // Add the collision object to the planning scene
    planning_scene_interface_->applyAttachedCollisionObject(collision_object_1);


}
void get_planning_plugin_names()
{
    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    
    if (!this->get_parameter("OMPL.planner_plugin", planner_plugin_names))
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
        // return ;
    }
    // set planner name 
    // const auto& planner_name = planner_plugin_names.at(0);
    const std::string planner_name = "ompl_interface/OMPLPlanner";


    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
        if (!planner_instance->initialize(robot_model_, shared_from_this(),
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

}
void set_move_group(){

    RCLCPP_INFO(this->get_logger(), "Move group interface initialized.");
    move_group_->setPlannerId("BiTRRTkConfigDefault");
    RCLCPP_INFO(this->get_logger(), "Planner ID set.");
    move_group_->setMaxVelocityScalingFactor(0.1);
    RCLCPP_INFO(this->get_logger(), "Max velocity scaling factor set.");
    move_group_->setMaxAccelerationScalingFactor(0.1);
    RCLCPP_INFO(this->get_logger(), "Max acceleration scaling factor set.");
    move_group_->setWorkspace(0.350, 0.50, 0.0, 1.0, 0.50, 1.50);
    RCLCPP_INFO(this->get_logger(), "Workspace set.");

}
// void def_workspace_bounds()
// {
//     // Define the workspace bounds
//     // Define the workspace bounds
//     workspace = std::make_shared<moveit_msgs::msg::WorkspaceParameters>();
//     workspace.header.frame_id = "panda_link0";
//     workspace.min_corner.x = -0.350;
//     workspace.min_corner.y = -0.50;
//     workspace.min_corner.z = -0.0;
//     workspace.max_corner.x = 1.0;
//     workspace.max_corner.y = 0.50;
//     workspace.max_corner.z = 1.50;

// }
bool plan_and_execute_to_pose(const geometry_msgs::msg::Pose& pose)
{
    move_group_->setStartStateToCurrentState();

    // Set the target pose
    move_group_->setPoseTarget(pose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan to pose");\
        return false;
    }
    // RCLCPP_INFO(this->logger, "Visualizing plan 1 as trajectory line");
    // moveit_visual_tools.publishAxisLabeled(pose, "pose1");
    // moveit_visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
    move_group_->asyncExecute(plan);
    return true;
}
void move_from_right_to_left()
{
    if (moveRight)
    {
        // Plan and execute to the right target pose
        if (!plan_and_execute_to_pose(target_pose_right))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to right target pose");
            return;
        }
        moveRight = false;
    }
    else
    {
        // Plan and execute to the left target pose
        if (!plan_and_execute_to_pose(target_pose_left))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to left target pose");
            return;
        }
        moveRight = true;
    }
}

const std::string PLANNING_GROUP = "panda_arm";
bool moveRight = true;
rclcpp::TimerBase::SharedPtr timer_;
std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
moveit::core::RobotModelPtr robot_model_;
moveit::core::RobotStatePtr robot_state_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
std::vector<std::string> planner_plugin_names;
geometry_msgs::msg::Pose target_pose_left;
geometry_msgs::msg::Pose target_pose_right;
std::shared_ptr<moveit_msgs::msg::WorkspaceParameters> workspace_parameters;
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

};




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMoverWithCollision>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}