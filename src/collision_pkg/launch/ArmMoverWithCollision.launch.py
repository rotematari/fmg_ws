import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# ros2 launch hello_moveit hello_moveit.launch.py position_x
def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", robot_description="robot_description",package_name="franka_moveit_config")
        .robot_description(file_path="config/panda_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_arm.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/panda_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl",pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    # print(moveit_config.to_dict())

    hollow_box_pub =Node(
        name="hollow_box_pub",
        package="collision_pkg",
        executable="HollowBoxPub",
        output="screen",
        parameters=[
                moveit_config.to_dict()                       
                    ]
    )
    arm_mover_with_collision_node = Node(
        name="collision_checker_node",
        package="collision_pkg",
        executable="ArmMoverWithCollisionNode",
        output="screen",
        parameters=[
                moveit_config.to_dict()                       
                    ]
    )
    return LaunchDescription([
        hollow_box_pub,
        arm_mover_with_collision_node
    ])