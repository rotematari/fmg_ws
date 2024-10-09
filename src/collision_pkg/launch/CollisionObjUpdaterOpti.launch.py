import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


# ros2 launch hello_moveit hello_moveit.launch.py position_x
def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", robot_description="robot_description",package_name="franka_moveit_config")
        .robot_description(file_path="config/panda_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_arm.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/panda_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    pub_natnet_node = Node(
        name="pub_natnet_node",
        package="natnet_pub_pkg",
        executable="optitrack_pub_node",
        output="screen",
        )
    
    collision_obj_updater_node = Node(
        name="collision_updater_node",
        package="collision_pkg",
        executable="CollisionObjUpdater",
        output="screen",
        parameters=[
                moveit_config.to_dict()                       
                    ]
    )

    return LaunchDescription([
        pub_natnet_node,
        collision_obj_updater_node,
    ])