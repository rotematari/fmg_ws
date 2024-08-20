import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

# pos_x:=0.28 pos_y:=-0.2 pos_z:=0.5 ori_x:=0.0 ori_y:=0.0 ori_z:=0.0 ori_w:=1.0
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
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
        package="collision_pkg",
        executable="OptiTrackPubNode.py",
        output="screen",
        )
    collision_obj_updater_node = Node(
        name="collision_checker_node",
        package="collision_pkg",
        executable="CollisionObjUpdater",
        output="screen",
        parameters=[
                moveit_config.to_dict()                       
                    ]
    )
    collision_checker_node = Node(
        name="collision_checker_node",
        package="collision_pkg",
        executable="CollisionCheckerNode",
        output="screen",
        parameters=[
                moveit_config.to_dict()                       
                    ]
    )
    return LaunchDescription([
        pub_natnet_node,
        collision_obj_updater_node,
        collision_checker_node,



    ])