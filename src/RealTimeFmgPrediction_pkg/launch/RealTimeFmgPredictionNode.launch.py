import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
# Load YAML file into a dictionary
def load_yaml_to_dict(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)
    

def generate_launch_description():
 
    # Get the path to the YAML file
    config = os.path.join(
        get_package_share_directory('RealTimeFmgPrediction_pkg'),
        'config',
        'RealTimeConfig.yaml'
    )
    # Load the YAML file into a dictionary
    # config_dict = load_yaml_to_dict(config)
    # print(config_dict)
    # print(moveit_config.to_dict())
    realtime_prediction_pub_node = Node(
        name="realtime_prediction_pub_node",
        package="RealTimeFmgPrediction_pkg",
        executable="RealTimeFmgPredictionNode",
        output="screen",
        parameters=[
                config
                ]
    )
    return LaunchDescription([
        realtime_prediction_pub_node
    ])