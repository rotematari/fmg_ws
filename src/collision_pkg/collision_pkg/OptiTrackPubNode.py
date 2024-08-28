#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from custom_msg.msg import ArmPositions
import numpy as np
from natnet.NatnetReader import init_natnetClient, read_sample 
from scipy.spatial.transform import Rotation as R


class OptiTrackPubNode(Node):
    def __init__(self):
        super().__init__('optitrack_pub_node')

        # Initialize OptiTrack client
        self.natnet = init_natnetClient()
        self.natnet.run()

        # Create a publisher for Pose messages
        self.pose_publisher = self.create_publisher(ArmPositions, 'optitrack_pose', 10)

        # Set a timer to periodically call the callback function
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('OptiTrack publisher node has been initialized')

    def publish_pose(self):
        
        # Get a sample from the OptiTrack client
        sample = self.get_natnet_sample()

        # Create an ArmPositions message
        arm_positions_msg = ArmPositions()
        
        QsholderToElbow,shoulder_position = self.calculate_quaternion(sample['shoulder'][0], sample['elbow'][0])
        QelbowToWrist, elbow_position = self.calculate_quaternion(sample['elbow'][0], sample['wrist'][0])

        arm_positions_msg.upper_arm.position.x = sample['shoulder'][0][0] + shoulder_position[0]
        arm_positions_msg.upper_arm.position.y = sample['shoulder'][0][1] +shoulder_position[1]
        arm_positions_msg.upper_arm.position.z = sample['shoulder'][0][2] +shoulder_position[2]
        
        arm_positions_msg.upper_arm.orientation.x = QsholderToElbow[0]
        arm_positions_msg.upper_arm.orientation.y = QsholderToElbow[1]
        arm_positions_msg.upper_arm.orientation.z = QsholderToElbow[2]
        arm_positions_msg.upper_arm.orientation.w = QsholderToElbow[3]

        arm_positions_msg.lower_arm.position.x = sample['elbow'][0][0] + elbow_position[0]
        arm_positions_msg.lower_arm.position.y = sample['elbow'][0][1] +elbow_position[1]
        arm_positions_msg.lower_arm.position.z = sample['elbow'][0][2] +elbow_position[2]
        arm_positions_msg.lower_arm.orientation.x = QelbowToWrist[0]
        arm_positions_msg.lower_arm.orientation.y = QelbowToWrist[1]
        arm_positions_msg.lower_arm.orientation.z = QelbowToWrist[2]
        arm_positions_msg.lower_arm.orientation.w = QelbowToWrist[3]

        # Publish the Pose message
        self.pose_publisher.publish(arm_positions_msg)
        

    def get_natnet_sample(self):
        return read_sample(natnet=self.natnet)
    
    def calculate_quaternion(self,point_1, point_2):
        
        # Step 1: Calculate the vector from the point_1 to the point_2
        vector = np.array(point_2) - np.array(point_1)
        position = vector/2
        # Step 2: Normalize the vector
        vector_norm = vector / np.linalg.norm(vector)
        
        # Step 3: Initial z-axis (0, 0, 1)
        z_axis = np.array([0, 0, 1])
        
        rotation = R.align_vectors(z_axis, vector_norm)

        # Step 4: Calculate the rotation axis (cross product of z_axis and vector_norm)
        rotation_axis = np.cross(z_axis, vector_norm)
        
        # Step 5: Calculate the angle (dot product of z_axis and vector_norm)
        angle = np.arccos(np.dot(z_axis, vector_norm))
        
        # Step 6: Normalize the rotation axis
        if np.linalg.norm(rotation_axis) != 0:
            rotation_axis_norm = rotation_axis / np.linalg.norm(rotation_axis)
        else:
            rotation_axis_norm = np.array([0, 0, 0])
        
        # Step 7: Calculate the quaternion
        w = np.cos(angle / 2)
        x, y, z = rotation_axis_norm * np.sin(angle / 2)
        
        quaternion = np.array([x, y, z, w])
        
        # return rotation.as_quat()
        return quaternion , position
    
    def translate_in_local_frame(self,position, orientation, local_translation):
        # Convert quaternion to rotation matrix
        rotation = R.from_quat(orientation)
        
        # Rotate the local translation vector to align with the world frame
        world_translation = rotation.apply(local_translation)
        
        # Update the position
        new_position = np.array(position) + world_translation
        
        return new_position
    
def main(args=None):
    rclpy.init(args=args)
    optitrack_node = OptiTrackPubNode()

    try:
        rclpy.spin(optitrack_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        optitrack_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()