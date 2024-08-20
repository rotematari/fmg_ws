#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


from natnet.NatnetReader import init_natnetClient, read_sample 

class OptiTrackPubNode(Node):
    def __init__(self):
        super().__init__('optitrack_pub_node')

        # Initialize OptiTrack client
        self.natnet = init_natnetClient()
        self.natnet.run()

        # Create a publisher for Pose messages
        self.pose_publisher = self.create_publisher(Pose, 'optitrack_pose', 10)

        # Set a timer to periodically call the callback function
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('OptiTrack publisher node has been initialized')

    def publish_pose(self):
        # Get a sample from the OptiTrack client
        sample = self.get_natnet_sample()

        # Create a Pose message
        pose_msg = Pose()

        # Set position
        pose_msg.position.x = sample['elbow'][0][0]
        pose_msg.position.y = sample['elbow'][0][1]
        pose_msg.position.z = sample['elbow'][0][2]


        # Publish the Pose message
        self.pose_publisher.publish(pose_msg)

    def get_natnet_sample(self):
        return read_sample(natnet=self.natnet)

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