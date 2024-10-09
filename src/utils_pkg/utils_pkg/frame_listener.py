import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call the function to get transform periodically
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            # Get the transformation from 'world' to 'panda_link0' (or any frame of interest)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('world', 'panda_link8', now)

            # Print the translation and rotation (position and orientation)
            self.get_logger().info(f"Position: {trans.transform.translation}")
            self.get_logger().info(f"Orientation: {trans.transform.rotation}")
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
