import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math

class TFDistanceNode(Node):
    def __init__(self):
        super().__init__('tf_distance_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            
            x_diff = trans.transform.translation.x
            y_diff = trans.transform.translation.y

            self.get_logger().info(f'X difference: {x_diff}, Y difference: {y_diff}')

        except Exception as e:
            self.get_logger().error('Could not transform map to odom: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    tf_distance_node = TFDistanceNode()
    rclpy.spin(tf_distance_node)
    tf_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
