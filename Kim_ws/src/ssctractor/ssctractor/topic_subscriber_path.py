import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class TopicSuscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')

        self.plan_sub = self.create_subscription(
            Path,
            'plan_hybrid',
            self.plan_callback,
            10)
        self.trailer_sub = self.create_subscription(
            Path,
            'pathTrailer',
            self.trailer_callback,
            10)
        self.hybrid_tractorpath = self.create_publisher(Path,'plan_hybrid',10)
        self.hybrid_trailerpath = self.create_publisher(Path,'pathTrailer',10)
        # self.imu_sub = self.create_subscription(
        #     Imu,
        #     '/imu_controller/out',
        #     self.imu_callback,
        #     10)
       

    
    def plan_callback(self, msg):
        self.hybrid_tractorpath.publish(msg)

    def trailer_callback(self, msg):
        self.hybrid_trailerpath.publish(msg)

    # def imu_callback(self, msg):
    #     self.odom_msg.pose.pose.orientation = msg.orientation

    

   


def main(args=None):
    rclpy.init(args=args)

    topic_sub = TopicSuscriber()

    rclpy.spin(topic_sub)

    topic_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
