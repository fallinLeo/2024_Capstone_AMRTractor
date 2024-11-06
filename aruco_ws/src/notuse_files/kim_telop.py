#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import math

class CmdPoseNode(Node):

    def __init__(self):
        super().__init__('cmd_pose')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.callback, 10)
        self.start_pub = self.create_publisher(Int32, 'start_arduino', 10)
        self.start_sub = self.create_subscription(Int32, 'start_trailer', self.start_callback, 10)
        self.end_ = False

    def callback(self, pose):
        angular = math.atan2(pose.position.x, pose.position.z)
        linear = 0.15 * math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)
        linear_value = math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)

        self.get_logger().info(f'Received Pose: position=({pose.position.x}, {pose.position.y}, {pose.position.z}), orientation=({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})')
        self.get_logger().info(f'Calculated angular: {angular}, linear: {linear_value}')
 
        cmd = Twist()

        if linear_value <= 0.23 and abs(angular) <= 0.15:
            linear = 0.0
            angular = 0.0
            start_msg = Int32()
            start_msg.data = 1
            self.start_pub.publish(start_msg)
            self.get_logger().info('Sending "1" command to Arduino')
            
            # 종료 조건
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)  # 0을 보내서 로봇 멈추기
            self.end_ = True  # 종료 플래그 설정
            
        else:
            input_max = 3.0
            input_min = 0.23
            output_max = 0.26
            output_min = 0.1
            linear = (linear_value - input_min) * (output_max - output_min) / (input_max - input_min) + output_min
            angular = math.atan2(pose.position.x, pose.position.z)

        cmd.linear.x = -linear
        cmd.angular.z = -angular

        # try:
        #     self.vel_pub.publish(cmd)
        # except KeyboardInterrupt:
        #     print("Shutting down")

    def start_callback(self, msg):
        self.get_logger().info(f'Received message from "start_trailer" topic: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdPoseNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)  # 한 번씩만 이벤트를 처리
            if node.end_:  # 종료 조건 확인
                node.get_logger().info("Ending node as 'end_' flag is True")
                break  # 루프를 빠져나가면서 종료

    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
