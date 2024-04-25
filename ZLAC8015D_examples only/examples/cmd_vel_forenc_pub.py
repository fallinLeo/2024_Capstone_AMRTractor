#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.enc_L = self.create_publisher(Int16,'/encoderL',10)
        self.enc_R = self.create_publisher(Int16,'/encoderR',10)
        self.cmds = [0.0,0.0]
        self.timer = self.create_timer(0.001, self.state_sub)
    

    def cmd_vel_callback(self, msg):
        # self.cmds[0] = int(((msg.linear.x - msg.angular.z)*0.285)/0.05)
        # self.cmds[1] = int(-((msg.linear.x + msg.angular.z)*0.285)/0.05)
        self.cmds[0] = int((msg.linear.x - 0.285 * msg.angular.z) / 0.1)
        self.cmds[1] = int((msg.linear.x + 0.285 * msg.angular.z) / 0.1)
        
        
        #pub_message()
        # self.motors.set_rpm(self.cmds[0], self.cmds[1])  # RPM 설정

        # msg1 = Int16()
        # msg2 = Int16()
        # msg1.data = int(tick_L)
        # msg2.data = int(-tick_R) # 엔코더 RIGHT값은 -로 퍼블리시뒤어 -값 추가함.
        # print(msg1.data)
        # self.encoderL_pub.publish(msg1)
        # self.encoderR_pub.publish(msg2)

    def state_sub(self):
        msg1 = Int16()
        msg2 = Int16()
        msg1.data = int(self.cmds[0])
        msg2.data = int(self.cmds[1])
        print(f"left : {self.cmds[0]}, right : {self.cmds[1]}")
        self.enc_L.publish(msg1)
        self.enc_R.publish(msg2)



def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()
    
    try:
        rclpy.spin(data_subscriber)
    except KeyboardInterrupt:
        data_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

