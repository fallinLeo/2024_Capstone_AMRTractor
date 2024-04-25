#!/usr/bin/env python3

from zlac8015d import ZLAC8015D
import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


motors = ZLAC8015D.Controller(port="/dev/ttyUSB1")

motors.disable_motor()

motors.set_accel_time(1000,1000)
motors.set_decel_time(1000,1000)

motors.set_mode(3)
motors.enable_motor()

# cmds = [140, 170]
#cmds = [100, 50]
#cmds = [150, -100]
cmds = [0,0] # 왼쪽 오른쪽?


class DataSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.enc_L = self.create_publisher(Float32,'/encoderL',10)
        self.enc_R = self.create_publisher(Float32,'/encoderR',10)
    

    def cmd_vel_callback(self, msg):
        if msg.linear.x>0:
            if msg.angular.z ==0:
                cmds[0] = float(msg.linear.x *10)
                cmds[1] = float(-(msg.linear.x *10))
            elif msg.angular.z >0:
                cmds[0] = float(msg.linear.x *5)
                cmds[1] = float(-(msg.linear.x *10))
            elif msg.angular.z <0:
                cmds[0] = float(msg.linear.x *10)
                cmds[1] = float(-(msg.linear.x *5))
        elif msg.linear.x<0:
            if msg.angular.z ==0:
                cmds[0] = float(-(msg.linear.x *10))
                cmds[1] = float(msg.linear.x *10)
            elif msg.angular.z >0:
                cmds[0] = float(-(msg.linear.x *5))
                cmds[1] = float(msg.linear.x *10)
            elif msg.angular.z <0:
                cmds[0] = float(-(msg.linear.x *10))
                cmds[1] = float(msg.linear.x *5)
        elif msg.linear.x==0:
            cmds[0]=0
            cmds[1]=0

        motors.set_rpm(cmds[0],cmds[1])
        # self.enc_L.publish(cmds[0])
        # self.enc_R.publish(-cmds[1])
        print(cmds[0])
        print(cmds[1])


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()
    motors.set_rpm(cmds[0],cmds[1])
    
    try:
        rclpy.spin(data_subscriber)
    except KeyboardInterrupt:
        data_subscriber.destroy_node()
        motors.disable_motor()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

