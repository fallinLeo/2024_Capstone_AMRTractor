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
        super().__init__('enc_subscriber')

        self.enc_L = self.create_subscription(Float32, '/encoderL', self.enc_l_callback, 10)
        self.enc_R = self.create_subscription(Float32,'/encoderR',self.enc_r_callback,10)

    

        


    def enc_l_callback(self,msg):
        cmds[0] = int(msg.data * 5000)
        motors.set_rpm(cmds[0],cmds[1])
    
    def enc_r_callback(self,msg):
        cmds[1] = int(-msg.data * 5000)
        motors.set_rpm(cmds[0],cmds[1])        


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()
    
    try:
        rclpy.spin(data_subscriber)
    except KeyboardInterrupt:
        data_subscriber.destroy_node()
        motors.disable_motor()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

