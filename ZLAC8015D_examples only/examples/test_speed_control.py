
from zlac8015d import ZLAC8015D
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32,Int16
from geometry_msgs.msg import Twist
import time
import numpy as np
from sensor_msgs.msg import JointState



class Node0(Node):
    def __init__(self):
        super().__init__('encoderL')
        qos_profile = QoSProfile(depth=10)
        self.encoderL_pub = self.create_publisher(Int16, '/encoderL', qos_profile)
        self.encoderR_pub = self.create_publisher(Int16,'/encoderR',qos_profile)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.create_time=self.create_timer(0.001,self.encoder_pub_message)
        #self.create_time=self.create_timer(0.001,self.encoderL_pub_message)
        self.motors = ZLAC8015D.Controller(port="/dev/ttyUSB1")  # 이제 Node0 내부에서 모터를 제어
        self.motors.set_accel_time(1000, 1000)
        self.motors.set_decel_time(1000, 1000)
        self.motors.set_mode(3)
        self.motors.enable_motor()
        self.cmds = [0, 0]

        # self.last_joint_positions = np.array([0,0]) # rpm값을 왼. 오 보낼거임.

    def encoder_pub_message(self):
        msg1 = Int16()
        msg2 = Int16()
        # msg1.data = int(self.cmds[0])
        # msg2.data = int(self.cmds[1]) # 엔코더 RIGHT값은 -로 퍼블리시뒤어 -값 추가함.
        tick_L, tick_R = self.motors.get_rpm()

        msg1.data = int(tick_L)
        msg2.data = int(-tick_R) # 엔코더 RIGHT값은 -로 퍼블리시뒤어 -값 추가함.
        print(f'encoder_L : {msg1.data}, encoder_R : {msg2.data}')
        # print(f'Error : {msg1.data - msg2.data}')
        self.encoderL_pub.publish(msg1)
        self.encoderR_pub.publish(msg2)
   
    def cmd_vel_callback(self, msg):
        # self.cmds[0] = int(((msg.linear.x - msg.angular.z)*0.285)/0.05)
        # self.cmds[1] = int(-((msg.linear.x + msg.angular.z)*0.285)/0.05)
        self.cmds[0] = int((msg.linear.x - 0.285 * msg.angular.z) / 0.1)
        self.cmds[1] = int(-(msg.linear.x + 0.285 * msg.angular.z) / 0.1)
        
        
        #pub_message()
        self.motors.set_rpm(self.cmds[0], self.cmds[1])  # RPM 설정

        # msg1 = Int16()
        # msg2 = Int16()
        # msg1.data = int(tick_L)
        # msg2.data = int(-tick_R) # 엔코더 RIGHT값은 -로 퍼블리시뒤어 -값 추가함.
        # print(msg1.data)
        # self.encoderL_pub.publish(msg1)
        # self.encoderR_pub.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node0 = Node0()
    try:
        rclpy.spin(node0)
        
    except KeyboardInterrupt:
        node0.get_logger().info('Keyboard Interrupt (SIGINT)')
        node0.motors.set_rpm(0,0)
        zero_msg = Int16()
        zero_msg.data = 0  # data 필드에 0.0 할당
        # encoderL_pub과 encoderR_pub에 zero_msg를 publish
        node0.encoderL_pub.publish(zero_msg)
        node0.encoderR_pub.publish(zero_msg)
        
    finally:
        node0.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
