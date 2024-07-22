
from zlac8015d import ZLAC8015D
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32,Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import serial
import math



class Node0(Node):
    def __init__(self):
        super().__init__('joint_pub_controller')
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.timer = self.create_timer(0.01, self.publish_sensor_state)
        self.joint_state_pub = self.create_publisher(JointState,'joint_states',10)
        # self.last_yaw = 0.0
        #self.create_time=self.create_timer(0.001,self.encoderL_pub_message)
        self.motors = ZLAC8015D.Controller(port="/dev/ttyUSB1")  # 이제 Node0 내부에서 모터를 제어
        self.motors.set_accel_time(1000, 1000)
        self.motors.set_decel_time(1000, 1000)
        self.motors.set_mode(3)
        self.motors.enable_motor()
        self.cmds = [0, 0]

        self.twist_vel_call = self.create_publisher(Twist,'cmd_vel_call',10)
        #시간 출력토픽
        # self.current_top = self.create_publisher(Float32,'cmd_vel_sub_time',10)
        # self.last_joint_positions = np.array([0,0]) # rpm값을 왼. 오 보낼거임.

    def get_encoder_ticks(self):
        tick_L, tick_R = self.motors.get_rpm()
        return tick_L,tick_R
    def degrees_to_radians(self,degrees):
        radians = degrees * (math.pi / 180)
        return radians
    
    def cmd_vel_callback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.linear.x
        cmd_vel.angular.z = msg.angular.z
        self.twist_vel_call.publish(cmd_vel)


        l_rad_s = (msg.linear.x - 0.285 * msg.angular.z) / 0.1
        r_rad_s = -(msg.linear.x + 0.285 * msg.angular.z) / 0.1
        #원래 들어가는 단위가 rad/s였어서, rpm으로 바꾸기 위해 60/2pi 곱함.
        self.cmds[0] = round(l_rad_s * 60/(2 * math.pi))
        self.cmds[1] =  round(r_rad_s * 60/(2 * math.pi))

        # self.cmds[0] = int((msg.linear.x - 0.2535 * msg.angular.z) / 0.1) *10
        # self.cmds[1] = int(-(msg.linear.x + 0.2535 * msg.angular.z) / 0.1) *10
        
        #pub_message()
        self.motors.set_rpm(self.cmds[0], self.cmds[1])  # RPM 설정
        
    
    def publish_sensor_state(self):
        tick_L, tick_R = self.get_encoder_ticks()

        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']  # 조인트 이름 설정
        joint_state_msg.position = [0.0, 0.0]  # 엔코더의 경우 위치 정보가 필요없을 수 있음
        joint_state_msg.velocity = [tick_L, -tick_R]  # RPM으로 가정, 필요에 따라 단위 변환 필요
        joint_state_msg.effort = []  # 토크 정보가 필요 없는 경우 비워둠

        self.joint_state_pub.publish(joint_state_msg)
        # self.get_logger().info(f'Published JointState: {joint_state_msg}')
    

def main(args=None):
    rclpy.init(args=args)
    node0 = Node0()
    try:
        rclpy.spin(node0)
        
    except KeyboardInterrupt:
        node0.get_logger().info('Keyboard Interrupt (SIGINT)')
        node0.motors.set_rpm(0,0)
        node0.publish_sensor_state()


        
    finally:
        node0.destroy_node()
        node0.motors.set_rpm(0,0)
        node0.publish_sensor_state()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
