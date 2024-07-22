
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32,Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
import time
import numpy as np
import serial
import math



class Node0(Node):
    def __init__(self):
        super().__init__('imu_pub')
        qos_profile = QoSProfile(depth=10)
        self.timer = self.create_timer(0.01, self.publish_sensor_state)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.py_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=0.03)  #OPENCR IMU 사용
        # self.last_yaw = 0.0
        #self.create_time=self.create_timer(0.001,self.encoderL_pub_message)
        
    
    def degrees_to_radians(self,degrees):
        radians = degrees * (math.pi / 180)
        return radians
    
    def publish_sensor_state(self):
        self.read_serial_data()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        x, y, z, w = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w

        # 가속도 데이터 설정 (g -> m/s² 변환)
        imu_msg.linear_acceleration.x = self.acc_x
        imu_msg.linear_acceleration.y = self.acc_y
        imu_msg.linear_acceleration.z = self.acc_z

        # # 각속도 데이터 설정 (도/초 -> rad/s 변환)
        imu_msg.angular_velocity.x = self.gyro_x  # 도/초를 rad/s로 변환
        imu_msg.angular_velocity.y = self.gyro_y
        imu_msg.angular_velocity.z = self.gyro_z
        self.imu_publisher_.publish(imu_msg)
        # self.get_logger().info(f'Published JointState: {joint_state_msg}')
    
    def read_serial_data(self):
      if self.py_serial.in_waiting > 0:
          line = self.py_serial.readline().decode('utf-8').strip()
          
          # 공백으로 구분된 데이터를 리스트로 변환
          data_list = line.split()
          print(len(data_list))
          try:
              # 리스트의 마지막 요소를 float으로 변환하여 저장
              self.roll, self.pitch, self.yaw = map(float, data_list[1:4])
              self.acc_x, self.acc_y, self.acc_z = map(float, data_list[4:7])
              self.gyro_x, self.gyro_y, self.gyro_z = map(float, data_list[7:10])
            #   yaw = self.calculate_angle_change(self.last_yaw,self.yaw)


              self.roll = self.degrees_to_radians(self.roll)
              self.pitch = self.degrees_to_radians(self.pitch)
              self.yaw = self.degrees_to_radians(self.yaw)

              print(f"Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}, 단위 rad")
              print(f"ACC - X: {self.acc_x}, Y: {self.acc_y}, Z: {self.acc_z}")
              print(f"GYRO - X: {self.gyro_x}, Y: {self.gyro_y}, Z: {self.gyro_z}")

            #   self.last_yaw = self.yaw
          except ValueError as e:
              self.get_logger().error(f'Error converting data to float: {e}')

    def calculate_angle_change(self, old_angle, new_angle):
        delta_angle = new_angle - old_angle
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360
        return delta_angle


def main(args=None):
    rclpy.init(args=args)
    node0 = Node0()
    try:
        rclpy.spin(node0)
        
    except KeyboardInterrupt:
        node0.get_logger().info('Keyboard Interrupt (SIGINT)')
        node0.publish_sensor_state()
        
    finally:
        node0.destroy_node()
        node0.publish_sensor_state()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
