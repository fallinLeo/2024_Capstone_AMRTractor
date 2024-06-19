#! /usr/bin/env python3

import numpy as np
import rclpy
import time
from math import cos, sin
import math
from rclpy.duration import Duration
from rclpy.node import Node
import tf_transformations
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped, TransformStamped, QuaternionStamped, Quaternion
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

import serial #Imu Yaw값만



#TF 발행하고 Odometry 계산해 Odom publish하는 코드
class TF_Publisher(Node):
  def __init__(self):
    super().__init__('tf_pub')
    # qos_profile = QoSProfile(depth=10)
    self.odom_pub = self.create_publisher(Odometry,'/odom',10)
    # self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
    self.py_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=0.03)

    # self.timer = self.create_timer(0.001, self.read_serial_data)

    self.marker_pub = self.create_publisher(Marker,'robot_pose_visul',10)
    self.encoderL_sub = self.create_subscription(Int16,'/encoderL',self.encoderL_callback,10)
    self.encoderR_sub = self.create_subscription(Int16,'/encoderR',self.encoderR_callback,10)
    
    self.current_imu_yaw = 0.0
    self.last_imu_yaw = 0.0
    self.robot_pose_ = [0.0, 0.0, 0.0] #x,y, heading
    self.robot_vel_ = [0.0, 0.0, 0.0] # x,y, 세타
    self.enc = [0.0, 0.0] #왼쪽 엔코더, 오른쪽 엔코더 라디안값
    self.use_imu_ = True

    self.wheels_radius_ = 0.1
    self.wheels_separation_ = 0.507


    self.last_time = [0,0,0,0] # x값, y값, imu값, odom_cal값 초기화

    self.last_time[0] = time.time() #odom
    self.last_time[1] = time.time() #encoderL
    self.last_time[2] = time.time() #encoderR
    self.last_time[3] = time.time() 

    #low-pass filter변수들
    self.filtered_imu_yaw = 0.0
    self.alpha = 0.45
    self.imu_pub = self.create_publisher(Float32,'/imu_yaw',10) #그래프 계산용 토픽 퍼블리시
    self.imu_filter_pub = self.create_publisher(Float32,'imu_yaw_filtered',10)

    self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
    self.subscription  # prevent unused variable warning

    self.broadcaster = TransformBroadcaster(self)
    #robot_state_publisher 여기까지 주석처리

  def read_serial_data(self):
      if self.py_serial.in_waiting > 0:
          line = self.py_serial.readline().decode('utf-8').strip()
          
          # 공백으로 구분된 데이터를 리스트로 변환
          data_list = line.split()
          print(len(data_list))          
          try:
              # 리스트의 마지막 요소를 float으로 변환하여 저장
              self.roll, self.pitch, self.current_imu_yaw = map(float, data_list[1:4])
              self.acc_x, self.acc_y, self.acc_z = map(float, data_list[4:7])
              self.gyro_x, self.gyro_y, self.gyro_z = map(float, data_list[7:10])
              
              # print(f"Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}")
              # print(f"ACC - X: {self.acc_x}, Y: {self.acc_y}, Z: {self.acc_z}")
              # print(f"GYRO - X: {self.gyro_x}, Y: {self.gyro_y}, Z: {self.gyro_z}")
            

          except ValueError as e:
              self.get_logger().error(f'Error converting data to float: {e}')
  
  def initial_pose_callback(self, msg):
      # 쿼터니언에서 오일러 각으로 변환
      orientation_q = msg.pose.pose.orientation
      orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)


      # 로봇의 헤딩 방향 업데이트
      self.robot_pose_[2] = yaw
      self.current_imu_yaw = yaw
      self.last_imu_yaw = yaw

      # self.get_logger().info(f'Updated heading: {self.robot_pose_[2]} rad')

        
  
  # def euler_from_quaternion(self, x, y, z, w):
  #       t0 = +2.0 * (w * x + y * z)
  #       t1 = +1.0 - 2.0 * (x * x + y * y)
  #       roll_x = np.arctan2(t0, t1)

  #       t2 = +2.0 * (w * y - z * x)
  #       t2 = +1.0 if t2 > +1.0 else t2
  #       t2 = -1.0 if t2 < -1.0 else t2
  #       pitch_y = np.arcsin(t2)

  #       t3 = +2.0 * (w * z + x * y)
  #       t4 = +1.0 - 2.0 * (y * y + z * z)
  #       yaw_z = np.arctan2(t3, t4)

  #       return yaw_z
  
  def euler_to_quaternion(self, roll, pitch, yaw):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q
  
  def rpm_to_rad_per_sec(self,rpm):
    return (2 * math.pi * rpm) / 60
  def degrees_to_radians(self,degrees):
    radians = degrees * (math.pi / 180)
    return radians
  
  def calculate_angle_change(self, old_angle, new_angle):
    delta_angle = new_angle - old_angle
    if delta_angle > 180:
        delta_angle -= 360
    elif delta_angle < -180:
        delta_angle += 360
    return delta_angle
  
 
  def encoderL_callback(self,msg):
     self.enc[0] = msg.data * (2 * math.pi / 60) #rad/s
     current_time = time.time()
     duration = current_time - self.last_time[1]
     self.enc[0] = self.enc[0] * duration   # rad/s * s = rad
     self.calculate_odometry(duration) # 0.01초보다 작은 간격에서만 odometry 계산
     self.last_time[1] = current_time
     
     

  def encoderR_callback(self,msg):
     self.enc[1] = msg.data * (2 * math.pi / 60)
     current_time = time.time()
     duration = current_time - self.last_time[2] # 라디안 단위
     self.enc[1] = self.enc[1] * duration #* 0.1 지워놈
     self.calculate_odometry(duration)
     self.last_time[2] = current_time
  
  # def imu_callback(self, msg):
  #   quaternion = msg.orientation
  #   self.imu_yaw = self.euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
  #   header = msg.header
  #   current_time = header.stamp.sec + header.stamp.nanosec * 1e-9
  #   duration = Duration(seconds=current_time - self.last_time[2])
  #   self.last_time[2] = current_time

  #   result = self.calculate_odometry(duration)
  #   if result: self.get_logger().info('Odometry calculation succeeded.')

  # def imu_yaw_callback(self,msg):
  


  def marker_visualize(self):
      marker = Marker()
      marker.header.frame_id = "odom"
      marker.header.stamp = self.get_clock().now().to_msg()
      marker.ns = "robot_direction"
      marker.id = 0
      marker.type = Marker.ARROW
      marker.action = Marker.ADD
      
      # 화살표의 시작점 설정 (로봇의 현재 위치)
      start_point = Point()
      start_point.x = self.robot_pose_[0]
      start_point.y = self.robot_pose_[1]
      start_point.z = 0.0  # 2D 환경 가정
      
      # 화살표의 끝점 계산 (로봇의 방향을 고려)
      end_point = Point()
      end_point.x = start_point.x + math.cos(self.robot_pose_[2]) * 1.0  # 1.0은 화살표 길이
      end_point.y = start_point.y + math.sin(self.robot_pose_[2]) * 1.0
      end_point.z = 0.0
      
      marker.points.append(start_point)
      marker.points.append(end_point)
      
      # 화살표의 시각적 속성 설정
      marker.scale.x = 0.5 # 화살표의 굵기
      marker.scale.y = 0.4  # 화살표 끝의 너비
      marker.scale.z = 0.4  # 화살표 끝의 높이
      marker.color.a = 1.0  # 알파 값
      marker.color.r = 1.0  # 빨간색
      marker.color.g = 0.0
      marker.color.b = 0.0
      
      # 화살표 퍼블리시
      self.marker_pub.publish(marker)
      

      
  
  def calculate_odometry(self, duration: Duration) -> bool:
        self.read_serial_data()
        # rotation value of wheel [rad]
        wheel_l = self.enc[0]
        wheel_r = self.enc[1]

        delta_s = 0.0
        delta_theta = 0.0

        theta = 0.0

        # v = translational velocity [m/s]
        # w = rotational velocity [rad/s]
        v = 0.0
        w = 0.0

        # step_time = duration.nanoseconds / 1e9
        step_time = duration #마지막 계산된 시간부터 지금까지의 시간간격 ms 단위지금


        if step_time == 0.0:
            return False

        if wheel_l is None or wheel_l != wheel_l:
            wheel_l = 0.0

        if wheel_r is None or wheel_r != wheel_r:
            wheel_r = 0.0

        delta_s = self.wheels_radius_ * (wheel_r + wheel_l) / 2.0
        # print(f"delta s : {delta_s}")

        if self.use_imu_:
            theta = self.current_imu_yaw #degree단위
            delta_theta = self.calculate_angle_change(self.last_imu_yaw,self.current_imu_yaw) #degree단위

            # theta = self.wheels_radius_ * (wheel_r - wheel_l) / self.wheels_separation_  # IMU 실제 사용하면 여기 두 줄 다 없애기!
            # delta_theta = theta

            
        else:
            theta = self.wheels_radius_ * (wheel_r - wheel_l) / self.wheels_separation_
            delta_theta = theta

        # compute odometric pose delta_s = wheel_radius*(0L+0R)/2
        self.robot_pose_[0] += delta_s * cos(self.robot_pose_[2] + (delta_theta / 2.0))
        self.robot_pose_[1] += delta_s * sin(self.robot_pose_[2] + (delta_theta / 2.0))
        self.robot_pose_[2] += round(self.degrees_to_radians(delta_theta),3) #소수점 3자리까지만
        
        print(f"받아온 imu yaw값 변화량(rad) : {self.robot_pose_[2]}")
        

        # compute odometric instantaneous velocity
        v = delta_s / step_time
        w = delta_theta / step_time

        self.robot_vel_[0] = v
        self.robot_vel_[2] = w
        print(f"delta_theta : {delta_theta}, w : {self.robot_vel_[2]}")

        ## Odom TF publish
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'robot_footprint'

        ## Odom topic publish
        self.odom = Odometry()
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'robot_footprint'



        

        # #odometry pose
        self.odom.pose.pose.position.x = self.robot_pose_[0]
        self.odom.pose.pose.position.y = self.robot_pose_[1]
        self.odom.twist.twist.linear.x = self.robot_vel_[0]
        self.odom.twist.twist.angular.z = self.robot_vel_[2]


        # self.transform.transform.translation.z = self.odom.pose.pose.position.z
        self.transform.transform.translation.x = self.odom.pose.pose.position.x
        self.transform.transform.translation.y = self.odom.pose.pose.position.y
        ##
        q = self.euler_to_quaternion(0, 0, self.robot_pose_[2])
        self.transform.transform.rotation = q
        
        self.marker_visualize() #마커 시각화

        self.broadcaster.sendTransform(self.transform)
        self.odom_pub.publish(self.odom)


        self.last_imu_yaw = theta
        return True
       

def main(args=None):
  rclpy.init(args=args)
  node = TF_Publisher()



  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
	main()