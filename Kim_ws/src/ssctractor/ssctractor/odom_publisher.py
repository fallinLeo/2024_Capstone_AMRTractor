#! /usr/bin/env python3

import numpy as np
import rclpy
import time
from math import cos, sin

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import Quaternion


#TF 발행하고 Odometry 계산해 Odom publish하는 코드
class TF_Publisher(Node):
  def __init__(self):
    super().__init__('tf_pub')
    # qos_profile = QoSProfile(depth=10)
    self.odom_pub = self.create_publisher(Odometry,'odom',10)
    self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
    #self.imu_yaw_sub = self.create_subscription(Imu, 'imu', self.imu_yaw_callback,10)

    self.encoderL_sub = self.create_subscription(Float32,'/encoderL',self.encoderL_callback,10)
    self.encoderR_sub = self.create_subscription(Float32,'/encoderR',self.encoderR_callback,10)
    
    self.imu_yaw = 0.0
    self.robot_pose_ = [0.0, 0.0, 0.0]
    self.robot_vel_ = [0.0, 0.0, 0.0] # x,y, 세타
    self.enc = [0.0, 0.0] #왼쪽 엔코더, 오른쪽 엔코더 라디안값
    self.use_imu_ = True

    self.wheels_radius_ = 0.1
    self.wheels_separation_ = 0.2

    self.last_theta = 0.0

    self.timer = self.create_timer(0.001, self.state_sub)
    self.last_time = [0,0,0]

    # #robot_state_publisher  주석처리
    # self.baseframe_broadcaster = StaticTransformBroadcaster(self)
    # self.laser_broadcaster = StaticTransformBroadcaster(self)
    # self.robot_fotbroadcaster = StaticTransformBroadcaster(self)

    self.broadcaster = TransformBroadcaster(self)
    #robot_state_publisher 여기까지 주석처리



    

  def state_sub(self):
    # self.robot_state_publisher()  #-> robot_state_publisher 추가함 URDF 수정하기 전까지
    ## Odom TF publish
    transform = TransformStamped()
    transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'robot_footprint'

    ## Odom topic publish
    self.odom = Odometry()
    self.odom.header.stamp = self.get_clock().now().to_msg()
    self.odom.header.frame_id = 'odom'
    self.odom.child_frame_id = 'robot_footprint'
    transform.transform.translation.x = self.robot_pose_[0]
    transform.transform.translation.y = self.robot_pose_[1]
    transform.transform.translation.z = 0.0
    ##
    q = self.euler_to_quaternion(0, 0, self.robot_pose_[2])
    transform.transform.rotation = q


    ## odometry Calculate 동기화
    current_time = time.time()
    duration = Duration(seconds=current_time - self.last_time[0])
    result = self.calculate_odometry(duration)
    self.last_time[0] = current_time
    if result: self.get_logger().info('Odometry calculation succeeded.')

    
    self.broadcaster.sendTransform(transform)
    self.odom_pub.publish(self.odom)
  
  def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return yaw_z
  
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
  
  def robot_state_publisher(self):  #TF transform of StaticformBroadcaster
      # base_link - >odom
    transform = TransformStamped()
    transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'robot_footprint'
    transform.transform.translation.x = self.robot_pose_[0]
    transform.transform.translation.y = self.robot_pose_[1]
    transform.transform.translation.z = 0.0
    ##
    q = self.euler_to_quaternion(0, 0, self.robot_pose_[2])
    transform.transform.rotation = q

    self.broadcaster.sendTransform(transform)

    # imu_link -> base_link
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'imu_link'
    t.transform.translation.x = 0.0 # 라이다 위치
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.04
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    t2 = TransformStamped()
    t2.header.stamp = self.get_clock().now().to_msg()
    t2.header.frame_id = 'base_link'
    t2.child_frame_id = 'laser'
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 1.0
    t2.transform.rotation.x = 0.0
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0

    t3 = TransformStamped()
    t3.header.stamp = self.get_clock().now().to_msg()
    t3.header.frame_id = 'robot_footprint'
    t3.child_frame_id = 'base_link'
    t3.transform.translation.x = 0.0
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 1.0
    t3.transform.rotation.x = 0.0
    t3.transform.rotation.y = 0.0
    t3.transform.rotation.z = 0.0
    t3.transform.rotation.w = 1.0

    self.baseframe_broadcaster.sendTransform(t)
    self.laser_broadcaster.sendTransform(t2)
    self.robot_fotbroadcaster.sendTransform(t3)


  def encoderL_callback(self,msg):
     self.enc[0] = msg.data
    #  current_time = time.time()
    #  duration = Duration(seconds=current_time - self.last_time[0])
    #  result = self.calculate_odometry(duration)
    #  self.last_time[0] = current_time
    #  if result: self.get_logger().info('Odometry calculation succeeded.')

  def encoderR_callback(self,msg):
     self.enc[1] = msg.data
    #  current_time = time.time()
    #  duration = Duration(seconds=current_time - self.last_time[1])
    #  result = self.calculate_odometry(duration)
    #  self.last_time[1] = current_time
    #  if result: self.get_logger().info('Odometry calculation succeeded.')
  
  def imu_callback(self, msg):
    quaternion = msg.orientation
    self.imu_yaw = self.euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    # header = msg.header
    # current_time = header.stamp.sec + header.stamp.nanosec * 1e-9
    # duration = Duration(seconds=current_time - self.last_time[2])
    # self.last_time[2] = current_time

    # result = self.calculate_odometry(duration)
    # if result: self.get_logger().info('Odometry calculation succeeded.')

  # def imu_yaw_callback(self,msg):
      
  
  def calculate_odometry(self, duration: Duration) -> bool:
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

        step_time = duration.nanoseconds / 1e9

        if step_time == 0.0:
            return False

        if wheel_l is None or wheel_l != wheel_l:
            wheel_l = 0.0

        if wheel_r is None or wheel_r != wheel_r:
            wheel_r = 0.0

        delta_s = self.wheels_radius_ * (wheel_r + wheel_l) / 2.0

        if self.use_imu_:
            theta = self.imu_yaw
            delta_theta = theta - self.last_theta
        else:
            theta = self.wheels_radius_ * (wheel_r - wheel_l) / self.wheels_separation_
            delta_theta = theta

        # compute odometric pose
        self.robot_pose_[0] += delta_s * cos(self.robot_pose_[2] + (delta_theta / 2.0))
        self.robot_pose_[1] += delta_s * sin(self.robot_pose_[2] + (delta_theta / 2.0))
        self.robot_pose_[2] += delta_theta


        self.get_logger().debug(f"x : {self.robot_pose_[0]}, y : {self.robot_pose_[1]}")

        # compute odometric instantaneous velocity
        v = delta_s / step_time
        w = delta_theta / step_time

        self.robot_vel_[0] += v
        self.robot_vel_[2] += w

        #odometry pose
        self.odom.pose.pose.position.x = self.robot_pose_[0]
        self.odom.pose.pose.position.y = self.robot_pose_[1]
        self.odom.twist.twist.linear.x = self.robot_vel_[0]
        self.odom.twist.twist.linear.z = self.robot_vel_[2]


        self.last_theta = theta
        return True

  
#   def timer_callback(self,event):
#      first_time=time.time()





#   def yaw_to_quaternion(heading):
#     '''
#     Converts yaw heading to quaternion coordinates.
#     '''
#     theta = 0.5 * heading
#     quaternion = Quaternion()
#     quaternion.z = np.sin(theta)
#     quaternion.w = np.cos(theta)

#     return quaternion

       

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
