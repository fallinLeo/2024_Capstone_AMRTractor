import rclpy
import numpy as np
from math import cos, sin, atan2
import math
import tf_transformations
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Twist


class Odom_class(Node):
    def __init__(self):
        super().__init__('leo_odometry')
        self.use_imu = True
        self.publish_tf = True

        self.wheels_separation = 0.507
        self.wheels_radius = 0.1

        self.imu_angle = 0.0
        self.last_imu_angle = 0.0
        self.last_theta = 0.0
        self.diff_joint_positions = np.array([0.0,0.0])
        self.last_joint_positions = np.zeros(2)
        self.robot_pose = np.array([0.0,0.0,0.0])
        self.robot_vel = np.array([0.0,0.0,0.0])

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.frame_id_of_odometry = 'odom'
        self.child_frame_id_of_odometry = 'robot_footprint'
        self.twist_vel2 = self.create_publisher(Twist, 'cmd_vel2',10)
        


        if self.use_imu:
            queue_size = 10
            slop = 0.1  # 이 값은 메시지 동기화의 대략적인 시간 차이를 설정합니다. 필요에 따라 조정해야 할 수 있습니다.

            self.joint_state_sub = Subscriber(self, JointState, 'joint_states')
            self.imu_sub = Subscriber(self, Imu, 'imu')

            self.sync = ApproximateTimeSynchronizer([self.joint_state_sub, self.imu_sub], queue_size, slop)
            self.sync.registerCallback(self.joint_state_and_imu_callback)
        else:
            self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
        self.subscription  # prevent unused variable warning



    def update_joint_state(self, joint_state: JointState,duration: Duration):
        step_time = duration.nanoseconds / 1e9
        enc_L = joint_state.velocity[0] * (2* math.pi/60) * step_time
        enc_R = joint_state.velocity[1] * (2* math.pi/60) * step_time
        

        self.diff_joint_positions[0] = enc_L
        self.diff_joint_positions[1] = enc_R      #두개의 단위 라디안
        # self.last_joint_positions[0] = joint_state.position[0]
        # self.last_joint_positions[1] = joint_state.position[1]
    
    def update_imu(self, imu_msg):
        self.imu_angle = atan2(
            imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
            0.5 - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z)
        print(f"imu_angle : {self.imu_angle}")
        # self.imu_angle = self.quaternion_to_euler(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w) #imu_yaw값
    
    def initial_pose_callback(self, msg):
      # 쿼터니언에서 오일러 각으로 변환
      orientation_q = msg.pose.pose.orientation
      orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

      # 로봇의 위치 업데이트
      self.robot_pose[0] = msg.pose.pose.position.x
      self.robot_pose[1] = msg.pose.pose.position.y
      # 로봇의 헤딩 방향 업데이트 및 yaw값 특정값으로 초기화
      self.robot_pose[2] = yaw
      self.imu_angle = yaw
      self.last_theta = yaw
        
    


    def joint_state_and_imu_callback(self, joint_state_msg, imu_msg):
        # self.get_logger().info(
        #     f"[joint_state_msg] nanosec : {joint_state_msg.header.stamp.nanosec} [imu_msg] nanosec : {imu_msg.header.stamp.nanosec}")

        # 현재 메시지의 시간을 저장
        current_time = joint_state_msg.header.stamp

        # last_time이 정의되어 있지 않다면, 현재 메시지의 시간으로 초기화
        if not hasattr(self, 'last_time'):
            self.last_time = current_time

        # duration 계산
        duration_sec = current_time.sec - self.last_time.sec
        duration_nanosec = current_time.nanosec - self.last_time.nanosec

        # 초(sec)와 나노초(nanosec)를 모두 고려한 duration 계산
        duration = Duration(seconds=duration_sec, nanoseconds=duration_nanosec)

        self.update_joint_state(joint_state_msg,duration)
        self.update_imu(imu_msg)
        self.calculate_odometry(duration)
        self.publish(joint_state_msg.header.stamp)

        # 메시지 처리 후에 last_time 업데이트
        self.last_time = current_time

        # self.get_logger().info('JointState and IMU messages synchronized.')


    def joint_state_callback(self, joint_state_msg):
        last_time = joint_state_msg.header.stamp
        duration = Duration(seconds=joint_state_msg.header.stamp.sec - last_time.sec,
                    nanoseconds=joint_state_msg.header.stamp.nanosec - last_time.nanosec)

        self.update_joint_state(joint_state_msg,duration)
        self.calculate_odometry(duration)
        self.publish(joint_state_msg.header.stamp)

        last_time = joint_state_msg.header.stamp
    
    def calculate_odometry(self, duration: Duration):
        wheel_l = self.diff_joint_positions[0]
        wheel_r = self.diff_joint_positions[1]

        delta_s = 0.0
        delta_theta = 0.0

        theta = 0.0
        

        step_time = duration.nanoseconds / 1e9  # 초 단위로 변환
        # print(f"step_time = {step_time:.9f}")


        if step_time == 0.0:
            return False

        if math.isnan(wheel_l):
            wheel_l = 0.0

        if math.isnan(wheel_r):
            wheel_r = 0.0

        delta_s = self.wheels_radius * (wheel_r + wheel_l) / 2.0   #R 세타, m * rad
        

        if self.use_imu:
            # theta = self.imu_angle
            # print(f"theta = {theta}")
            # delta_theta = self.calculate_angle_change(self.last_imu_angle,self.imu_angle)
            theta = self.imu_angle
            delta_theta = theta - self.last_theta
            # print(f"delta_theta = {delta_theta} 단위 : degree")
        else:
            theta = self.wheels_radius * (wheel_r - wheel_l) / self.wheels_separation
            delta_theta = theta

        self.robot_pose[0] += delta_s * math.cos(self.robot_pose[2] + (delta_theta / 2.0))
        self.robot_pose[1] += delta_s * math.sin(self.robot_pose[2] + (delta_theta / 2.0))
        self.robot_pose[2] += delta_theta
        # self.robot_pose[2] += round(self.degrees_to_radians(delta_theta),3) #소수점 3자리까지만
        print(f"robot_pose[2] = {self.robot_pose[2]}")

        # self.get_logger().info(f"x : {self.robot_pose[0]}, y : {self.robot_pose[1]}")

        v = delta_s / step_time #m * rad / second  -> m/s
        w = delta_theta / step_time

        self.robot_vel[0] = v
        self.robot_vel[1] = 0.0
        self.robot_vel[2] = w

        self.last_theta = theta
        # self.last_imu_angle = theta
        return True
    
    def publish(self, now):
        odom_msg = Odometry()

        odom_msg.header.frame_id = self.frame_id_of_odometry
        odom_msg.child_frame_id = self.child_frame_id_of_odometry
        odom_msg.header.stamp = now

        odom_msg.pose.pose.position.x = self.robot_pose[0]
        odom_msg.pose.pose.position.y = self.robot_pose[1]
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.robot_vel[0]
        odom_msg.twist.twist.angular.z = self.robot_vel[2]

        # Set the covariance values if needed
        cmd_vel = Twist()
        cmd_vel.linear.x = self.robot_vel[0]
        cmd_vel.angular.z = self.robot_vel[2]
        self.twist_vel2.publish(cmd_vel)

        odom_tf = TransformStamped()

        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation

        odom_tf.header.frame_id = self.frame_id_of_odometry
        odom_tf.child_frame_id = self.child_frame_id_of_odometry
        odom_tf.header.stamp = now

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(odom_tf)
    
    def quaternion_to_euler(self,x, y, z, w):
        # sin, cos 등의 계산을 위해 각 성분을 사용
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        pitch_y = np.arcsin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        
        return yaw_z
    def calculate_angle_change(self, old_angle, new_angle):
        delta_angle = new_angle - old_angle
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360
        return delta_angle
    def degrees_to_radians(self,degrees):
        radians = degrees * (math.pi / 180)
        return radians


def main(args=None):
    rclpy.init(args=args)
    node = Odom_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
