import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32
import math
from geometry_msgs.msg import PoseStamped
import tf_transformations


class PathFollower(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel3', 10)
        self.start_pub = self.create_publisher(Int32, 'start_arduino', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.callback, 10) #카메라 아르코마커
        self.distance_to_goal = 0.0
        self.path_created = False  # 경로 생성 여부를 저장하는 플래
        self.waypoints = []
        self.goal_x = None
        self.goal_y = None

        self.now_pose_pub = self.create_publisher(PoseStamped, 'now_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        #현재 위치 저장 변수
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.previous_odom = None

        # PD 제어기에서 사용할 변수 초기화
        self.prev_error = 0.0
        self.Kp = 0.1  # 비례 게인
        self.Kd = 0.08  # 미분 게인

    def odom_callback(self, msg):
        # 이전 odom 값이 있는 경우에만 차이를 계산
        if self.previous_odom is not None:
            # 이전 위치와 현재 위치의 차이 계산
            delta_x = msg.pose.pose.position.x - self.previous_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.previous_odom.pose.pose.position.y
            
            # 계산된 차이를 current_pose에 반영
            self.current_pose.pose.position.x += delta_x
            self.current_pose.pose.position.y += delta_y

            # 쿼터니언을 오일러 각도로 변환하여 yaw 값 추출
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )

            # 헤더의 frame_id를 base_link로 설정
            self.current_pose.header.frame_id = "base_link"

            # 헤딩 값과 위치를 로깅
            # self.get_logger().info(f'Current Position: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, yaw={yaw}')

            # now_pose 퍼블리시
            self.now_pose_pub.publish(self.current_pose)

        # 현재 odom 메시지를 이전 odom으로 저장
        self.previous_odom = msg


 

    def callback(self, goal_pose):

        
        # 차량의 초기 위치 및 목표 위치
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        self.goal_x = goal_pose.pose.position.x
        self.goal_y = goal_pose.pose.position.z

        if not self.path_created:
            self.waypoints = self.generate_path(start_x,start_y,self.goal_x,self.goal_y)
            self.path_created = True #경로 생성되면


        # PD 제어를 이용해 경로를 따라가는 로직
        if self.path_created :
            self.follow_path(self.waypoints)
    
    def generate_path(self, start_x, start_y, goal_x, goal_y):
        """3차 다항식 경로를 생성하는 함수"""
        x_points = np.linspace(start_x, goal_x, num=10)
        coefficients = np.polyfit([start_x, goal_x], [start_y, goal_y], 3)
        y_points = np.polyval(coefficients, x_points)

        # Path 메시지로 변환 후 퍼블리시
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        waypoints = []
        for i in range(len(x_points)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = -y_points[i]  # x축을 반대로 설정
            pose.pose.position.y = x_points[i]  # y축을 그대로 유지
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
            waypoints.append(pose.pose.position)  # 경로 저장

        # 경로 퍼블리시
        self.path_pub.publish(path_msg)

        return waypoints

    def follow_path(self, waypoints):
        """
        Use PD control to follow the planned path.
        """
        for waypoint in waypoints:
            cmd = Twist()
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y

            # 목표까지의 거리 계산
            self.distance_to_goal = math.sqrt((self.goal_x - current_x) ** 2 + (self.goal_y - current_y) ** 2)
            self.get_logger().info(f'distance_to_goal : {self.distance_to_goal}')

            # PD 제어 적용
            error_x = waypoint.x - current_x
            error_y = waypoint.y - current_y
            goal_heading = math.atan2(error_y, error_x) # 목표 방향
            self.get_logger().info(f'goal_heading : {goal_heading}')
            self.get_logger().info(f'error_X : {error_x}, error_y = {error_y}')

            # 현재 차량의 헤딩 추출
            orientation_q = self.current_pose.pose.orientation
            _, _, current_heading = tf_transformations.euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.get_logger().info(f'current_heading :{current_heading}')

            # Heading error (difference between current heading and goal heading)
            error_heading = goal_heading - (current_heading - math.pi)
            d_error = error_heading - self.prev_error
            angular_output = self.Kp * error_heading + self.Kd * d_error
            self.get_logger().info(f'error_heading : {error_heading} d_error :{d_error}')

            # 각속도의 최대값을 0.15로 제한
            max_angular_speed = 0.15
            angular_output = max(-max_angular_speed, min(max_angular_speed, angular_output))

            # 이전 오차 저장
            self.prev_error = error_heading


            if self.distance_to_goal < 0.2:  # 목표 지점에 거의 도달하면 멈춤
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)
                start_msg = Int32()
                start_msg.data = 1
                self.start_pub.publish(start_msg)
                self.get_logger().info('Arrived at goal')
                break
            else:
                # 속도 명령 생성
                cmd.linear.x = -0.15  # 일정한 선속도
                cmd.angular.z = angular_output  # PD 제어기 각속도 출력
                self.get_logger().info(f'angular_output : {angular_output}')
                self.vel_pub.publish(cmd)

    
    


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
