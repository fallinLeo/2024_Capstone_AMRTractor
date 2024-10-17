import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
import tf_transformations
import time


class PathFollower(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel2', self.velocity_callback, 10)  # 속도 토픽 구독
        # self.start_pub = self.create_publisher(Int32, 'start_arduino', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.callback, 10) # 카메라 아르코마커
        self.current_velocity = Twist()  # 현재 속도를 저장할 변수
        self.distance_to_goal = 0.0
        self.path_created = False  # 경로 생성 여부를 저장하는 플래그
        self.waypoints = []
        self.goal_x = None
        self.goal_y = None
        self.current_index = 2
        self.previous_odom = None
        self.first_odom = True  # 처음 odom 호출을 체크하기 위한 플래그
        self.initial_yaw = 0.0  # 처음 yaw 값을 저장할 변수
        self.yaw = 0.0
        self.lookahead_distance = 0.04

        self.closest_pub = self.create_publisher(PointStamped, 'closest_waypoint', 10)
        self.ld_marker = self.create_publisher(PointStamped,'ld_point',10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.now_pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.docking_flag = self.create_subscription(Bool,'docking_start',self.docking_flag_callback,10)
        self.doc_flag = False

        self.pub_to_ard = self.create_publisher(Int32,'gripper_connect',10)

        # PD 제어기에서 사용할 변수 초기화
        self.prev_error = 0.0
        self.Kp = 1.2  # 비례 게인
        self.Kd = 0.8 # 미분 게인


        # Base link frame - starting from (0,0)
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
    
    def odom_callback(self, msg):
        # if self.doc_flag ==False : return
        
        # odom 좌표계의 orientation을 이용해 yaw 계산
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        if self.first_odom:
            # 처음 yaw 값을 기준으로 설정
            self.initial_yaw = self.yaw
            self.first_odom = False

        # 초기 yaw 값을 빼서 기준 yaw 값을 0으로 설정
        self.yaw -= self.initial_yaw


    def velocity_callback(self, msg):
        # if self.doc_flag ==False : return #도킹시작전은 아무동작 x
        self.current_velocity = msg
    
    def docking_flag_callback(self,msg):
        self.doc_flag = msg.data


    def timer_callback(self):
        # if self.doc_flag ==False : return #도킹시작전은 아무동작 x


        # current_pose 계산: self.yaw와 self.current_velocity를 사용해 업데이트
        delta_time = 0.01  # 타이머 콜백 주기 (0.1초)
        v = self.current_velocity.linear.x  # cmd_vel2에서 가져온 선속도

        # base_link 좌표계에서 delta_x, delta_y 계산
        delta_x_base_link = v * delta_time * math.cos(self.yaw)
        delta_y_base_link = v * delta_time * math.sin(self.yaw)

        # current_pose 업데이트
        self.current_pose.pose.position.x += delta_x_base_link
        self.current_pose.pose.position.y += delta_y_base_link

        # 헤딩 방향 (yaw 값) 정보를 쿼터니언으로 변환
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        self.current_pose.pose.orientation.x = q[0]
        self.current_pose.pose.orientation.y = q[1]
        self.current_pose.pose.orientation.z = q[2]
        self.current_pose.pose.orientation.w = q[3]

        # current_pose 퍼블리시
        self.now_pose_pub.publish(self.current_pose)

        if self.path_created:
            self.follow_path(self.waypoints)
 

    def callback(self, goal_pose):
        # if self.doc_flag ==False : return
        
        # 목표 위치 설정
        self.goal_x = goal_pose.pose.position.x
        self.goal_y = goal_pose.pose.position.z


        # 목표 지점까지의 거리 계산
        self.distance_to_goal = math.sqrt((self.goal_x) ** 2 + (self.goal_y) ** 2)

        if not self.path_created:
            self.waypoints = self.generate_path(self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.goal_x, self.goal_y)
            self.path_created = True  # 경로 생성되면
    
    def generate_path(self, start_x, start_y, goal_x, goal_y):
        """y축에서 시작하는 경로 생성"""
        
        # y 축을 중심으로 시작하기 위해 x축은 작게 설정하고 y축을 주로 이동하도록 경로 생성
        y_points = np.linspace(start_y, goal_y, num=10)

        # y축 기준으로 경로 생성
        coefficients = np.polyfit([start_y, goal_y], [start_x, goal_x], 6)  # 3차 다항식으로 경로 설계
        x_points = np.polyval(coefficients, y_points)



        # Path 메시지로 변환 후 퍼블리시
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        waypoints = []
        for i in range(len(x_points)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = -y_points[i]  # x축 값 설정
            pose.pose.position.y = x_points[i] # y축을 기준으로 경로 설정
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
            waypoints.append(pose.pose.position)  # 경로 저장

        # 경로 퍼블리시
        self.path_pub.publish(path_msg)

        return waypoints



    def follow_path(self, waypoints):
        """
        Use PD control to follow the planned path with lookahead distance.
        """
        

        if self.current_index >= len(waypoints):
            return

        cmd = Twist()

        # Base link 기준 (0, 0)에서 시작하므로 현재 위치는 항상 (0, 0)
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        #L_d점 퍼블리시
        lookahead_marker = PointStamped()
        lookahead_marker.header.frame_id = "base_link"  # 좌표계 설정
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 설정
        lookahead_marker.point.x = current_x - self.lookahead_distance * math.cos(self.yaw)  # 퍼블리시할 위치 정보
        lookahead_marker.point.y = current_y - self.lookahead_distance * math.sin(self.yaw)
        lookahead_marker.point.z = 0.0  # z 좌표는 0으로 설정

        goal_to_current = math.sqrt((current_x -self.goal_x) ** 2 + 
                                            (current_y - self.goal_y) ** 2)
        if self.distance_to_goal < 0.2 or goal_to_current < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)
            
            time.sleep(1)
            grip_start = Int32()
            grip_start.data = 1
            self.pub_to_ard.publish(grip_start)
            self.get_logger().info('Arrived at goal, stopping...')
            self.doc_flag = False
            self.path_created =False #이 노드 실행 x되게
            return
        

        # 가장 가까운 waypoints를 찾는 로직
        for i in range(self.current_index, len(waypoints)):
            waypoint_to_current = math.sqrt((waypoints[i].x - lookahead_marker.point.x) ** 2 + 
                                            (waypoints[i].y - lookahead_marker.point.y) ** 2)
            # self.get_logger().info(f'current_idx_x: {waypoints[i].x} & lookahead_pt_x :{lookahead_marker.point.x}')
            # Lookahead point보다 뒤에 있지 않도록 보정
            if waypoint_to_current < self.lookahead_distance or waypoints[i].x > lookahead_marker.point.x:
                # 인덱스가 범위를 넘지 않도록 체크
                if i + 1 < len(waypoints):
                    self.current_index = i + 1
                else:
                    self.current_index = i  # 마지막 인덱스에 도달하면 그 인덱스로 유지
                break

        closest_waypoint_aft = waypoints[self.current_index]
        self.get_logger().info(f'current_index: {self.current_index}')
        self.get_logger().info(f'goal_to_current: {goal_to_current}')
        

    
        # closest_waypoint_aft를 퍼블리시
        closest_marker = PointStamped()
        closest_marker.header.frame_id = "base_link"  # 좌표계 설정
        closest_marker.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 설정
        closest_marker.point.x = closest_waypoint_aft.x  # 퍼블리시할 위치 정보
        closest_marker.point.y = closest_waypoint_aft.y
        closest_marker.point.z = 0.0  # z 좌표는 0으로 설정


        self.closest_pub.publish(closest_marker)  # 퍼블리시 실행
        self.ld_marker.publish(lookahead_marker)

        
        # closest와 next waypoint를 퍼블리시

        # PD 제어 적용
        error_x = closest_waypoint_aft.x - current_x
        error_y = closest_waypoint_aft.y - current_y
        goal_heading = math.atan2(error_y, error_x) + math.pi
        goal_heading = self.normalize_angle(goal_heading)  # 각도를 정규화

        # 현재 차량의 헤딩 추출
        current_heading = self.yaw

        error_heading = self.normalize_angle(goal_heading - current_heading)
        

        d_error = error_heading - self.prev_error
        angular_output = self.Kp * error_heading + self.Kd * d_error

        # 각속도의 최대값을 제한
        max_angular_speed = 0.07
        angular_output = max(-max_angular_speed, min(max_angular_speed, angular_output))
        # if math.sqrt((waypoints[-1].x - lookahead_marker.point.x) ** 2 + 
        #                                     (waypoints[-1].y - lookahead_marker.point.y) ** 2) < 0.3 :
        #     angular_output = -math.atan2(self.goal_x,self.goal_y)
        #     self.get_logger().info(f'angular_output: {angular_output}')


        # 이전 오차 저장
        self.prev_error = error_heading

        # 속도 명령 생성
        cmd.linear.x = -0.1  # 일정한 선속도
        cmd.angular.z = angular_output  # PD 제어기 각속도 출력

        self.vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        """
        주어진 각도를 -π ~ π 범위로 정상화하는 함수.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
