import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import transforms3d
import tf2_ros
import math

set_id = 3
angle = math.radians(90)  # 90도를 라디안으로 변환
qx_rot = 0
qy_rot = math.sin(angle/2)
qz_rot = 0
qw_rot = math.cos(angle/2)

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher_ = self.create_publisher(Image, 'aruco_image', 10)
        self.pose_publisher = self.create_publisher(PoseStamped,'pose',10)
        self.theta_publisher = self.create_publisher(Float32,'aruco_theta',10)
        self.bridge = CvBridge()

        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # 스트리밍 시작
        self.pipeline.start(config)

        # 카메라 내부 파라미터 얻기
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.cmtx = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs)

        # ArUco 마커 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # 마커 사이즈 (mm 단위)
        self.marker_size = 65
        self.marker_3d_edges = np.array([
            [-self.marker_size / 2, -self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [-self.marker_size / 2, self.marker_size / 2, 0]
        ], dtype='float32')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # 이미지를 numpy 배열로 변환
        img = np.asanyarray(color_frame.get_data())

        # ArUco 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.parameters)

       
        if ids is not None:
                for i, marker_id in enumerate(ids):
                    # 특정 ID (예: 1)인 마커만 처리
                    if marker_id[0] == set_id:  # ID가 1인 마커만 처리
                        corner = corners[i].reshape((4, 2))
                        topLeft, topRight, bottomRight, bottomLeft = corner

                        # PnP 문제 해결
                        ret, rvec, tvec = cv2.solvePnP(self.marker_3d_edges, corner, self.cmtx, self.dist)
                        if ret:
                            Ry_90 = np.array([[0, 0, 1],   
                                            [0, 1, 0],
                                            [-1, 0, 0]])
                            Rx_90 = np.array([[1, 0, 0],   
                                      [0, 0, 1],
                                      [0, -1, 0]])
                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            rotation_matrix = Rx_90 @ (Ry_90 @ rotation_matrix)
                            rvec, _ = cv2.Rodrigues(rotation_matrix)
                            
                            tvec = Rx_90 @ (Ry_90 @ tvec)
                            
                            theta = math.atan2(rotation_matrix[1,0], rotation_matrix[0,0])
                            theta_msg = Float32()
                            theta_msg.data = theta
                            self.theta_publisher.publish(theta_msg)


                            quat = transforms3d.quaternions.mat2quat(rotation_matrix)
                            tvec[2][0] =0
                            qx, qy,qz,qw = quat[0], quat[1], quat[2], quat[3]
                            pose_msg = PoseStamped()  # PoseStamped로 변경하여 헤더 포함
                            pose_msg.header.frame_id = 'camera'  # 헤더 프레임 설정
                            pose_msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 설정

                            pose_msg.pose.position.x = tvec[0][0] / 1000.0
                            pose_msg.pose.position.y = tvec[1][0] / 1000.0
                            pose_msg.pose.position.z = tvec[2][0] / 1000.0
                            pose_msg.pose.orientation.w = qw * qw_rot - qx * qx_rot - qy * qy_rot - qz * qz_rot
                            pose_msg.pose.orientation.x = qx * qw_rot + qw * qx_rot + qy * qz_rot - qz * qy_rot
                            pose_msg.pose.orientation.y = qy * qw_rot + qw * qy_rot + qz * qx_rot - qx * qz_rot
                            pose_msg.pose.orientation.z = qz * qw_rot + qw * qz_rot + qx * qy_rot - qy * qx_rot
                            # pose_msg.pose.orientation.w = quat[1]
                            # pose_msg.pose.orientation.x = quat[2]
                            # pose_msg.pose.orientation.y = quat[3]
                            # pose_msg.pose.orientation.z = quat[0]

                            # tf2 브로드캐스트
                            t = TransformStamped()
                            t.header.stamp = self.get_clock().now().to_msg()
                            t.header.frame_id = 'camera'
                            t.child_frame_id = f'aruco_marker_{marker_id[0]}'
                            t.transform.translation.x = tvec[0][0] / 1000.0
                            t.transform.translation.y = tvec[1][0] / 1000.0
                            t.transform.translation.z = tvec[2][0] / 1000.0
                            t.transform.rotation.x = quat[1]
                            t.transform.rotation.y = quat[2]
                            t.transform.rotation.z = quat[3]
                            t.transform.rotation.w = quat[0]

                            self.tf_broadcaster.sendTransform(t)
                            self.pose_publisher.publish(pose_msg)

                            # ArUco 마커의 좌표축을 이미지에 그리기
                            cv2.drawFrameAxes(img, self.cmtx, self.dist, rvec, tvec, self.marker_size / 2)

                            # 마커 ID를 이미지에 표시
                            cv2.putText(img, f'ID: {marker_id[0]}', (int(topLeft[0]), int(topLeft[1] - 10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)


        # 결과 이미지 ROS2 메시지로 변환하여 퍼블리시
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    if aruco_detector.pipeline is not None:
        try:
            rclpy.spin(aruco_detector)
        except KeyboardInterrupt:
            pass
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()