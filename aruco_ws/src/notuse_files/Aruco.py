import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import transforms3d
import tf2_ros

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher_ = self.create_publisher(Image, 'aruco_image', 10)
        self.pose_publisher = self.create_publisher(PoseStamped,'pose',10)
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
            for i, corner in enumerate(corners):
                corner = corner.reshape((4, 2))
                topLeft, topRight, bottomRight, bottomLeft = corner

                # PnP 문제 해결
                ret, rvec, tvec = cv2.solvePnP(self.marker_3d_edges, corner, self.cmtx, self.dist)
                if ret:
                    # 회전 벡터를 쿼터니언으로 변환
                    rotation_matrix = cv2.Rodrigues(rvec)[0]
                    quat = transforms3d.quaternions.mat2quat(rotation_matrix)

                    
                    pose_msg = PoseStamped()  # PoseStamped로 변경하여 헤더 포함
                    pose_msg.header.frame_id = 'camera'  # 헤더 프레임 설정
                    pose_msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 설정

                    pose_msg.pose.position.x = tvec[0][0] / 1000.0
                    pose_msg.pose.position.y = tvec[1][0] / 1000.0
                    pose_msg.pose.position.z = tvec[2][0] / 1000.0
                    pose_msg.pose.orientation.x = quat[1]
                    pose_msg.pose.orientation.y = quat[2]
                    pose_msg.pose.orientation.z = quat[3]
                    pose_msg.pose.orientation.w = quat[0]
                    # tf2 브로드캐스트
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera'
                    t.child_frame_id = f'aruco_marker_{ids[i][0]}'
                    t.transform.translation.x = tvec[0][0] / 1000.0
                    t.transform.translation.y = -tvec[1][0] / 1000.0
                    t.transform.translation.z = tvec[2][0] / 1000.0
                    t.transform.rotation.x = quat[1]
                    t.transform.rotation.y = quat[2]
                    t.transform.rotation.z = quat[3]
                    t.transform.rotation.w = quat[0]

                    self.tf_broadcaster.sendTransform(t) 
                    self.pose_publisher.publish(pose_msg)  

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
