import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32, Int32, String, Bool
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import transforms3d
import tf2_ros
import math
import base64

class DualArucoDetector(Node):
    def __init__(self):
        super().__init__('dual_aruco_detector')
        self.publisher_realsense = self.create_publisher(Image, 'realsense_image', 10)
        self.pose_publisher_realsense = self.create_publisher(PoseStamped, 'pose_realsense', 10)
        self.theta_publisher = self.create_publisher(Float32, 'aruco_theta', 10)
        self.charge_flag = self.create_subscription(Bool,'charge_start',self.charge_flag_callback,10)
        self.chag_flag = False

        self.publisher_webcam = self.create_publisher(Image, 'webcam_image', 10)
        self.pose_publisher_webcam = self.create_publisher(Pose, 'pose_webcam', 10)
        self.image_pub_webcam = self.create_publisher(String, 'webcam/image_processed', 10)

        self.set_id = 0
        self.subscription = self.create_subscription(
            Int32,
            'set_id',
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        
        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # RealSense 카메라 내부 파라미터 얻기
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.cmtx_realsense = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist_realsense = np.array(intr.coeffs)

        # 웹캠 초기화
        self.cap = cv2.VideoCapture(8)  # 기본 웹캠 사용
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 웹캠 내부 파라미터 설정
        self.cmtx_webcam = np.array([[1034.15573, 0, 629.042812],
                                     [0, 1035.85237, 321.766957],
                                     [0, 0, 1]], dtype='float32')
        self.dist_webcam = np.array([[3.09506194e-01, -3.93378737e+00, -1.19794465e-02,
                                      -5.13587967e-03, 1.37189534e+01]], dtype='float32')

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
        self.timer = self.create_timer(0.033, self.timer_callback)

    def listener_callback(self, msg):
        self.set_id = msg.data
        self.get_logger().info(f'Received set_id: {self.set_id}')

    def process_aruco(self, img, cmtx, dist, frame_id):
        # ArUco 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.set_id:
                    corner = corners[i].reshape((4, 2))
                    ret, rvec, tvec = cv2.solvePnP(self.marker_3d_edges, corner, cmtx, dist)
                    if ret:
                        rotation_matrix = cv2.Rodrigues(rvec)[0]
                        quat = transforms3d.quaternions.mat2quat(rotation_matrix)
                        
                        # Pose 메시지 생성 및 퍼블리시
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = frame_id
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
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
                        t.header.frame_id = frame_id
                        t.child_frame_id = f'aruco_marker_{marker_id[0]}'
                        t.transform.translation.x = tvec[0][0] / 1000.0
                        t.transform.translation.y = tvec[1][0] / 1000.0
                        t.transform.translation.z = tvec[2][0] / 1000.0
                        t.transform.rotation.x = quat[1]
                        t.transform.rotation.y = quat[2]
                        t.transform.rotation.z = quat[3]
                        t.transform.rotation.w = quat[0]

                        self.tf_broadcaster.sendTransform(t)

                        cv2.drawFrameAxes(img, cmtx, dist, rvec, tvec, self.marker_size / 2)
                        cv2.putText(img, f'ID: {marker_id[0]}', (int(corner[0][0]), int(corner[0][1] - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)

                        if frame_id == 'realsense_camera':
                            self.pose_publisher_realsense.publish(pose_msg)
                        else:
                            pose_simple_msg = Pose(
                                position=pose_msg.pose.position,
                                orientation=pose_msg.pose.orientation)
                            self.pose_publisher_webcam.publish(pose_simple_msg)

    def timer_callback(self):
        # RealSense 카메라 처리
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            img_realsense = np.asanyarray(color_frame.get_data())
            self.process_aruco(img_realsense, self.cmtx_realsense, self.dist_realsense, 'realsense_camera')
            self.publisher_realsense.publish(self.bridge.cv2_to_imgmsg(img_realsense, "bgr8"))
            cv2.imshow('RealSense Aruco Detector', img_realsense)

        # 웹캠 처리
        ret, img_webcam = self.cap.read()
        if ret:
            self.process_aruco(img_webcam, self.cmtx_webcam, self.dist_webcam, 'webcam')
            self.publisher_webcam.publish(self.bridge.cv2_to_imgmsg(img_webcam, "bgr8"))

            _, jpeg_image = cv2.imencode('.jpg', img_webcam)
            base64_image = base64.b64encode(jpeg_image.tobytes()).decode('utf-8')
            image_msg = String(data=base64_image)
            self.image_pub_webcam.publish(image_msg)
            
            cv2.imshow('Webcam Aruco Detector', img_webcam)
        cv2.imshow('RealSense Aruco Detector', img_realsense)
        cv2.imshow('Webcam Aruco Detector', img_webcam)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.pipeline.stop()
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = DualArucoDetector()
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
