import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from mss import mss  # mss 라이브러리를 사용한 화면 캡처
# from rclpy.executors import MultiThreadedExecutor

class ScreenCapturePublisher(Node):
    def __init__(self):
        super().__init__('screen_capture_publisher')
       
        # ROS2 퍼블리셔 설정
        self.publisher = self.create_publisher(Image, 'screen_image', 10)
        self.bridge = CvBridge()
        img = np.zeros((950, 240, 3), dtype=np.uint8)

        # mss를 사용해 화면 캡처 설정
        self.sct = mss()
        self.monitor = {
            "top": 120,  # 캡처할 영역의 y 좌표
            "left": 1500,  # 캡처할 영역의 x 좌표
            "width": 240,  # 캡처할 영역의 너비
            "height": 950  # 캡처할 영역의 높이
        }
        # self.monitor = self.sct.monitors[1]
        # print(self.sct.monitors)
        # print(f"Monitor configuration: {self.monitor}")

        # 타이머로 주기적인 캡처 설정
        self.timer = self.create_timer(2, self.timer_callback)  # 10Hz로 캡처
        self.publish_timer = self.create_timer(2, self.publish_callback)

        self.current_img = None

    def timer_callback(self):
        # 화면의 일부를 캡처
        screen_shot = self.sct.grab(self.monitor)
        # numpy 배열로 변환하고 BGR로 변환 (OpenCV 호환)
        img = np.array(screen_shot)
        # img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img = img[:, :, :3]
        img = cv2.resize(img, (int(img.shape[1] / 10), int(img.shape[0] / 10)))
        # OpenCV 이미지 -> ROS2 이미지 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # ROS2 이미지 퍼블리시
        self.publisher.publish(ros_image)
        # self.get_logger().info('Publishing screen image')
        self.current_img = img

    def publish_callback(self):
        if self.current_img is not None:
            ros_image = self.bridge.cv2_to_imgmsg(self.current_img, encoding="bgr8")
            self.publisher.publish(ros_image)
            



def main(args=None):
    rclpy.init(args=args)
    screen_capture_publisher = ScreenCapturePublisher()
    rclpy.spin(screen_capture_publisher)
    screen_capture_publisher.destroy_node()
    rclpy.shutdown()
    # executor = MultiThreadedExecutor()
    # executor.add_node(screen_capture_publisher)
    # try:
    #     executor.spin()
    # finally:
    #     screen_capture_publisher.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
