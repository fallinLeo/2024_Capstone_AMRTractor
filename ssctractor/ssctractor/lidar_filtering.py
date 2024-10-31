#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import math
def deg_2_rad(x):
    return x * np.pi / 180.0

class LidarVisualization(Node):
    def __init__(self):
        super().__init__('lidar_filtering')
        # 각도 범위 지정
        self.angle_min = -80.0 * np.pi / 180.0
        self.angle_max = 80.0 * np.pi / 180.0

        self.lidar_data = None
        # scan2 토픽에 대한 퍼블리셔 추가
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        self.raw_lid_sub = self.create_subscription(LaserScan, 'raw_scan', self.lidar_callback, 10)
        self.raw_lid_sub  # prevent unused variable warning
    


    def lidar_callback(self,msg):
        self.lidar_data = msg

    def filter_lidar_data_and_publish(self):
        if self.lidar_data is not None:
            ranges = np.array(self.lidar_data.ranges)
            angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))
            
            # 범위 밖의 값을 np.nan으로 설정
            for i in range(len(ranges)):
                if self.angle_min <= angles[i] <= self.angle_max:
                    ranges[i] = np.nan
            
            # np.isinf를 사용하여 무한대 값을 np.nan으로 변경
            ranges[np.isinf(ranges)] = np.nan
            
            # 너무 큰 값들을 np.nan으로 변경
            ranges[ranges > 1e6] = np.nan
            
            # numpy 배열을 Python 리스트로 변환하고, np.nan을 float('nan')으로 변환
            filtered_ranges = [float('nan') if np.isnan(x) else x for x in ranges.tolist()]
            
            # 새로운 LaserScan 메시지 생성 및 발행
            new_scan = LaserScan()
            new_scan.header = self.lidar_data.header
            new_scan.angle_min = self.lidar_data.angle_min
            new_scan.angle_max = self.lidar_data.angle_max
            new_scan.angle_increment = self.lidar_data.angle_increment
            new_scan.time_increment = self.lidar_data.time_increment
            new_scan.scan_time = self.lidar_data.scan_time
            new_scan.range_min = self.lidar_data.range_min
            new_scan.range_max = self.lidar_data.range_max
            new_scan.ranges = filtered_ranges
            new_scan.intensities = self.lidar_data.intensities
            
            self.scan_pub.publish(new_scan)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.filter_lidar_data_and_publish()

def main(args=None):
    rclpy.init(args=args)
    lidar_visualization = LidarVisualization()
    lidar_visualization.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
