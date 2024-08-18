#ifndef Gripper_H
#define Gripper_H

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <Arduino.h>
#include <IMU.h>

// Encoder variables
#define encoderPinA 4
#define encoderPinB 7
extern long gripperencPos;

// Motor control pins
#define ENABLE 5
#define motorDir1 10
#define motorDir2 11

#define ACCEL_FACTOR 0.000598550415
#define GYRO_FACTOR 0.0010642
#define MAG_FACTOR 15e-8

class Gripper {
private:
    void subscription_callback(const void *msgin);
    void error_loop();

    // ROS variables
    rcl_subscription_t subscriber;
    std_msgs__msg__Int32 msg;
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 pub_msg;
    rcl_publisher_t imu_publisher;
    sensor_msgs__msg__Imu imu_msg;
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    // Timing variables
    static uint32_t tTime[3];

public:
    Gripper();
    void doEncoderA();
    void doEncoderB();
    static void EncoderA(); // 메서드 이름 수정
    static void EncoderB(); // 메서드 이름 수정
    static Gripper* instance; // 정적 멤버 변수 선언
    void gripper_setup();
    void gripper_loop();
    void publish_imu_data(cIMU& imu);
};

#endif
