#include "Gripper.h"
long gripperencPos = 0;
#define RCSOFTCHECK(fn) // 필요한 매크로 정의
#define RCCHECK(fn)

Gripper* Gripper::instance = nullptr;
Gripper::Gripper(){}

// void Gripper::error_loop() {
//     while (1) {
//         digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//         delay(100);
//     }
// }

void Gripper::subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    if (msg->data == 1) {
        if (abs(gripperencPos) < 1000) {
            digitalWrite(motorDir1, HIGH);
            digitalWrite(motorDir2, LOW);
        } else {
            digitalWrite(motorDir1, HIGH);
            digitalWrite(motorDir2, HIGH);
            pub_msg.data = 2;
            RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        }
    }
}

void Gripper::doEncoderA() {
  gripperencPos += digitalRead(encoderPinA) == digitalRead(encoderPinB) ? 1 : -1;
}

void Gripper::doEncoderB() {
  gripperencPos += digitalRead(encoderPinA) == digitalRead(encoderPinB) ? -1 : 1;
}

void Gripper::EncoderA() {
  instance->doEncoderA();
}
void Gripper::EncoderB() {
  instance->doEncoderB();
}

// void Gripper::publish_imu_data(cIMU& imu) {
//     imu_msg.header.stamp.sec = millis() / 1000;
//     imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
//     strcpy(imu_msg.header.frame_id.data, "imu_link");
//     imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
//     imu_msg.header.frame_id.capacity = sizeof(imu_msg.header.frame_id.data);

//     imu_msg.orientation.x = imu.rpy[0];
//     imu_msg.orientation.y = imu.rpy[1];
//     imu_msg.orientation.z = imu.rpy[2];
//     imu_msg.orientation.w = 0.0; // 필요시 수정

//     imu_msg.angular_velocity.x = imu.gyroData[0] * GYRO_FACTOR;
//     imu_msg.angular_velocity.y = imu.gyroData[1] * GYRO_FACTOR;
//     imu_msg.angular_velocity.z = imu.gyroData[2] * GYRO_FACTOR;

//     imu_msg.linear_acceleration.x = imu.accData[0] * ACCEL_FACTOR;
//     imu_msg.linear_acceleration.y = imu.accData[1] * ACCEL_FACTOR;
//     imu_msg.linear_acceleration.z = imu.accData[2] * ACCEL_FACTOR;

//     Serial.print("Orienta#include <IMU.h>tion: ");
//     Serial.print(imu_msg.orientation.x); Serial.print(", ");
//     Serial.print(imu_msg.orientation.y); Serial.print(", ");
//     Serial.println(imu_msg.orientation.z);

//     Serial.print("Gyro: ");
//     Serial.print(imu_msg.angular_velocity.x); Serial.print(", ");
//     Serial.print(imu_msg.angular_velocity.y); Serial.print(", ");
//     Serial.println(imu_msg.angular_velocity.z);

//     Serial.print("Accel: ");
//     Serial.print(imu_msg.linear_acceleration.x); Serial.print(", ");
//     Serial.print(imu_msg.linear_acceleration.y); Serial.print(", ");
//     Serial.println(imu_msg.linear_acceleration.z);

//     RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg,NULL));
//     RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
//     //Serial.println("rcl_ok");
// }


void Gripper::gripper_setup() {
    set_microros_transports();


    pinMode(motorDir1, OUTPUT);
    pinMode(motorDir2, OUTPUT);
    analogWrite(ENABLE, 255);

    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), EncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), EncoderB, CHANGE);
    
    allocator = rcl_get_default_allocator();

    // Initialize micro-ROS
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // Create ROS subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "start_arduino"));

    // Create ROS publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "start_trailer"));

    // ROS IMU publisher initialization
    // RCCHECK(rclc_publisher_init_default(
    //     &imu_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "imu_data"));

    // Initialize ROS executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &Gripper::subscription_callback, ON_NEW_DATA));
}

void Gripper::gripper_loop() {
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    Serial.print("gripper_loop");
}

