//#include "Gripper.h"
//#include "Hook.h"
#include <IMU.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Gripper mygripper;
// Hook myHook;
cIMU IMU;
#define ACCEL_FACTOR 0.000598550415
#define GYRO_FACTOR 0.0010642
#define MAG_FACTOR 15e-8
#define encoderPinA 4
#define encoderPinB 7
#define ENABLE 5
#define motorDir1 10
#define motorDir2 11



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //delay(100);
  }
}

//ros2 관련 변수
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

volatile long encoderPos = 0;

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

    // 수신된 메시지가 9이면 "nine" 출력
    if (msg->data == 9) {
      digitalWrite(motorDir1, HIGH);
      digitalWrite(motorDir2, LOW);
    }
    // 수신된 메시지가 8이면 "eight" 출력
    else if (msg->data == 8) {
      digitalWrite(motorDir1, HIGH);
      digitalWrite(motorDir2, HIGH);
    }

    else if (msg->data == 7) {
      digitalWrite(motorDir1, LOW);
      digitalWrite(motorDir2, HIGH);
    }
    
    // 기존 동작 코드 (예: 1일 때 모터 작동)
    if (msg->data == 1) {
      if (abs(encoderPos) < 2300) {
          digitalWrite(motorDir1, HIGH);
          digitalWrite(motorDir2, LOW);
      } else {
          digitalWrite(motorDir1, HIGH);
          digitalWrite(motorDir2, HIGH);
      }
    }
}

void doEncoderA() {
    encoderPos += digitalRead(encoderPinA) == digitalRead(encoderPinB) ? 1 : -1;
}

void doEncoderB() {
    encoderPos += digitalRead(encoderPinA) == digitalRead(encoderPinB) ? -1 : 1;
}


void setup(){
    // Arduino 초기화
    Serial.begin(115200);
    pinMode(motorDir1, OUTPUT);
    pinMode(motorDir2, OUTPUT);
    analogWrite(ENABLE, 255);

    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

    // micro-ROS 초기화
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    
    // micro-ROS 노드 및 executor 초기화
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "opencr_node", "", &support));
    
    // ROS 2 토픽 구독자 초기화
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "gripper_cmd"  // 구독할 토픽 이름을 "gripper_cmd"로 설정
    ));

    // Executor 초기화 및 구독자 추가
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    IMU.begin();
}

void loop(){
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;
  Serial.println("ok");
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];

  // if( (millis()-tTime[1]) >= 50 )
  // {
  //   tTime[1] = millis();
  //   Serial.print(imu_time);
  //   Serial.print(" ");
  //   Serial.print(IMU.rpy[0]);
  //   Serial.print(" ");
  //   Serial.print(IMU.rpy[1]);
  //   Serial.print(" ");
  //   Serial.print(IMU.rpy[2]);
  //   Serial.print(" \t");
  //   Serial.print(IMU.accData[0] * ACCEL_FACTOR);    // ACC X
  //   Serial.print(" \t");
  //   Serial.print(IMU.accData[1] * ACCEL_FACTOR);    // ACC Y
  //   Serial.print(" \t");
  //   Serial.print(IMU.accData[2] * ACCEL_FACTOR);    // ACC Z
  //   Serial.print(" ");
  //   Serial.print(" \t");
  //   Serial.print(IMU.gyroData[0] * GYRO_FACTOR);    // GYRO X
  //   Serial.print(" \t");
  //   Serial.print(IMU.gyroData[1]* GYRO_FACTOR);    // GYRO Y
  //   Serial.print(" \t");
  //   Serial.println(IMU.gyroData[2] * GYRO_FACTOR);    // GYRO Z
  // }

  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
      }

      Serial.print("ACC Cali End ");
    }
  }
}
