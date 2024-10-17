#ifndef FakeSign_H_
#define FakeSign_H_

#include "turtlebot3_sensor.h"

/* functions */
void init_imu();
void setPinMode();
void update_imu();  // 처음 init_imu 부분에서 imu 한 번 업데이트 후 시작  
void setInitAngle();

/* Parameters */
#define ODOM_INTERVAL 25 //ms단위. controller 주기인 20hz(50ms)보단 빠르게, odom주기인 100hz(20ms)보단 느리게. 이값이 너무 커지면 로봇이 천천히 회전할때 delta_theta값이 0이 되어버림.
#define PI 3.141592

/* pinMode */
#define LED_PIN 13
#define motorDirL 12
#define motorDirR 11
#define motorPwmL 6
#define motorPwmR 9

#define ENC_L_2 2
#define ENC_L_4 4
#define ENC_R_7 7
#define ENC_R_8 8


static Turtlebot3Sensor sensors;

String s_pulse_R;  // convert wheel pulse to string operator
String s_pulse_L;
String s_delta_theta;
String cmd;
String logic_heading_angle;

int logic_warn_sound;
int heading_dir;

float linx;  // cmd_vel linear.x
float angz;  // cmd_vel angular.z 
float theta;  // IMU angle (Yaw)
float last_theta;
float delta_theta;
float theta_sum;
float heading_angle;

float gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;
float ori_x, ori_y, ori_z, ori_w;

bool initIMU_ret; 

int16_t input_B = 0;

float prev_time;
float init_theta;

String rotate_done;


/* functions */
void setPinMode(){
  pinMode(motorDirL, OUTPUT);
  pinMode(motorDirR, OUTPUT);
  pinMode(motorPwmL, OUTPUT);
  pinMode(motorPwmR, OUTPUT);

  pinMode(45, INPUT);  // or INPUT_PULLUP
  pinMode(72, INPUT);
}

void init_imu(){
  initIMU_ret = sensors.init();
  sensors.initIMU();
  sensors.calibrationGyro();
  update_imu();
}

void update_imu()
{
  sensors.updateIMU();
  float* p_imu_data;

  p_imu_data = sensors.getImuAngularVelocity();
  gyro_x = p_imu_data[0];
  gyro_y = p_imu_data[1];
  gyro_z = p_imu_data[2];

  p_imu_data = sensors.getImuLinearAcc();
  acc_x = p_imu_data[0];
  acc_y = p_imu_data[1];
  acc_z = p_imu_data[2];

  p_imu_data = sensors.getImuMagnetic();
  mag_x = p_imu_data[0];
  mag_y = p_imu_data[1];
  mag_z = p_imu_data[2];

  /* ori_x, ori_y가 0이 안나와서 일단 0으로 고정시킴. 
  calibration문제이거나 실제로 opencr보드를 좀 기울게 설치해서 그러거나. */
  p_imu_data = sensors.getOrientation();
  ori_w = p_imu_data[0];
  ori_x = p_imu_data[1];
  ori_y = p_imu_data[2];
  ori_z = p_imu_data[3];
}

void setInitAngle(){
  init_theta = atan2f(ori_x * ori_y + ori_w * ori_z, 0.5f - ori_y * ori_y - ori_z  * ori_z);  //0.9-> 1.0이 아니라 0.9->0.10이 되어버린다. 아두이노 float->string 변환 안됨. 값이상해짐. 이부분 파이썬으로 옮기기.
  last_theta = init_theta;
  delta_theta = 0.;
  rotate_done = "0";
}


#endif
