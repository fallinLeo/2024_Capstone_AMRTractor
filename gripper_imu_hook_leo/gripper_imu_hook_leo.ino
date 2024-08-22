#include "Gripper.h"
#include "Hook.h"
// #include <IMU.h>


Gripper mygripper;
Hook myHook;
cIMU IMU;


void setup(){
  Serial.begin(9600);
  myHook.hook_setup();
  mygripper.gripper_setup();
  IMU.begin();

}



void loop(){
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;



  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    //led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];



  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

    Serial.print(imu_time);
    Serial.print(" ");
    Serial.print(IMU.rpy[0]);
    Serial.print(" ");
    Serial.print(IMU.rpy[1]);
    Serial.print(" ");
    Serial.print(IMU.rpy[2]);
    Serial.print(" \t");
    Serial.print(IMU.accData[0] * ACCEL_FACTOR);    // ACC X
    Serial.print(" \t");
    Serial.print(IMU.accData[1] * ACCEL_FACTOR);    // ACC Y
    Serial.print(" \t");
    Serial.print(IMU.accData[2] * ACCEL_FACTOR);    // ACC Z
    Serial.print(" ");
    Serial.print(" \t");
    Serial.print(IMU.gyroData[0] * GYRO_FACTOR);    // GYRO X
    Serial.print(" \t");
    Serial.print(IMU.gyroData[1]* GYRO_FACTOR);    // GYRO Y
    Serial.print(" \t");
    Serial.println(IMU.gyroData[2] * GYRO_FACTOR);    // GYRO Z
  }


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
        mygripper.gripper_loop();
        myHook.hook_loop();
      }

      Serial.print("ACC Cali End ");
    }
  }
}


