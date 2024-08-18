#include "Gripper.h"
#include "Hook.h"
#include <IMU.h>


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
  
  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];

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
        mygripper.publish_imu_data(IMU);
      }
    }
  }
  mygripper.gripper_loop();
  myHook.hook_loop();
  
}
