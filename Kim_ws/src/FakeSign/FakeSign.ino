#include "CEncoder.h"
#include "FakeSign.h"

/* DC Motor instances*/
CEncoder* CEncoder::instances[2] = {NULL, NULL};  // Left motor, Right motor
CEncoder CMotor_L;
CEncoder CMotor_R;

void setup() {
  /* Serial, pin setting */
  Serial.begin(9600);
  setPinMode();

  /* IMU setting */
  init_imu();
  setInitAngle();

  /* DC Motor pin setting */
  CMotor_L.begin(ENC_L_2, ENC_L_4, 1);   // int pin1, int pin2, motor_num
  CMotor_R.begin(ENC_R_7, ENC_R_8, 0);
}


void loop() {

  if(millis() - prev_time >= ODOM_INTERVAL){
    prev_time = millis();
    
    CMotor_L.CurrentVel(millis() - prev_time);
    CMotor_R.CurrentVel(millis() - prev_time);
    
    update_imu();
    
    theta = atan2f(ori_w * ori_z, 0.5f - ori_z  * ori_z); 
    delta_theta = theta - last_theta;
    last_theta = theta;
    
    rotate_done= "0";

    if(delta_theta>PI){  //정확히 -3.13xx에서 3.13xx로 회전할때.
      delta_theta = -1*((2*PI)-delta_theta);  
    }
    else if(delta_theta<-PI){
      delta_theta = (2*PI)-abs(delta_theta);
    }
    // heading_angle=0.513;
    if(heading_angle!=0.){
      theta_sum += abs(delta_theta);
      if(theta_sum >= abs(heading_angle)-0.001){ //+-0.286도
      heading_angle=0.;
      angz = 0.;
      theta_sum=0;
      CMotor_L.CMDVELtoTarget(linx, angz);
      CMotor_R.CMDVELtoTarget(linx, angz);
      rotate_done= "1";
      }
    }
    
    s_pulse_R = String(int(CMotor_R.pulse*1000));
    s_pulse_L = String(int(CMotor_L.pulse*1000));
    s_delta_theta = String(int(delta_theta*10000));
    Serial.println("s"+s_delta_theta+"a"+s_pulse_L+"b"+s_pulse_R+"c"+rotate_done+"e");
  }
  
  
  /* (DC Motor) Encoder pid */
  CMotor_L.EncoderPID();
  CMotor_R.EncoderPID();

  /* (DC Motor) convert pid result to PWM */
  CMotor_L.PIDtoPWM();
  CMotor_R.PIDtoPWM();

  /* (DC Motor) motor output */
  digitalWrite(motorDirL, !CMotor_L.dir);
  digitalWrite(motorDirR, !CMotor_R.dir);
  analogWrite(motorPwmL, CMotor_L.pwm);
  analogWrite(motorPwmR, CMotor_R.pwm); 
}


/* Serial Communication Read */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    cmd += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      if(cmd.startsWith("s")){   //창민이꺼 로봇팔 동작할때는 시작문자를 다른거써서 프로토콜 바꿔사용하기.
        // Serial.println(cmd);
        linx = cmd.substring(1,cmd.indexOf(',')).toFloat();
        angz = cmd.substring(cmd.indexOf(',')+1,cmd.indexOf('a')).toFloat();
        logic_warn_sound = cmd.substring(cmd.indexOf('a')+1,cmd.indexOf('b')).toInt();
        heading_angle = cmd.substring(cmd.indexOf('b')+1,cmd.length()).toFloat();
        // Serial.print(String(int(heading_angle*1000)));
        if (logic_warn_sound==1){
          sensors.makeMelody(1);
          sensors.onMelody();
          CMotor_R.pulse=0;
          CMotor_L.pulse=0;
          Serial.read();
          return;
        }
        if (heading_angle!=0.){
          if(heading_angle>0.) angz=0.4;
          else angz = -0.4;
          theta = atan2f(ori_w * ori_z, 0.5f - ori_z  * ori_z);  //0.9-> 1.0이 아니라 0.9->0.10이 되어버린다. 아두이노 float->string 변환 안됨. 값이상해짐. 이부분 파이썬으로 옮기기.
          last_theta = theta;
          theta_sum = 0;
          Serial.read();
        }

        CMotor_L.CMDVELtoTarget(linx, angz);
        CMotor_R.CMDVELtoTarget(linx, angz);
      }
        cmd = "";
    }
  }
}
