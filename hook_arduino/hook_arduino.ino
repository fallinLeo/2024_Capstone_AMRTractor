#define encoderPinA 2
#define encoderPinB 3
#define motorDirPin 8
#define motorPwmPin 9
#define BUTTON 7

//---------------------선언---------------------//
long encoderPos = 0;
String str_ang;
float ang;
float python_distance = 0;
float target_distance = 0;
float error1, error2;
int angle_check = 0;
int All_mode = 0;

float Kp = 1.0;
float Ki = 0;
float Kd = 0;
float Ltime = 0.001;
float control = 0;
float P_control, I_control, D_control;
float error_previous = 0;

//---------------------------------------------//


void doEncoderA() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}
void doEncoderB() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}

void motorcontrol(bool Dir,int Pwm){
  //True -> HIGH & False : LOW
  if(Dir==True){
    digitalWrite(motorDirPin,HIGH);
    analogWrite(motorPwmPin,Pwm);
  }
  else{
    digitalWrite(motorDirPin,LOW);
    analogWrite(motorPwmPin,Pwm);
  }
}

// void doMotor(bool dir, int vel)
// {
//   digitalWrite(motorDirPin, dir);
//   analogWrite(motorPwmPin, min(vel, 255));
// }
void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(0, doEncoderB, CHANGE);

  pinMode(motorDirPin, OUTPUT);
  pinMode(motorPwmPin, OUTPUT);
  Serial.begin(9600);
}
//각도 뽑는식-> EncoderPos*30/641


void loop() {
  switch(All_mode){
    case 0:
      if (angle_check == 0) {
        motorcontrol(True,22);
      if (digitalRead(BUTTON) == HIGH) {
        //Serial.println('Button on');
        ang = 0;
        encoderPos = 0;
        angle_check = 1;
      }
      } else if (angle_check == 1) {
        ang = 45;
        angle_check = 2;
      } 
      error1 = ang - encoderPos * 0.6006;  //504/1 , 294/1 , 144/1, 104/1, 61/1, 49/1

      if (error1 < -2) {
        motorcontrol(True,22);
      } else if (error1 > 2) {
        motorcontrol(False,20);        
      }

      else if (error1 > -2 || error1 < 2) {
        motorcontrol(True,0);
        All_mode = 1;
      }
      break;  
    case 1:
      if(angle_check ==2)
      {
        if (Serial.available() > 0) {
          python_distance = Serial.parseFloat();
          python_distance = float(python_distance);
        }
        error2 = target_distance - python_distance;
        P_control = (Kp * error2);
        I_control += (Ki * error2 * Ltime);
        D_control = Kd * (error2 - error_previous) / (Ltime);
        control = P_control + I_control + D_control;
        error_previous = error2;

        if (control >= 1) {
          motorcontrol(True,25);
        } else if (control <= -1) {
          motorcontrol(False,25);
        } else {
          motorcontrol(True,0);
        }
      }
      else: break;
      
  }
  Serial.println(control);
}

//doMotor((control>=0)?HIGH:LOW,min(abs(control),255));