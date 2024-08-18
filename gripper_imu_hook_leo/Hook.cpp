#include "Hook.h"

Hook* Hook::instance = nullptr;

Hook::Hook(){
    // 생성자 초기화
}

void Hook::doEncoderA() {
  encoderPos += (digitalRead(hookPInA) == digitalRead(hookPInB)) ? 1 : -1;
}

void Hook::doEncoderB() {
  encoderPos += (digitalRead(hookPInA) == digitalRead(hookPInB)) ? -1 : 1;
}

void Hook::encoderA() {
  instance->doEncoderA(); // 인스턴스를 통해 비정적 멤버 함수 호출
}

void Hook::encoderB() {
  instance->doEncoderB(); // 인스턴스를 통해 비정적 멤버 함수 호출
}

void Hook::hook_setup() {
  pinMode(hookBUTTON, INPUT);
  pinMode(hookPInA, INPUT);
  pinMode(hookPInB, INPUT);
  attachInterrupt(digitalPinToInterrupt(hookPInA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hookPInB), encoderB, CHANGE);

  pinMode(hookDirPin, OUTPUT);
  pinMode(hookPwmPin, OUTPUT);
}

void Hook::hook_loop() {
  if (angle_check == 0) {
    digitalWrite(hookDirPin, HIGH);
    analogWrite(hookPwmPin, 25);
    if (digitalRead(hookBUTTON) == HIGH) {
      ang = 0;
      encoderPos = 0;
      angle_check = 1;
      Serial.println(python_check);
    }
  } else if (angle_check == 1) {
    ang = 45;
    angle_check = 2;    
  } else if (angle_check == 2) {
    error = ang - encoderPos * 0.6006; 
    
    if (error < -2) {
      digitalWrite(hookDirPin, HIGH);
      analogWrite(hookPwmPin, 25);
    } else if (error > 2) {
      digitalWrite(hookDirPin, LOW);
      analogWrite(hookPwmPin, 25);
    } else if (error > -2 && error < 2) {
      digitalWrite(hookDirPin, HIGH);
      analogWrite(hookPwmPin, 0);
    }  
  }
}

