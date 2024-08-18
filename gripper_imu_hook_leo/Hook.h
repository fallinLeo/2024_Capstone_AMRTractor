#ifndef HOOK_H
#define HOOK_H

#define hookPInA 2
#define hookPInB 3
#define hookDirPin 8
#define hookPwmPin 9
#define hookBUTTON 7

#include <Arduino.h>

class Hook {
  private:
    long encoderPos; // 기본값 없이 선언
    String str_ang;
    float ang;
    float pyton_angle;
    int error;
    int angle_check = 0;
    int python_check = 1;

    static Hook* instance; // Hook 클래스의 인스턴스를 저장하는 정적 포인터

  public:
    Hook();

    void doEncoderA();
    void doEncoderB();
    static void encoderA();
    static void encoderB();
    void hook_setup();
    void hook_loop();

    // encoderPos에 대한 접근자 메서드 추가
    long getEncoderPos() const { return encoderPos; }
    void setEncoderPos(long pos) { encoderPos = pos; }
};


#endif // HOOK_H

