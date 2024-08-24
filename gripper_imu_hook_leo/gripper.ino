//Gripper encoder //양으로 돌리면 -값으로 나옴
//#include <Servo.h> //로 주석한 부분은 필요시 사용하면 된다.
//Servo myservo;
#define encoderPinA 4
#define encoderPinB 7
long encoderPos = 0;

void doEncoderA(){encoderPos +=(digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){encoderPos +=(digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}
long angleL = 0;
long angleR = 0;
long distanceL = 0;
long distanceR = 0;
volatile int lastEncodedL = 0;
volatile int lastEncodedR = 0;
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

//---------motor-------------//
#define ENABLE 5
#define motorDir1 10
#define motorDir2 11
//---------------------------//

void setup() {


//myservo.attach(9);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  analogWrite(ENABLE,255);
  // pinMode(ENABLE, OUTPUT);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  
  Serial.begin(9600);
}

void loop(){

//Do stuff here
  Serial.print("encoderPos : ");
  Serial.println(encoderPos);

  // analogWrite(ENABLE,255);
  digitalWrite(motorDir1,LOW);
  digitalWrite(motorDir2,HIGH);
  
// myservo.write(angle);
}