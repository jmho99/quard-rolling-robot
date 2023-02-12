//블루투스 설정---------------------------------------
#include <SoftwareSerial.h>

SoftwareSerial bt(0, 1); //(cnc쉴드 사용시 0,1),(UNO사용시 2,3)

char userInput;
//스텝모터 설정---------------------------------------
#include <AccelStepper.h>

const int xstep_pin = 2;
const int ystep_pin = 3;
const int xdir_pin  = 5;
const int ydir_pin  = 6; //cnc쉴드에서 배정된 핀번호

int stepspeed = 1500; //스텝모터 속도

long motorposition1 = 0;
long motorposition2 = 0;
//long Lmotorposition=0;

float Angle1;
float Angle2;
float stepangle = 1.8; //스텝각
int microcontrol = 2; //마이크로스텝

//float Distance2;
//float radius=10; //바퀴 반지름

int pulse;

AccelStepper stepper1(AccelStepper::DRIVER, xstep_pin, xdir_pin);
AccelStepper stepper2(AccelStepper::DRIVER, ystep_pin, ydir_pin);
//서보모터 설정---------------------------------------
#include <Arduino.h>
#define USE_PCA9685_SERVO_EXPANDER
#include "ServoEasing.hpp"

ServoEasing Servo0(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo2(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo3(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo4(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo5(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo6(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo7(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo8(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo9(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo10(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo11(PCA9685_DEFAULT_ADDRESS, &Wire);
const int Millis = 500;
const int SERVO_PIN[12] = {0, 1, 2,
                           3, 4, 5,
                           6, 7, 8,
                           9, 10, 11
                          };


int x01 = 127.5 + 20; //크립시작할때각도
int x02 = 37.5 + 0;
int x03 = 122.5 + 25;
int x04 = 57.5 - 15;

int y01 = 30 - 7.5;
int y02 = 0 + 32.5;
int y03 = 30 - 2.5;
int y04 = 0 + 25;

int z01 = 40 + 7.5;
int z02 = 80 - 32.5;
int z03 = 40 + 2.5;
int z04 = 70 - 10;

int y11 = 30; //크립다리들때각도
int z11 = 30;

int x22 = 70; //크립다리짧게각도
int y22 = 0;
int z22 = 0;

int x33 = 30; //크립다리길게각도
int y33 = 0;
int z33 = 0;

int b11 = 30; //롤링으로 변환시 다리들때각도
int c11 = 30;

int a22 = 70; //롤링으로 변환시 다리짧게각도
int b22 = 0;
int c22 = 0;

int a33 = 30; //롤링으로 변환시 다리길게각도
int b33 = 0;
int c33 = 0;

int b44 = 120; //롤링으로 변환시
int c44 = 45;

int Servoinputlast[12];
int Servoinput[12] = {x01+a33,x02-a33,x03+a33,x04-a33,
                      y01+b44+35,y02+60+105,y03+b44+35,y04+60+105,
                      z01-c44,z02+15-75,z03-c44,z04+15-75
                     };
//----------------------------------------------------
void setup() {
  Serial.begin(9600);
//블루투스---------------------------------------------
  bt.begin(38400);
//스텝모터---------------------------------------------
  stepper1.setMaxSpeed(4000);
  stepper2.setMaxSpeed(4000);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
//서보모터---------------------------------------------
  Servo0.attach(SERVO_PIN[0], Servoinput[0]);
  Servo1.attach(SERVO_PIN[1], Servoinput[1]);
  Servo2.attach(SERVO_PIN[2], Servoinput[2]);
  Servo3.attach(SERVO_PIN[3], Servoinput[3]);
  Servo4.attach(SERVO_PIN[4], Servoinput[4]);
  Servo5.attach(SERVO_PIN[5], Servoinput[5]);
  Servo6.attach(SERVO_PIN[6], Servoinput[6]);
  Servo7.attach(SERVO_PIN[7], Servoinput[7]);
  Servo8.attach(SERVO_PIN[8], Servoinput[8]);
  Servo9.attach(SERVO_PIN[9], Servoinput[9]);
  Servo10.attach(SERVO_PIN[10], Servoinput[10]);
  Servo11.attach(SERVO_PIN[11], Servoinput[11]);
  allservo();
}
void loop() {
  if (bt.available() > 0) {
    userInput = bt.read();
    Serial.println(userInput);

    if (Servoinput[4] == y01+b44+35&&Servoinput[5]==y02+60+105&&
        Servoinput[6] == y03+b44+35&&Servoinput[7]==y04+60+105){
      switch (userInput) {
        case 'f':
          forward();
          Serial.println(userInput);
          break;

        case 'b':
          back();
          Serial.println(userInput);
          break;

        case 'l':
          left();
          Serial.println(userInput);
          break;

        case 'r':
          right();
          Serial.println(userInput);
          break;

        case 's':
          stepper1.stop();
          stepper2.stop();
          stepper1.disableOutputs();
          stepper2.disableOutputs();
          //Distance2=2*PI*radius*(Angle/360); //거리계산
          Serial.println(userInput);
          stepstatus();

          horizontal1();
          horizontal2();
          break;

        case 'c':
          to_walking();
          break;
      }
    }

    else {
      switch (userInput) {
        case 'f':
          go_forward();
          break;

        case 'b':
          go_backward();
          break;

        case 'l':
          turn_left();
          break;

        case 'r':
          turn_right();
          break;

        case 'c':
          to_rolling();
          break;
      }
    }
  }
}

void forward() {
  for (;;) {
    stepper1.move(-1);
    stepper2.move(1);
    stepper1.setSpeed(stepspeed);
    stepper2.setSpeed(stepspeed);
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
    delay(10);
    userInput = bt.read();
    if (userInput == 'f' || userInput == 'b' || userInput == 'r' || userInput == 'l' || userInput == 's') {
      return;
    }
  }
}

void back() {
  for (;;) {
    stepper1.move(1);
    stepper2.move(-1);
    stepper1.setSpeed(stepspeed);
    stepper2.setSpeed(stepspeed);
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
    delay(10);
    userInput = bt.read();
    if (userInput == 'f' || userInput == 'b' || userInput == 'r' || userInput == 'l' || userInput == 's') {
      return;
    }
  }
}

void right() {
  for (;;) {
    stepper1.move(-1);
    stepper2.move(-1);
    stepper1.setSpeed(stepspeed);
    stepper2.setSpeed(stepspeed);
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
    delay(10);
    userInput = bt.read();
    if (userInput == 'f' || userInput == 'b' || userInput == 'r' || userInput == 'l' || userInput == 's') {
      return;
    }
  }
}

void left() {
  for (;;) {
    stepper1.move(1);
    stepper2.move(1);
    stepper1.setSpeed(stepspeed);
    stepper2.setSpeed(stepspeed);
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
    delay(10);
    userInput = bt.read();
    if (userInput == 'f' || userInput == 'b' || userInput == 'r' || userInput == 'l' || userInput == 's') {
      return;
    }
  }
}

void horizontal1() {
  if (Angle1 >= 0) {
    if (Angle1 >= 180) {
      pulse = ((360 - Angle1) * microcontrol) / stepangle;
      stepper1.setCurrentPosition(0);
      for (;;) {
        stepper1.move(1);
        stepper1.setSpeed(stepspeed);
        stepper1.runSpeedToPosition();
        delay(10);
        if (stepper1.currentPosition() == pulse) {
          stepper1.stop();
          stepper1.disableOutputs();
          stepper1.setCurrentPosition(0);
          break;
        }
      }
    }
    else if (Angle1 < 180) {
      pulse = ((0 - Angle1) * microcontrol) / stepangle;
      stepper1.setCurrentPosition(0);
      for (;;) {
        stepper1.move(-1);
        stepper1.setSpeed(stepspeed);
        stepper1.runSpeedToPosition();
        delay(10);
        if (stepper1.currentPosition() == pulse) {
          stepper1.stop();
          stepper1.disableOutputs();
          stepper1.setCurrentPosition(0);
          break;
        }
      }
    }
  }
  else if (Angle1 < 0) {
    if (Angle1 <= -180) {
      pulse = ((abs(Angle1) - 360) * microcontrol) / stepangle;
      stepper1.setCurrentPosition(0);
      for (;;) {
        stepper1.move(-1);
        stepper1.setSpeed(stepspeed);
        stepper1.runSpeedToPosition();
        delay(10);
        if (stepper1.currentPosition() == pulse) {
          stepper1.stop();
          stepper1.disableOutputs();
          stepper1.setCurrentPosition(0);
          break;
        }
      }
    }
    else if (Angle1 > -180) {
      pulse = ((abs(Angle1) - 0) * microcontrol) / stepangle;
      stepper1.setCurrentPosition(0);
      for (;;) {
        stepper1.move(1);
        stepper1.setSpeed(stepspeed);
        stepper1.runSpeedToPosition();
        delay(10);
        if (stepper1.currentPosition() == pulse) {
          stepper1.stop();
          stepper1.disableOutputs();
          stepper1.setCurrentPosition(0);
          break;
        }
      }
    }
  }
}

void horizontal2() {
  if (Angle2 >= 0) {
    if (Angle2 >= 180) {
      pulse = ((360 - Angle2) * microcontrol) / stepangle;
      stepper2.setCurrentPosition(0);
      for (;;) {
        stepper2.move(1);
        stepper2.setSpeed(stepspeed);
        stepper2.runSpeedToPosition();
        delay(10);
        if (stepper2.currentPosition() == pulse) {
          stepper2.stop();
          stepper2.disableOutputs();
          stepper2.setCurrentPosition(0);
          break;
        }
      }
    }
    else if (Angle2 < 180) {
      pulse = ((0 - Angle2) * microcontrol) / stepangle;
      stepper2.setCurrentPosition(0);
      for (;;) {
        stepper2.move(-1);
        stepper2.setSpeed(stepspeed);
        stepper2.runSpeedToPosition();
        delay(10);
        if (stepper2.currentPosition() == pulse) {
          stepper2.stop();
          stepper2.disableOutputs();
          stepper2.setCurrentPosition(0);
          break;
        }
      }
    }
  }
  else if (Angle2 < 0) {
    if (Angle2 <= -180) {
      pulse = ((360 + Angle2) * microcontrol) / stepangle;
      stepper2.setCurrentPosition(0);
      for (;;) {
        stepper2.move(-1);
        stepper2.setSpeed(stepspeed);
        stepper2.runSpeedToPosition();
        delay(10);
        if (abs(stepper2.currentPosition()) == pulse) {
          stepper2.stop();
          stepper2.disableOutputs();
          stepper2.setCurrentPosition(0);
          break;
        }
      }
    }
    else if (Angle2 > -180) {
      pulse = ((0 + Angle2) * microcontrol) / stepangle;
      stepper2.setCurrentPosition(0);
      for (;;) {
        stepper2.move(1);
        stepper2.setSpeed(stepspeed);
        stepper2.runSpeedToPosition();
        delay(10);
        if (stepper2.currentPosition() == abs(pulse)) {
          stepper2.stop();
          stepper2.disableOutputs();
          stepper2.setCurrentPosition(0);
          break;
        }
      }
    }
  }
}

void stepstatus() {
  motorposition1 = stepper1.currentPosition();
  motorposition2 = stepper2.currentPosition();

  Angle1 = (motorposition1) * stepangle / microcontrol; //각도 계산
  Angle2 = (motorposition2) * stepangle / microcontrol;

  if (Angle1 > 360) {
    for (Angle1; Angle1 >= 360;) {
      Angle1 -= 360;
      if (Angle1 < 360)
        break;
    }
  }

  else if (Angle1 < -360) {
    for (Angle1; Angle1 <= -360;) {
      Angle1 += 360;
      if (Angle1 > -360)
        break;
    }
  }

  if (Angle2 > 360) {
    for (Angle2; Angle2 >= 360;) {
      Angle2 -= 360;
      if (Angle2 < 360)
        break;
    }
  }

  else if (Angle2 < -360) {
    for (Angle2; Angle2 <= -360;) {
      Angle2 += 360;
      if (Angle2 > -360)
        break;
    }
  }
  Serial.print("stepper1 = "); Serial.print(motorposition1);
  Serial.print("  ||  stepper1angle = "); Serial.println(Angle1);
  //Serial.print("  ||  stepper2distance2 = "); Serial.println(Distance2);

  Serial.print("stepper2 = "); Serial.print(motorposition2);
  Serial.print("  ||  stepper2angle = "); Serial.println(Angle2);
  //Serial.print("  ||  stepper2distance2 = "); Serial.println(Distance2);
}

void go_forward() {
  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 + x22;
    allservo();

    Servoinput[5] = y02 + y22;
    Servoinput[9] = z02 - z22;
    allservo();

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 + x33;
    allservo();

    Servoinput[6] = y03 + y33;
    Servoinput[10] = z03 - z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 - x22;
    allservo();

    Servoinput[6] = y03 - y22;
    Servoinput[10] = z03 + z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 - x33;
    allservo();

    Servoinput[5] = y02 - y33;
    Servoinput[9] = z02 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[0] = x01 - x22;
    Servoinput[1] = x02;
    Servoinput[2] = x03;
    Servoinput[3] = x04 - x33;
    Servoinput[4] = y01 - y22;
    Servoinput[5] = y02;
    Servoinput[6] = y03;
    Servoinput[7] = y04 - y33;
    Servoinput[8] = z01 + z22;
    Servoinput[9] = z02;
    Servoinput[10] = z03;
    Servoinput[11] = z04 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04 + x22;
    allservo();

    Servoinput[7] = y04 + y22;
    Servoinput[11] = z04 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 + x33;
    allservo();

    Servoinput[4] = y01 + y33;
    Servoinput[8] = z01 - z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[0] = x01;
    Servoinput[1] = x02 + x22;
    Servoinput[2] = x03 + x33;
    Servoinput[3] = x04;
    Servoinput[4] = y01;
    Servoinput[5] = y02 + y22;
    Servoinput[6] = y03 + y33;
    Servoinput[7] = y04;
    Servoinput[8] = z01;
    Servoinput[9] = z02 - z22;
    Servoinput[10] = z03 - z33;
    Servoinput[11] = z04;
    allservo();
  }
}

void go_backward() {
  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 - x22;
    allservo();

    Servoinput[6] = y03 - y22;
    Servoinput[10] = z03 + z22;
    allservo();

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 - x33;
    allservo();

    Servoinput[4] = y02 - y33;
    Servoinput[8] = z02 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 + x22;
    allservo();

    Servoinput[5] = y02 + y22;
    Servoinput[9] = z02 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 + x33;
    allservo();

    Servoinput[6] = y03 + y33;
    Servoinput[10] = z03 - z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[0] = x01 + x33;
    Servoinput[1] = x02;
    Servoinput[2] = x03;
    Servoinput[3] = x04 + x22;
    Servoinput[4] = y01 + y33;
    Servoinput[5] = y02;
    Servoinput[6] = y03;
    Servoinput[7] = y04 + y22;
    Servoinput[8] = z01 - z33;
    Servoinput[9] = z02;
    Servoinput[10] = z03;
    Servoinput[11] = z04 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 - x22;
    allservo();

    Servoinput[4] = y01 - y22;
    Servoinput[8] = z01 + z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04 - x33;
    allservo();

    Servoinput[7] = y04 - y33;
    Servoinput[11] = z04 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[0] = x01;
    Servoinput[1] = x02 - x33;
    Servoinput[2] = x03 - x22;
    Servoinput[3] = x04;
    Servoinput[4] = y01;
    Servoinput[5] = y02 - y33;
    Servoinput[6] = y03 - y22;
    Servoinput[7] = y04;
    Servoinput[8] = z01;
    Servoinput[9] = z02 + z33;
    Servoinput[10] = z03 + z22;
    Servoinput[11] = z04;
    allservo();
  }
}

void turn_left() {
  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 + x22;
    allservo();

    Servoinput[5] = y02 + y22;
    Servoinput[9] = z02 - z22;
    allservo();

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 + x33;
    allservo();

    Servoinput[6] = y03 + y33;
    Servoinput[10] = z03 - z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 - x22;
    allservo();

    Servoinput[6] = y03 - y22;
    Servoinput[10] = z03 + z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[0] = x01 + x33;
    Servoinput[1] = x02 + x22;
    Servoinput[2] = x03;
    Servoinput[3] = x04 + x22;
    Servoinput[4] = y01 + y33;
    Servoinput[5] = y02 + y11;
    Servoinput[6] = y03;
    Servoinput[7] = y04 + y22;
    Servoinput[8] = z01 - z33;
    Servoinput[9] = z02 - z11;
    Servoinput[10] = z03;
    Servoinput[11] = z04 - z22;
    allservo();

    Servoinput[1] = x02;
    allservo();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 - x22;
    allservo();

    Servoinput[4] = y01 - y22;
    Servoinput[8] = z01 + z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[0] = x01;
    Servoinput[1] = x02 + x22;
    Servoinput[2] = x03 + x33;
    Servoinput[3] = x04 + x22;
    Servoinput[4] = y01;
    Servoinput[5] = y02 + y22;
    Servoinput[6] = y03 + y33;
    Servoinput[7] = y04 + y11;
    Servoinput[8] = z01;
    Servoinput[9] = z02 - z22;
    Servoinput[10] = z03 - z33;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04;
    allservo();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 + x22;
    allservo();

    Servoinput[5] = y02 + y22;
    Servoinput[9] = z02 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04 + x22;
    allservo();

    Servoinput[7] = y04 + y22;
    Servoinput[11] = z04 - z22;
    allservo();
  }
}

void turn_right() {
  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 - x22;
    allservo();

    Servoinput[6] = y03 - y22;
    Servoinput[10] = z03 + z22;
    allservo();

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 - x33;
    allservo();

    Servoinput[4] = y02 - y33;
    Servoinput[8] = z02 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 + x22;
    allservo();

    Servoinput[5] = y02 + y22;
    Servoinput[9] = z02 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[0] = x01 - x22;
    Servoinput[1] = x02;
    Servoinput[2] = x03 - x22;
    Servoinput[3] = x04 - x33;
    Servoinput[4] = y01 - y22;
    Servoinput[5] = y02;
    Servoinput[6] = y03 - y11;
    Servoinput[7] = y04 - y33;
    Servoinput[8] = z01 + z22;
    Servoinput[9] = z02;
    Servoinput[10] = z03 + z11;
    Servoinput[11] = z04 + z33;
    allservo();

    Servoinput[2] = x03;
    allservo();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04 + x22;
    allservo();

    Servoinput[7] = y04 + y22;
    Servoinput[11] = z04 - z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 - x22;
    Servoinput[1] = x02 - x33;
    Servoinput[2] = x03 - x22;
    Servoinput[3] = x04;
    Servoinput[4] = y01 - y11;
    Servoinput[5] = y02 - y33;
    Servoinput[6] = y03 - y22;
    Servoinput[7] = y04;
    Servoinput[8] = z01 + z11;
    Servoinput[9] = z02 + z33;
    Servoinput[10] = z03 + z22;
    Servoinput[11] = z04;
    allservo();

    Servoinput[0] = x01;
    allservo();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03 - x22;
    allservo();

    Servoinput[6] = y03 - y22;
    Servoinput[10] = z03 + z22;
    allservo();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 - x22;
    allservo();

    Servoinput[4] = y01 - y22;
    Servoinput[8] = z01 + z22;
    allservo();
  }
}

void to_rolling() {
  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {
//3번다리 1자로변형
    Servoinput[6] = y03 - b11;
    Servoinput[10] = z03 + c11;
    allservo();

    Servoinput[2] = x03 + a33;
    allservo();

    Servoinput[6] = y03 + b33;
    Servoinput[10] = z03 - c33;
    allservo();
//1번다리 1자로변형
    Servoinput[4] = y01 - b11;
    Servoinput[8] = z01 + c11;
    allservo();

    Servoinput[0] = x01 + a33;
    allservo();

    Servoinput[4] = y01 + b33;
    Servoinput[8] = z01 - c33;
    allservo();
//4번다리 1자로변형
    Servoinput[7] = y04 + b11;
    Servoinput[11] = z04 - c11;
    allservo();

    Servoinput[3] = x04 - a33;
    allservo();

    Servoinput[7] = y04 - b33;
    Servoinput[11] = z04 + c33;
    allservo();
//2번다리 1자로변형
    Servoinput[5] = y02 + b11;
    Servoinput[9] = z02 - c11;
    allservo();

    Servoinput[1] = x02 - a33;
    allservo();

    Servoinput[5] = y02 - b33;
    Servoinput[9] = z02 + c33;
    allservo();
//1,3번 다리의 2,3관절 접음
    Servoinput[4] = y01 + b44;
    Servoinput[8] = z01 - c44;
    Servoinput[6] = y03 + b44;
    Servoinput[10] = z03 - c44;
    allservo();
//2,4번 다리의 2,3관절 올림
    Servoinput[5] = y02;
    Servoinput[9] = z02 + 15;
    Servoinput[7] = y04;
    Servoinput[11] = z04 + 15;
    allservo();
//2,4번 다리의 2관절 내림
    Servoinput[5] = y02 + 60;
    Servoinput[7] = y04 + 60;
    allservo();
//2,4번 다리의 3관절 조정
    Servoinput[9] = z02 + 15 - 75;
    Servoinput[11] = z04 + 15 - 75;
    allservo();
//2,4번 다리의 2관절 완전히 접힘
    Servoinput[5] = y02 + 60 + 105;
    Servoinput[7] = y04 + 60 + 105;
    allservo();
//1,3번 다리의 2관절 완전히 접힘
    Servoinput[4] = y01 + b44 + 35;
    Servoinput[6] = y03 + b44 + 35;
    allservo();
  }
//아래 else if는 다리를 모두 x01,x02,x03,x04로 만들어주는 작업
  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03;
    allservo();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo();

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02;
    allservo();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02 - x33;
    allservo();

    Servoinput[5] = y02 - y33;
    Servoinput[9] = z02 + z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 + y11;
    Servoinput[9] = z02 - z11;
    allservo();

    Servoinput[1] = x02;
    allservo();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo();

    Servoinput[6] = y03 - y11;
    Servoinput[10] = z03 + z11;
    allservo();

    Servoinput[2] = x03;
    allservo();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04;
    allservo();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo();

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01;
    allservo();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01 + x33;
    allservo();

    Servoinput[4] = y01 + y33;
    Servoinput[8] = z01 - z33;
    allservo();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo();

    Servoinput[0] = x01;
    allservo();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo();

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo();

    Servoinput[3] = x04;
    allservo();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo();
  }
}

void to_walking() {
  if (
    Servoinput[0] == x01 + a33 &&
    Servoinput[1] == x02 - a33 &&
    Servoinput[2] == x03 + a33 &&
    Servoinput[3] == x04 - a33 &&
    Servoinput[4] == y01 + b44 + 35 &&
    Servoinput[5] == y02 + 60 + 105 &&
    Servoinput[6] == y03 + b44 + 35 &&
    Servoinput[7] == y04 + 60 + 105 &&
    Servoinput[8] == z01 - c44 &&
    Servoinput[9] == z02 + 15 - 75 &&
    Servoinput[10] == z03 - c44 &&
    Servoinput[11] == z04 + 15 - 75) {
//1,3번 다리의 2관절 내림
    Servoinput[4] = y01 + b44;
    Servoinput[6] = y03 + b44;
    allservo();
//2,4번 다리의 2관절 올림
    Servoinput[5] = y02 + 60 + 80;
    Servoinput[7] = y04 + 60 + 80;
    allservo();
//2,4번 다리의 3관절 내림
    Servoinput[9] = z02 + 15 - 75 + 30;
    Servoinput[11] = z04 + 15 - 75 + 30;
    allservo();
//2,4번 다리의 2,3관절 땅에 닿음
    Servoinput[5] = y02 + 60 + 20;
    Servoinput[7] = y04 + 60 + 20;
    Servoinput[9] = z02 + 15 - 75;
    Servoinput[11] = z04 + 15 - 75;
    allservo();
//2,4번 다리의 3관절 조정
    Servoinput[9] = z02 + 15;
    Servoinput[11] = z04 + 15;
    allservo();
//모든 다리의 2,3관절 조정->이때 모든 다리가 펼쳐짐
    Servoinput[4] == y01 + b33;
    Servoinput[5] == y02 - b33;
    Servoinput[6] == y03 + b33;
    Servoinput[7] == y04 - b33;
    Servoinput[8] == z01 - c33;
    Servoinput[9] == z02 + c33;
    Servoinput[10] == z03 - c33;
    Servoinput[11] == z04 + c33;
    allservo();
//3번다리 X03으로
    Servoinput[6] = y03 - b11;
    Servoinput[10] = z03 + c11;
    allservo();

    Servoinput[2] = x03;
    allservo();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo();
//1번다리 X01으로
    Servoinput[4] = y01 - b11;
    Servoinput[8] = z01 + c11;
    allservo();

    Servoinput[0] = x01;
    allservo();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo();
//4번다리 X04으로
    Servoinput[7] = y04 + b11;
    Servoinput[11] = z04 - c11;
    allservo();

    Servoinput[3] = x04;
    allservo();
//2번다리 X02으로
    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo();

    Servoinput[5] = y02 + b11;
    Servoinput[9] = z02 - c11;
    allservo();

    Servoinput[1] = x02;
    allservo();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo();
  }
}

void allservo() {
  Servo0.startEaseToD(Servoinput[0], Millis);
  Servo1.startEaseToD(Servoinput[1], Millis);
  Servo2.startEaseToD(Servoinput[2], Millis);
  Servo3.startEaseToD(Servoinput[3], Millis);
  Servo4.startEaseToD(Servoinput[4], Millis);
  Servo5.startEaseToD(Servoinput[5], Millis);
  Servo6.startEaseToD(Servoinput[6], Millis);
  Servo7.startEaseToD(Servoinput[7], Millis);
  Servo8.startEaseToD(Servoinput[8], Millis);
  Servo9.startEaseToD(Servoinput[9], Millis);
  Servo10.startEaseToD(Servoinput[10], Millis);
  Servo11.startEaseToD(Servoinput[11], Millis);
  while (ServoEasing::areInterruptsActive()) {
    ; // no delays here to avoid break between forth and back movement
  }
}
