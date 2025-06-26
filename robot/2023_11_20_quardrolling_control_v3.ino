//#include <RF24.h>
//#include <RF24_config.h>
//#include <nRF24L01.h>
//#include <printf.h>
//#include <SoftwareSerial.h>

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
const int Millis = 100;
const int Millis2 = 100;
const int SERVO_PIN[12] = {8, 0, 4, 12,
                           9, 1, 5, 13,
                           10, 2, 6, 14
                          };


int x01 = 82 - 45; //walking시작할때각도 FL
int x02 = 5 + 45;  //FR
int x03 = 170 - 45;  //BR
int x04 = 11 + 45;  //BL

int y01 = 35;
int y02 = 180;
int y03 = 120;
int y04 = 0;

int z01 = 25; //3 +
int z02 = 35; //19 -
int z03 = 141; //92 -
int z04 = 117; //91 +

int y11 = 30; //크립다리들때각도
int z11 = 30;

int x22 = 55; //크립다리짧게각도
int y22 = 0;
int z22 = 0;

int x33 = 25; //크립다리길게각도
int y33 = 0;
int z33 = 0;

int rx01 = 80; //rolling 시작할때각도
int rx02 = 5;
int rx03 = 163;
int rx04 = 11;

int ry01 = 162; //140 --> 162
int ry02 = 12;
int ry03 = 3;
int ry04 = 162;

int rz01 = 9; //10-->9
int rz02 = 146;
int rz03 = 160;
int rz04 = 9; //8-->9

int Servoinputlast[12];
int Servoinput[12] = {rx01, rx02, rx03, rx04,
                      ry01, ry02, ry03, ry04,
                      rz01, rz02, rz03, rz04,
                     };

char robotstate = 'r';
//----------------------------------------------------

// 위의 라이브러리는 필요없음
#define recvCH1 38  // THROTTLE(오른쪽 스틱 상하) 연결 
#define recvCH2 40  // AILE(오른쪽 스틱 좌우) 연결
#define recvCH3 42  // ELEV(왼쪽 스틱 상하) 연결
#define recvCH4 44  // RUDO(왼쪽 스틱 좌우) 연결 
#define recvCH5 46  // S1 switch와 연결(디지털1)
#define recvCH6 48 // S3 switch와 연결(디지털2)

//인터럽트 핀 연결 변수
const int encoderPinA = 18; //엔코더 인터럽트 핀 연결
const int encoderPinB = 19; //엔코더 인터럽트 핀 연결
const int encoderPinC = 2; //엔코더 인터럽트 핀 연결
const int encoderPinD = 3; //엔코더 인터럽트 핀 연결


#define ACT 4   // Linear Actuator enable signal output pin

//모터 쉴드 변수선언
int E1 = 5; // 모터 1 속도 제어
int M1 = 4; // 모터 1 방향 제어
int E2 = 6; // 모터 2 속도 제어
int M2 = 7; // 모터 2 방향 제어
int motorspeed2;
// 한바퀴 펄스값
long onepulse = 2200;

//수신기의 6개 채널 변수선언
long valueCH1;
long valueCH2;
long valueCH3;
long valueCH4;
long valueCH5;
long valueCH6;

//변신모드 준비 변수
int aim_value = 0;
int control_R;
int control_L;
int a;
int b;
unsigned long time_j;


int *g ;
//캘리브레이션 변수
long maxValue = 0;
long minValue = 0;
long sensorMin = 20000;
long sensorMax = 0;
long time_J;

long valueCH_1;
long valueCH2_L;
long valueCH2_R;
long valueCH3_B;
long valueCH3_F;
long valueCH_4;
long valueCH_5;
long valueCH_6;

// PID 속도 제어 변수
unsigned int count = 0;
float control_P ;
float control_i;
float control_D ;
long encoderPos_R ;
long encoderPos_L ;
long error;
long old_error;
long error_i;
float Pcontrol = 0.06;
float Icontrol = 0.005;
float Dcontrol = 0;
int motorspeed;

//펄스값 읽는 인터럽트 함수

//직진, 후진
void doencoderPinA() {

  encoderPos_R += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : 1;
}

void doencoderPinB() {

  encoderPos_R += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : 1;

}

void doencoderPinC() {

  encoderPos_L += (digitalRead(encoderPinC) == digitalRead(encoderPinD)) ? 1 : 1;

}

void doencoderPinD() {

  encoderPos_L += (digitalRead(encoderPinC) == digitalRead(encoderPinD)) ? 1 : 1;

}

//좌회전, 우회전
void doencoderPinE() {

  encoderPos_R += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
  Serial.println("R");
}
void doencoderPinF() {
  encoderPos_R += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;

}
void doencoderPinG() {

  encoderPos_L += (digitalRead(encoderPinC) == digitalRead(encoderPinD)) ? 1 : -1;

}
void doencoderPinH() {
  encoderPos_L += (digitalRead(encoderPinC) == digitalRead(encoderPinD)) ? -1 : 1;

}


void setup() {

  //Wire.begin(4);                // join i2c bus with address #4
  //Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);
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

  pinMode(29, OUTPUT);
  pinMode(27, OUTPUT);
  delay(1000);
  if (pulseIn(recvCH1, HIGH) >= 1700) {
    Serial.println(" ");
    Serial.println("스로틀을 최대로 올리세요");
    Serial.println("A");

    while (millis() < 5000) {
      digitalWrite(29, HIGH);
      maxValue = analogRead(valueCH1 = pulseIn(recvCH1, HIGH));
      Serial.println("B");
      if (valueCH1 > sensorMax) {
        sensorMax = valueCH1;
      }
    }
    Serial.println("스로틀을 최대로 내리세요");
    Serial.println("C");
    delay(2000);
    Serial.println("D");
    while (millis() < 10000) {
      digitalWrite(29, LOW);
      minValue = analogRead(valueCH1 = pulseIn(recvCH1, HIGH));
      Serial.println("E");
      if (valueCH1 < sensorMin) {
        sensorMin = valueCH1;
      }

    }

    Serial.println("F");
  }
  else if (pulseIn(recvCH1, HIGH) < 1700) {
    Serial.println("쓰로틀을 최대로 올리고 다시 실행하세요.");
    digitalWrite(27, HIGH);
  }
  pinMode(30, OUTPUT);
  digitalWrite(30, HIGH);
  pinMode(recvCH1, INPUT);
  pinMode(recvCH2, INPUT);
  pinMode(recvCH3, INPUT);
  pinMode(recvCH4, INPUT);
  pinMode(recvCH5, INPUT);
  pinMode(recvCH6, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(5, doencoderPinA, CHANGE);

  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(4, doencoderPinB, CHANGE);

  pinMode(encoderPinC, INPUT_PULLUP);
  attachInterrupt(1, doencoderPinC, CHANGE);

  pinMode(encoderPinD, INPUT_PULLUP);
  attachInterrupt(0, doencoderPinD, CHANGE);
  /*
    pinMode(encoderPinA, INPUT_PULLUP);
    attachInterrupt(5, doencoderPinE, CHANGE);

    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(4, doencoderPinF, CHANGE);

    pinMode(encoderPinC, INPUT_PULLUP);
    attachInterrupt(3, doencoderPinG, CHANGE);

    pinMode(encoderPinD, INPUT_PULLUP);
    attachInterrupt(2, doencoderPinH, CHANGE);
  */

  TCCR0A = 0; //TCCR0A initialize

  TCCR0B = 0; //TCCR0B initialize

  TCNT0 = 0;  //TCNT0 initialize

  OCR0A = 255;

  TCCR0B |= (1 << WGM02);

  TCCR0B |= (1 << CS02) | (0 << CS00);

  TIMSK0 |= (1 << OCIE0A);

  sei();



}


ISR(TIMER0_COMPA_vect) {

  count++;

  if (count > 255)

  {
    error = encoderPos_L - encoderPos_R;

    control_P = error * Pcontrol;

    error_i = error + error_i;

    control_i = error_i * Icontrol * 0.1;

    control_D = (old_error - error) * Dcontrol;

    old_error = error;

    motorspeed = control_P + control_i + control_D;
    count = 0;
    motorspeed = constrain(map(motorspeed, -50, 50, -50, 50), -50, 50) ;
  }

}
void loop() {
  /*Serial.print(encoderPos_R);
    Serial.print("/");
    Serial.print(encoderPos_L);
    Serial.print("/");
    Serial.print(error);
    Serial.print("/");
    Serial.print(motorspeed);
    Serial.print("/");
    Serial.print(motorspeed2);
    Serial.print("/");
    Serial.println(valueCH2);
  */






  noInterrupts();


  //Serial.print(encoderPos_R);
  //Serial.print("/");
  //Serial.print(encoderPos_L);
  //Serial.println("/");
  // HIGH일 때 duration의 길이를 정수값으로 변환
  valueCH1 = pulseIn(recvCH1, HIGH); // 1090~1880까지 연속적으로 변화
  valueCH2 = pulseIn(recvCH2, HIGH); // 1090~1880까지 연속적으로 변화
  valueCH3 = pulseIn(recvCH3, HIGH); // 1090~1880까지 연속적으로 변화
  valueCH4 = pulseIn(recvCH4, HIGH); // 1090~1880까지 연속적으로 변화
  valueCH5 = pulseIn(recvCH5, HIGH); // 1090, 1880 두가지 값으로 변화
  valueCH6 = pulseIn(recvCH6, HIGH); // 1090, 1880 두가지 값으로 변화
  //valueCH1 = constrain(map(valueCH1, sensorMin, sensorMax, 0, 255),0,255);
  valueCH2_L = constrain(map(valueCH2, (sensorMin + sensorMax) / 2, sensorMax, 0, 200), 0, 200);
  valueCH2_R = constrain(map(valueCH2, sensorMin, (sensorMin + sensorMax) / 2, 200, 0), 0, 200);
  valueCH3_F = constrain(map(valueCH3, (sensorMin + sensorMax) / 2, sensorMax, 0, 200), 0, 200);
  valueCH3_B = constrain(map(valueCH3, sensorMin, (sensorMin + sensorMax) / 2, 200, 0), 0, 200);
  valueCH4 = constrain(map(valueCH4, sensorMin, sensorMax, 0, 200), 0, 200);
  valueCH5 = constrain(map(valueCH5, sensorMin, (sensorMin + sensorMax) / 2, 0, 1), 0, 1);
  valueCH6 = constrain(map(valueCH6, (sensorMin + sensorMax) / 2, sensorMax, 1, 0), 0, 1);
  /*
    if (valueCH5 >1200){
    digitalWrite(7, HIGH);
    }
    if (valueCH5 < 1200){
    digitalWrite(7, LOW);
    }  ]
    if (valueCH6 >1200){
    digitalWrite(6, HIGH);
    }
    if (valueCH6 < 1200){
    digitalWrite(6, LOW);
    }
  */

  //Serial.println(motor_speed);
  //Serial.println(encoderPos);
  // print the values with serial monitor
  //Serial.print("CH1: "); Serial.print(valueCH1); Serial.print("\t");
  //Serial.print("CH2_L: "); Serial.print(valueCH2_L); Serial.print("\t");
  //Serial.print("CH2_R: "); Serial.print(valueCH2_R); Serial.print("\t");
  //Serial.print("CH3_F: "); Serial.print(valueCH3_F); Serial.print("\t");
  //Serial.print("CH3_B: "); Serial.print(valueCH3_B); Serial.print("\t");
  //Serial.print("CH4: "); Serial.print(valueCH4); Serial.print("\t");
  //Serial.print("CH5: "); Serial.print(valueCH5); Serial.print("\t");
  //Serial.print("CH6: "); Serial.println(valueCH6);

  interrupts();

    // AILE로 좌우 방향 조절
    if (valueCH2_L > 30) {
      if (robotstate == 'w') {
      }
      else {
        digitalWrite(M1, 1);
        digitalWrite(M2, 1);
        analogWrite(E1, valueCH2_L);
        analogWrite(E2, valueCH2_L  - motorspeed);
      }
    }
    
    else if (valueCH2_R > 30) {
      if (robotstate == 'w') {
      }
      else {
        digitalWrite(M1, 0);
        digitalWrite(M2, 0);
        analogWrite(E1, valueCH2_R);
        analogWrite(E2, valueCH2_R - motorspeed);
      }
    }

    // ELEV로 전진/후진 조절
    else if (valueCH3_F > 30) {
      if (robotstate == 'w') {
        go_forward();
      }
      else {
        digitalWrite(M1, 0);
        digitalWrite(M2, 1);
        analogWrite(E1, valueCH3_F);
        analogWrite(E2,  valueCH3_F - 3 - motorspeed);
      }
    }

    else if (valueCH3_B > 30) {
      if (robotstate == 'w') {
        go_backward();
      }
      else {
        digitalWrite(M1, 1);
        digitalWrite(M2, 0);
        analogWrite(E1, valueCH3_B - 2);
        analogWrite(E2, valueCH3_B - motorspeed);
      }
    }
    else if (valueCH3_B <= 30 && valueCH3_F <= 30 ) {
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
 

  if (valueCH6 == HIGH) {
    digitalWrite(30, HIGH);
  }
  else if (valueCH6 == LOW) {
    digitalWrite(30, LOW);
  }
  if (valueCH5 == HIGH) {
    delay(100);
    if (robotstate == 'w') to_rolling();
  }
  if (valueCH5 == LOW) {
    delay(100);
    if (robotstate == 'r') to_walking();
  }
  delay(100);
  encoderPos_R = 0;
  encoderPos_L = 0;
  delay(100);
}


void go_forward() {
  if (robotstate == 'w') {
    if (
      Servoinput[0] == x01 &&
      Servoinput[1] == x02 &&
      Servoinput[2] == x03 &&
      Servoinput[3] == x04) {

      Servoinput[5] = y02 - y11;
      Servoinput[9] = z02 + z11;
      allservo();

      Servoinput[1] = x02 + x22;
      allservo();

      Servoinput[5] = y02;
      Servoinput[9] = z02;
      allservo();

      Servoinput[6] = y03 + y11;
      Servoinput[10] = z03 - z11;
      allservo();

      Servoinput[2] = x03 + x33;
      allservo();

      Servoinput[6] = y03;
      Servoinput[10] = z03;
      allservo();
    }

    else if (
      Servoinput[0] == x01 &&
      Servoinput[1] == x02 + x22 &&
      Servoinput[2] == x03 + x33 &&
      Servoinput[3] == x04) {

      Servoinput[6] = y03 + y11;
      Servoinput[10] = z03 - z11;
      allservo();

      Servoinput[2] = x03 - x22;
      allservo();

      Servoinput[6] = y03;
      Servoinput[10] = z03;
      allservo();
    }

    else if (
      Servoinput[0] == x01 &&
      Servoinput[1] == x02 + x22 &&
      Servoinput[2] == x03 - x22 &&
      Servoinput[3] == x04) {

      Servoinput[5] = y02 - y11;
      Servoinput[9] = z02 + z11;
      allservo();

      Servoinput[1] = x02 - x33;
      allservo();

      Servoinput[5] = y02;
      Servoinput[9] = z02;
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
      Servoinput[4] = y01;
      Servoinput[5] = y02;
      Servoinput[6] = y03;
      Servoinput[7] = y04;
      Servoinput[8] = z01;
      Servoinput[9] = z02;
      Servoinput[10] = z03;
      Servoinput[11] = z04;
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

      Servoinput[7] = y04;
      Servoinput[11] = z04;
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

      Servoinput[4] = y01;
      Servoinput[8] = z01;
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
      Servoinput[5] = y02;
      Servoinput[6] = y03;
      Servoinput[7] = y04;
      Servoinput[8] = z01;
      Servoinput[9] = z02;
      Servoinput[10] = z03;
      Servoinput[11] = z04;
      allservo();
    }
  }
}

void go_backward() {

  if (robotstate == 'w') {
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

      Servoinput[6] = y03;
      Servoinput[10] = z03;
      allservo();

      Servoinput[5] = y02 - y11;
      Servoinput[9] = z02 + z11;
      allservo();

      Servoinput[1] = x02 - x33;
      allservo();

      Servoinput[5] = y02;
      Servoinput[9] = z02;
      allservo();
    }

    else if (
      Servoinput[0] == x01 &&
      Servoinput[1] == x02 - x33 &&
      Servoinput[2] == x03 - x22 &&
      Servoinput[3] == x04) {

      Servoinput[5] = y02 - y11;
      Servoinput[9] = z02 + z11;
      allservo();

      Servoinput[1] = x02 + x22;
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

      Servoinput[6] = y03 + y11;
      Servoinput[10] = z03 - z11;
      allservo();

      Servoinput[2] = x03 + x33;
      allservo();

      Servoinput[6] = y03;
      Servoinput[10] = z03;
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
      Servoinput[4] = y01;
      Servoinput[5] = y02;
      Servoinput[6] = y03;
      Servoinput[7] = y04;
      Servoinput[8] = z01;
      Servoinput[9] = z02;
      Servoinput[10] = z03;
      Servoinput[11] = z04;
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

      Servoinput[4] = y01;
      Servoinput[8] = z01;
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

      Servoinput[7] = y04;
      Servoinput[11] = z04;
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
      Servoinput[5] = y02;
      Servoinput[6] = y03;
      Servoinput[7] = y04;
      Servoinput[8] = z01;
      Servoinput[9] = z02;
      Servoinput[10] = z03;
      Servoinput[11] = z04;
      allservo();
    }
  }
}
void to_rolling() {


  if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04) {

    //------------------------------------
    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo2();

    Servoinput[0] = rx01;
    allservo2();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo2();

    Servoinput[5] = y02 - y11;
    Servoinput[9] = z02 + z11;
    allservo2();

    Servoinput[1] = rx02;
    allservo2();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo2();

    Servoinput[6] = y03 + y11;
    Servoinput[10] = z03 - z11;
    allservo2();

    Servoinput[2] = rx03;
    allservo2();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo2();

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo2();

    Servoinput[3] = rx04;
    allservo2();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo2();

    Servoinput[4] = ry01 + 145;
    Servoinput[6] = ry03 - 145;
    Servoinput[8] = rz01 - 90 + 30;
    Servoinput[10] = rz03 + 30;
    Servoinput[5] = ry02 - 115;
    Servoinput[7] = ry04 + 115;
    Servoinput[9] = rz02 + 30;
    Servoinput[11] = rz04 - 30;
    allservo2();

    Servoinput[5] = ry02 - 30;
    Servoinput[7] = ry04 + 30;
    Servoinput[9] = rz02 + 10;
    Servoinput[11] = rz04 - 10;
    allservo2();

    Servoinput[8] = rz01;
    Servoinput[10] = rz03;
    allservo2();

    Servoinput[4] = ry01;
    Servoinput[5] = ry02;
    Servoinput[6] = ry03;
    Servoinput[7] = ry04;
    Servoinput[9] = rz02;
    Servoinput[11] = rz04;
    allservo2();

    robotstate = 'r';
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 + x33 &&
    Servoinput[3] == x04) {

    Servoinput[6] = y03 + y11;
    Servoinput[10] = z03 - z11;
    allservo2();

    Servoinput[2] = x03;
    allservo2();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo2();

    Servoinput[5] = y02 - y11;
    Servoinput[9] = z02 + z11;
    allservo2();

    Servoinput[1] = x02;
    allservo2();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo2();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 + x22 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 - y11;
    Servoinput[9] = z02 + z11;
    allservo2();

    Servoinput[1] = x02 - x33;
    allservo2();

    Servoinput[5] = y02 - y33;
    Servoinput[9] = z02 + z33;
    allservo2();
  }

  else if (
    Servoinput[0] == x01 &&
    Servoinput[1] == x02 - x33 &&
    Servoinput[2] == x03 - x22 &&
    Servoinput[3] == x04) {

    Servoinput[5] = y02 - y11;
    Servoinput[9] = z02 + z11;
    allservo2();

    Servoinput[1] = x02;
    allservo2();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo2();

    Servoinput[6] = y03 + y11;
    Servoinput[10] = z03 - z11;
    allservo2();

    Servoinput[2] = x03;
    allservo2();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo2();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 - x33) {

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo2();

    Servoinput[3] = x04;
    allservo2();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo2();

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo2();

    Servoinput[0] = x01;
    allservo2();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo2();
  }

  else if (
    Servoinput[0] == x01 - x22 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo2();

    Servoinput[0] = x01 + x33;
    allservo2();

    Servoinput[4] = y01 + y33;
    Servoinput[8] = z01 - z33;
    allservo2();
  }

  else if (
    Servoinput[0] == x01 + x33 &&
    Servoinput[1] == x02 &&
    Servoinput[2] == x03 &&
    Servoinput[3] == x04 + x22) {

    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo2();

    Servoinput[0] = x01;
    allservo2();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo2();

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo2();

    Servoinput[3] = x04;
    allservo2();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo2();
  }

}

void to_walking() {


  if (
    Servoinput[0] == rx01 &&
    Servoinput[1] == rx02 &&
    Servoinput[2] == rx03 &&
    Servoinput[3] == rx04 &&
    Servoinput[4] == ry01 &&
    Servoinput[5] == ry02 &&
    Servoinput[6] == ry03 &&
    Servoinput[7] == ry04 &&
    Servoinput[8] == rz01 &&
    Servoinput[9] == rz02 &&
    Servoinput[10] == rz03 &&
    Servoinput[11] == rz04) {

    Servoinput[4] = ry01 - 150;
    Servoinput[6] = ry03 + 150;
    Servoinput[8] = rz01 + 30;
    Servoinput[10] = rz03 - 30;
    Servoinput[5] = ry02 + 50;
    Servoinput[7] = ry04 - 50;

    allservo2();

    Servoinput[9] = rz02 - 15;
    Servoinput[11] = rz04 + 15;
    allservo2();

    Servoinput[5] = ry02 + 150;
    Servoinput[7] = ry04 - 150;
    Servoinput[9] = rz02 - 30;
    Servoinput[11] = rz04 + 30;
    allservo2();

    Servoinput[4] = y01;
    Servoinput[5] = y02;
    Servoinput[6] = y03;
    Servoinput[7] = y04;
    Servoinput[8] = z01;
    Servoinput[9] = z02;
    Servoinput[10] = z03;
    Servoinput[11] = z04;
    allservo2();
    //------------------------------------
    Servoinput[4] = y01 - y11;
    Servoinput[8] = z01 + z11;
    allservo2();

    Servoinput[0] = x01;
    allservo2();

    Servoinput[4] = y01;
    Servoinput[8] = z01;
    allservo2();

    Servoinput[5] = y02 - y11;
    Servoinput[9] = z02 + z11;
    allservo2();

    Servoinput[1] = x02;
    allservo2();

    Servoinput[5] = y02;
    Servoinput[9] = z02;
    allservo2();

    Servoinput[6] = y03 + y11;
    Servoinput[10] = z03 - z11;
    allservo2();

    Servoinput[2] = x03;
    allservo2();

    Servoinput[6] = y03;
    Servoinput[10] = z03;
    allservo2();

    Servoinput[7] = y04 + y11;
    Servoinput[11] = z04 - z11;
    allservo2();

    Servoinput[3] = x04;
    allservo2();

    Servoinput[7] = y04;
    Servoinput[11] = z04;
    allservo2();

    robotstate = 'w';
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

void allservo2() {
  Servo0.startEaseToD(Servoinput[0], Millis2);
  Servo1.startEaseToD(Servoinput[1], Millis2);
  Servo2.startEaseToD(Servoinput[2], Millis2);
  Servo3.startEaseToD(Servoinput[3], Millis2);
  Servo4.startEaseToD(Servoinput[4], Millis2);
  Servo5.startEaseToD(Servoinput[5], Millis2);
  Servo6.startEaseToD(Servoinput[6], Millis2);
  Servo7.startEaseToD(Servoinput[7], Millis2);
  Servo8.startEaseToD(Servoinput[8], Millis2);
  Servo9.startEaseToD(Servoinput[9], Millis2);
  Servo10.startEaseToD(Servoinput[10], Millis2);
  Servo11.startEaseToD(Servoinput[11], Millis2);
  while (ServoEasing::areInterruptsActive()) {
    ; // no delays here to avoid break between forth and back movement
  }
}
