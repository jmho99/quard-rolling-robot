#include <SoftwareSerial.h>

#define  joystick_x  A0  
#define joystick_y  A1
#define joystick_z 8
#define Switch1 13

SoftwareSerial bt(2,3);

void setup() {
  Serial.begin(9600);
  bt.begin(38400);
  pinMode(joystick_z,INPUT_PULLUP);
  pinMode(Switch1,INPUT_PULLUP);
}

void loop() {

  int x = analogRead(joystick_x);
  delay(10);
  int y = analogRead(joystick_y); 
  delay(10);
  int z = digitalRead(joystick_z);   
  delay(10);
  int switch1 = digitalRead(Switch1);
  
 
  if(x>990&&y<=550&&y>=500){
    bt.write('f');
    }

  else if(x<30&&y<=550&&y>=500){
    bt.write('b');
    }

  else if(y>990&&x<=540&&x>=480){
    bt.write('r');
    }

  else if(y<30&&x<=540&&x>=480){
    bt.write('l');
    }
  if(z==0){
    bt.write('s');
    }
  if(switch1==0){
    bt.write('c');
  }

  Serial.print("X: ");                   
  Serial.print(x);                         
  Serial.print("  Y: ");
  Serial.print(y);                          
  Serial.print("  Z: ");
  Serial.print(z); 
  Serial.print("  switch: ");
  Serial.println(switch1);
   delay(300);                       
}
