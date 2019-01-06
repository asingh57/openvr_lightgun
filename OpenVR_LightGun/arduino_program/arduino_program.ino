#include <Mouse.h>
String a;
int x;
int y;

void setup() {
  // put your setup code here, to run once:
  Mouse.begin();
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available()) {
  x= Serial.parseInt();// read the incoming data as string
  }
  if(Serial.available()) {
  y= Serial.parseInt();
  Mouse.move(x,y);
  }
  
}
