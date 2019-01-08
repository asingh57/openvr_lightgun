#include <Mouse.h>
int x=100,y;
String inString = "";
float mult_offset=0;
int x_sign=1;
int y_sign=1;

float read_int(){
  
  while(Serial.available() == 0){
    }
  while(Serial.available()>0){
      char inChar = Serial.read();
      inString += inChar;
      }

   float to_ret=inString.toFloat();
     inString = "";
   return to_ret;
  }

void setup() {
  // put your setup code here, to run once:
  //Mouse.begin();
  Serial.begin(9600);
  Serial.setTimeout(0);
  Mouse.begin();
  
  read_int();//block till signaled
  Mouse.move(x,0);
  mult_offset = 100.0/read_int();
  }

bool set_x=true;
char char_recv;
void loop() {
  // put your main code here, to run repeatedly:
  x=(int)(read_int()*mult_offset);
  y=(int)(read_int()*mult_offset);

  
          if(x<0){
          x_sign=-1;
          x=-x;
          }
         else{
          x_sign=1;
          }
         if(y<0){
          y_sign=-1;
          y=-y;
          }
         else{
          y_sign=1;
          }
          
        //only 127 units of input allowed at a time
        if(x>y){
          while(x>126){
            x=x-126;
            if(y>126){
            y=y-126;
            Mouse.move(x_sign*126,y_sign*126);  
              }
            else{
            Mouse.move(x_sign*126,y_sign*y);
            y=0;  
              }
            }
            Mouse.move(x_sign*x,y_sign*y);
          }
         else{
          while(y>126){
            y=y-126;
            if(x>126){
            x=x-126;
            Mouse.move(x_sign*126,y_sign*126);  
              }
            else{
            Mouse.move(x_sign*x,y_sign*126);
            x=0;  
              }
            }
            Mouse.move(x_sign*x,y_sign*y);
          }
        
        
        
}
