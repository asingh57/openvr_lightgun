#include <Mouse.h>
int x=100,y;
char inputBuffer[16];
float mult_offset=0;
int x_sign=1;
int y_sign=1;

void setup() {
  // put your setup code here, to run once:
  //Mouse.begin();
  Serial.begin(9600);
  Serial.setTimeout(0);
  Mouse.begin();
  
  while(Serial.available() == 0){
    }
  while(Serial.available()>0){
      char i;
      i=Serial.read();
      }
  Mouse.move(x,0);
  while(Serial.available() == 0){
    }
  while(Serial.available()>0){
      Serial.readBytes(inputBuffer, sizeof(inputBuffer));//get how many pixels were moved
      mult_offset = 100.0/atof(inputBuffer);// MOUSE PRECISION MUST BE DISABLED FOR THIS TO WORK PROPERLY
      Serial.println(mult_offset);
      memset(inputBuffer, 0, sizeof(inputBuffer));
      }

  }

bool set_x=true;
char char_recv;
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() > 0){

    if(set_x){
        // A function that reads characters from the serial port into a buffer.
        Serial.readBytes(inputBuffer, sizeof(inputBuffer));
    
        // Convert string to integer
        // cplusplus.com/reference/cstdlib/atoi/
        x = atoi(inputBuffer)*mult_offset;
    
        // memset clears buffer and updates string length so strlen(inputBuffer) is accurate.
        memset(inputBuffer, 0, sizeof(inputBuffer));
        set_x=false;
      }
      else{
        // A function that reads characters from the serial port into a buffer.
        Serial.readBytes(inputBuffer, sizeof(inputBuffer));
    
        // Convert string to integer
        // cplusplus.com/reference/cstdlib/atoi/
        y = atoi(inputBuffer)*mult_offset;
    
        // memset clears buffer and updates string length so strlen(inputBuffer) is accurate.
        memset(inputBuffer, 0, sizeof(inputBuffer));
        set_x=true;
        Serial.print(x);
        Serial.print(" ");
        Serial.println(y);
        
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
    
  }
 
  
  delay(2);  
}
