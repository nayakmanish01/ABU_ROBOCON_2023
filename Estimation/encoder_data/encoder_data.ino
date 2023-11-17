#include <Encoder.h>
#include<math.h>

Encoder knobLeft(2, 7);
Encoder knobRight(3, 8);
//   avoid using pins with LEDs attached
float turns_x,turns_y;
void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

long positionLeft_x  = -999;
long positionRight_y = -999;
void loop() {
  long total_x,total_y,old_x,old_y;
  total_x = knobLeft.read();
  turns_x=(total_x-old_x)/4010;
  total_y = knobRight.read();
  turns_y=(total_y-old_y)/4010;
  old_x=total_x;
  old_y=total_y;
  Serial.print("Left_x = ");
  Serial.print(turns_y);
  Serial.print("right_y = ");
  Serial.print(turns_x);
    

  }