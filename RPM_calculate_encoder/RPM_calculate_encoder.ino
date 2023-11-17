#include <Encoder.h>
#include<math.h>
#include "CytronMotorDriver.h"

Encoder knobLeft(2, 7);

#define PWM 3
// #define IN1 6
// #define IN2 5
CytronMD motor(PWM_DIR, 3, 6); 

//Encoder knobRight(3, 8);
//   avoid using pins with LEDs attached
float rpm=0;
int prev=0;
float prevT=0;
float error[10]={0};
int i=0;
void setup() {
  Serial.begin(115200);
  pinMode(PWM,OUTPUT);
  // pinMode(IN1,OUTPUT);
  // pinMode(IN2,OUTPUT);
  Serial.println("TwoKnobs Encoder Test:");
}

// long positionLeft_x  = -999;
// long positionRight_y = -999;

float sum()
{float esum=0;
for(int i=0;i<10;i++)
esum+=error[i];
return esum;
}
void loop() {
  long total_x;
  total_x = knobLeft.read();
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
   //Serial.println(knobLeft.read());
  rpm=((total_x-prev))/(deltaT*540*60);
   prev=total_x;
   prevT=currT;
  Serial.println(total_x);
  float vt = 200; // set rpm

  // Compute the control signal u
  float kp = 3.5;
  float ki = 10;
  float kd = 1500;
  
  float e = vt-rpm;
  float eintegral = sum()+e*deltaT;
  
  
  float u = kp*e + ki*eintegral - kd*e*deltaT;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
   
  }
  setMotor(dir,pwr);

  //Serial.print(u);
  //Serial.print(" ");
  //Serial.print(rpm);
  //Serial.println();
  delay(1);

  error[i]=e;
    i++;
    if(i>9)
    i=0;
}

void setMotor(int dir,int pwm)
{
  if(dir==-1)
  pwm=pwm*(-1);
  else
  pwm=pwm;

  motor.setSpeed(pwm);  // Run forward at 50% speed.
  delay(1000);
}
  
