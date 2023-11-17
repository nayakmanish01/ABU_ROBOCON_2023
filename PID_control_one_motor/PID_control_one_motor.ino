#include "CytronMotorDriver.h"

#define ENCA 19
#define ENCB 26
// #define PWM 3
// #define Dir 4

CytronMD motor(PWM_DIR, 3, 6); 

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
//float v2Filt = 0;
//float v2Prev = 0;
int i=0;
float error[10] = {0};


void setup() {
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  //pinMode(PWM,OUTPUT);
  // pinMode(IN1,OUTPUT);
  // pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  delay(500);
  // read the position and velocity
  int pos = 0;
  //float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  //velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  prevT_i = currT;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  // Convert count/s to RPM
  float v1 = velocity1/138*60.0;
  //float v2 = velocity2/144.0*60.0;
  // Low-pass filter (25 Hz cutoff)
  Serial.println();
  Serial.println(v1);
  //Serial.println(v1Filt);
  // v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  // v1Prev = v1;
  // v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  // v2Prev = v2;

  // Set a target
  float vt = 227;// set rpm

  // Compute the control signal u
  float kp = 0.25;
  float ki =0;
  float kd = 0;
  
  float e = vt-v1;
  //float e = vt-v1Filt;
  
   float eintegral = sum() + e*deltaT;
  float u = kp*e + ki*eintegral- kd*e*deltaT;
  //Serial.println(u);

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

  //Serial.println(vt);
  //Serial.println(" ");
  //Serial.println(v1);
  //Serial.println();
  // Serial.println(u);
  delay(1);
  if(i>9)
  i=0;

  error[i]=e;
  i++;
}
float sum()
{
  float esum=0;
  for(int i=0;i<10;i++)
  esum+=error[i];
return esum;
} 
void setMotor(int dir,int pwm)
{
  if(dir==-1)
  pwm=pwm*(-1);
  else
  pwm=pwm;

  motor.setSpeed(pwm);  // Run forward at 50% speed.
  // delay(1000);
}
// void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
//   analogWrite(pwm,pwmVal); // Motor speed
//   if(dir == 1){ 
//     // Turn one way
//     digitalWrite(in1,HIGH);
//     digitalWrite(in2,LOW);
//   }
//   else if(dir == -1){
//     // Turn the other way
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,HIGH);
//   }
//   else{
//     // Or dont turn
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,LOW);    
//   }
// }

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
 
}