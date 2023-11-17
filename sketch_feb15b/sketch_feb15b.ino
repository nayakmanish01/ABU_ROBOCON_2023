#include <Sabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);

#define ENCA 21
#define ENCB 28

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
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  SWSerial.begin(9600);
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);
  SWSerial.end();
  SWSerial.begin(115200);
  
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  // noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  // interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/324.0*60.0;
  float v2 = velocity2/324.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 200;

  // Compute the control signal u
  float kp = 4.5;
  float ki = 0.3;
  float kd = 95;
  
  float e = vt-v2Filt;
  eintegral = eintegral + e*deltaT;
  
  
  float u = kp*e + ki*eintegral - kd*e*deltaT;

  // Set the motor speed and direction
//  int dir = 1;
//  if (u<0){
//    dir = -1;
//  }
// 
  if (u > 127)
    u = 127;
  if (u < -127)
    u = -127;

  ST.motor(1, u);
  //setMotor(dir,pwr,PWM,IN1,IN2);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v2Filt);
  Serial.println();
  delay(1);
}

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
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
