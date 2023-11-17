#include <Sabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);  //2nd motor driver



#define ENC2_A 19              //define encoder pins 
#define ENC2_B 24
#define ENC1_A 21
#define ENC1_B 28


volatile int pos_1 = 0;
volatile float velocity_1 = 0;
volatile long prevT_1 = 0;
volatile int pos_2 = 0;
volatile float velocity_2 = 0;
volatile long prevT_2 = 0;


float v1Filt = 0;
float v1Prev = 0;
float deltaT1 = 0;
long currT1 = 0;
float v2Filt = 0;
float v2Prev = 0;
float deltaT2 = 0;
long currT2 = 0;

float eintegral1 = 0;
float eintegral2 = 0;





void setup() {
  SWSerial.begin(9600);
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);
  SWSerial.end();
  SWSerial.begin(115200);
  
  Serial.begin(115200);


  pinMode(ENC1_A,INPUT);
  pinMode(ENC1_B,INPUT);
  pinMode(ENC2_A,INPUT);
  pinMode(ENC2_B,INPUT);


  attachInterrupt(digitalPinToInterrupt(ENC2_A),readEncoder1,RISING);
  

}




void loop(){

  int pos1 = 0;
  float velocity1 = 0;
  int pos2 = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos1 = pos_1;
  velocity1 = velocity_1;
  pos1 = pos_1;
  velocity2 = velocity_2;
  interrupts();

  float v1 = velocity1/324.0*60.0;
  float v2 = velocity1/324.0*60.0;


  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev; 
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev; 
  v2Prev = v2;

  float vt = 100;
  

  
  
  
  
  
  
  float kp = 4.5;
  float ki = 0.3;
  float kd = 95;




  float e1 = vt-v1Filt;
  eintegral1 = eintegral1 + e1*deltaT1;
  float u1 = kp*e1 + ki*eintegral1 - kd*e1*deltaT1;

  float e2 = vt-v2Filt;
  eintegral2 = eintegral2 + e2*deltaT2;
  float u2 = kp*e2 + ki*eintegral2 - kd*e2*deltaT2;

  if (u1 > 127)
    u1 = 127;
  if (u1 < -127)
    u1 = -127;

  if (u2 > 127)
    u2 = 127;
  if (u2 < -127)
    u2 = -127;



    ST1.motor(2, u1);
    ST1.motor(1, u2);

    Serial.print(vt);
  Serial.print(" ");
  Serial.print(v2Filt);
 Serial.println();
//  Serial.print(v2Filt);
//  Serial.println();
  delay(1);
  
}


void readEncoder1(){
  int b2 = digitalRead(ENC1_B);
  int b1 = digitalRead(ENC2_B);
  int increment1 = 0;
  if(b1>0){
    // If B is high, increment forward
    increment1 = 1;
  }
  else{
    // Otherwise, increment backward
    increment1 = -1;
  }
  pos_1 = pos_1 + increment1;


  int increment2 = 0;
  if(b2>0){
    // If B is high, increment forward
    increment2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment2 = -1;
  }
  pos_2 = pos_2 + increment2;

  // Compute velocity with method 2
  long currT1 = micros();
  deltaT1 = ((float) (currT1 - prevT_1))/1.0e6;
  velocity_1 = increment1/deltaT1;
  prevT_1 = currT1;


  long currT2 = micros();
  deltaT2 = ((float) (currT2 - prevT_2))/1.0e6;
  velocity_2 = increment2/deltaT2;
  prevT_2 = currT2;
}