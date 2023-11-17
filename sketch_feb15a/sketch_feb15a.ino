#include <Sabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);  //2nd motor driver

#define ENC1_A 21              //motor 1 
#define ENC1_B 28
#define ENC2_A 20             //motor 2
#define ENC2_B 26
#define ENC3_A 18          //motor 3
#define ENC3_B 22
#define ENC4_A 19      //motor 4
#define ENC4_B 24

volatile float velocity_i[4]={0};
volatile int pos_1 = 0,pos_2 = 0,pos_3 = 0,pos_4 = 0;

float posPrev=0;
float prevT_1=0,prevT_2,prevT_3=0,prevT_4=0;
float prevT=0,eintegral = 0,vt = 200;

float v1Filt[4]={0};
float v1Prev[4] = {0};


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
  pinMode(ENC3_A,INPUT);
  pinMode(ENC3_B,INPUT);
  pinMode(ENC4_A,INPUT);
  pinMode(ENC4_B,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_A),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3_A),readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4_A),readEncoder4,RISING);
}






void loop() {

  float pos=pos_1;
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  float v1 = velocity1/324.0*60.0;
  // Convert count/s to RPM
   v1Filt[0] = 0.854*v1Filt[0] + 0.0728*v1 + 0.0728*v1Prev[0];
  v1Prev[0] = v1;
// volatile float vel1=velocity1/324.0*60.0;//calcVel(velocity_1);
  float u[4];


// calcVel(c);
// u[0]=pid(v1Filt[0],0);
// u[1]=pid(v1Filt[1],1);
// u[2]=pid(v1Filt[2],2);
// u[3]=pid(v1Filt[3],3);

// ST.motor(1,u[0]); //motor 1
// ST.motor(2,u[1]); //motor 2
// ST1.motor(1,u[2]); // motor 3
// ST1.motor(2,u[3]); // motor 4
ST.motor(1,127); //motor 1
ST.motor(2,127); //motor 2
ST1.motor(1,127); // motor 3
ST1.motor(2,127); // motor 4

Serial.print(v1Filt[0]);
Serial.print(" ");
Serial.print(v1Filt[1]);
Serial.print(" ");
Serial.print(v1Filt[2]);
Serial.print(" ");
Serial.println(v1Filt[3]);


}



void readEncoder1(){
  // Read encoder 1 when ENC1_A rises
  int b = digitalRead(ENC1_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_1 = pos_1 + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_1))/1.0e6;
   velocity_i[0] = increment/(deltaT*324.0*60.0);
  prevT_1 = currT;
//  Serial.println(velocity_1);
  v1Filt[0] = 0.854*v1Filt[0] + 0.0728*velocity_i[0] + 0.0728*v1Prev[0]; 
  v1Prev[0] = velocity_i[0];
 
}


void readEncoder2(){
  // Read encoder 1 when ENC1_A rises
  int b = digitalRead(ENC2_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_2 = pos_2 + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_1))/1.0e6;
   velocity_i[1] = increment/(deltaT*324.0*60.0);
  prevT_1 = currT;
  v1Filt[1] = 0.854*v1Filt[1] + 0.0728*velocity_i[1] + 0.0728*v1Prev[1]; 
  v1Prev[1] = velocity_i[1];
 
}

void readEncoder3(){
  // Read encoder 1 when ENC1_A rises
  int b = digitalRead(ENC3_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_3 = pos_3 + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_1))/1.0e6;
   velocity_i[2] = increment/(deltaT*324.0*60.0);
  prevT_1 = currT;
  v1Filt[2] = 0.854*v1Filt[2] + 0.0728*velocity_i[2] + 0.0728*v1Prev[2]; 
  v1Prev[2] = velocity_i[2];
 
}

void readEncoder4(){
  // Read encoder 1 when ENC1_A rises
  int b = digitalRead(ENC4_B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_4 = pos_4 + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_1))/1.0e6;
  velocity_i[3] = increment/(deltaT*324.0*60.0);
  prevT_1 = currT;
  v1Filt[3] = 0.854*v1Filt[3] + 0.0728*velocity_i[3] + 0.0728*v1Prev[3]; 
  v1Prev[3] = velocity_i[3];
 
}




// void calcVel(int i){   // a-pos     //b- velocity calculated from encoder fn

//   float x = 0;
//   noInterrupts(); // disable interrupts temporarily while reading
  
//   x = velocity_i[i];
//   interrupts();   // turn interrupts back on
//   float v = x/324.0*60.0;

//   //filter
//   v1Filt[i] = 0.854*v1Filt[i] + 0.0728*v + 0.0728*v1Prev[i]; 
//   v1Prev[i] = v;
  
// }



float pid(float k,int i)
{
  Serial.print(k);
  Serial.println(i);
  float kp = 4.5;
  float ki = 0.3;
  float kd = 95;
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/1.0e6;
  prevT = currT;
  float e = vt-k;
  eintegral = eintegral + e*deltaT;
  
  
  float u = kp*e + ki*eintegral - kd*e*deltaT;
  // Serial.print(u);
  // Serial.print(" ");
  // Serial.println(i);

  if (u > 127)
    u = 127;
  if (u < -127)
    u = -127;
  
  return u;
  
} 