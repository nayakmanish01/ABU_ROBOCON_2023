/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>


SoftwareSerial SWSerial(NOT_A_PIN, 32 );
Sabertooth ST1(128, SWSerial);
Sabertooth ST2(129, SWSerial);

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define ENC1A 2
#define ENC1B 7//need to define pins of encoders
#define ENC2A 3
#define ENC2B 33
#define ENC3A 20
#define ENC3B 34
#define ENC4A 21
#define ENC4B 22

#define RPM_MAX 227


USB Usb; 
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

long prevT= 0;

int i=0;
int posPrev[4] = {0};
float v1Filt[4]= {0};
float v1Prev[4]= {0};
float eintegral[4] = {0};
//volatile int pos_i[3] = {0};
int pos_i[4] = {0};
float error1[10]={0};
float error2[10]={0};
float error3[10]={0};
float error4[10]={0};

float kp = 3.5;
float ki = 10;
float kd = 1500;


void setup() {
//  SabertoothTXPinSerial.begin(115200);
 SWSerial.begin(9600);

  // ST1.setBaudRate(115200);
  // ST2.setBaudRate(115200);
  // SWSerial.end();
     ST1.autobaud();
     ST2.autobaud();

  // SWSerial.begin(115200);0

  Serial.begin(9600);

  Serial.println("Before bluetooth!!");
#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  // pinMode(ENC1A,INPUT);
  // pinMode(ENC1B,INPUT);
  // pinMode(ENC2A,INPUT);
  // pinMode(ENC2B,INPUT);
  // pinMode(ENC3A,INPUT);
  // pinMode(ENC3B,INPUT);
  // pinMode(ENC4A,INPUT);
  // pinMode(ENC4B,INPUT);
  
  // //pinSituation();
  // Serial.println("Before Interrupt!!");
  // attachInterrupt(digitalPinToInterrupt(ENC1A),readEncoder1,RISING); 
  // attachInterrupt(digitalPinToInterrupt(ENC2A),readEncoder2,RISING); 
  // attachInterrupt(digitalPinToInterrupt(ENC3A),readEncoder3,RISING); 
  // attachInterrupt(digitalPinToInterrupt(ENC4A),readEncoder4,RISING);
  // Serial.println("After Interrupt!!");

}

void loop() {
  Serial.println("In side loop");

  Usb.Task();
  
  int S1 = 0;
  int S2 = 0; 
  int S3 = 0;
  int S4 = 0;

  // while(!(PS3.PS3Connected || PS3.PS3NavigationConnected)){
  //   Usb.Task();
  //     Serial.println("yes");
  // }
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) { 
    Serial.println("PS3 connected");
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
      int x = 2*((PS3.getAnalogHat(LeftHatX)-130));
      int y = 2*(129 - (PS3.getAnalogHat(LeftHatY))); 
      if(x<-255)
      {
        x = -255;
      }
      if(x>255)
      {
        x = 255;
      }
      if(y<-255)
      {
        y = -255;
      }
      if(y>255)
      {
        y = 255;
      }
      if(x<50 and x>-50)
      {
        x = 0;
      }
      if(y<50 and y>-50)
      {
        y = 0;
      }

      int s1 = (-0.7071*x + 0.7071*y)*0.7352;
      int s2 = (-0.7071*x - 0.7071*y)*0.7352;
      int s3 = (0.7071*x - 0.7071*y)*0.7352;
      int s4 = (0.7071*x + 0.7071*y)*0.7352;    
      
      // Serial.print(F("\r\ns1: "));
      // // Serial.print(s1);
      // Serial.print(F("\ts2: "));
      // Serial.print(s2);
      // Serial.print(F("\ts3: "));
      // Serial.print(s3);
      // Serial.print(F("\ts4: "));
      // Serial.print(s4);
      // Serial.print(F("\tx: "));
      // Serial.print(x);
      // Serial.print(F("\ty: "));
      // Serial.print(y);

      S1=map(s1, -255, 255, -127, 127);
      S2=map(s2, -255, 255, -127, 127);
      S3=map(s3, -255, 255, -127, 127);
      S4=map(s4, -255, 255, -127, 127);
      
      Serial.print(F("\r\nS1: "));
      Serial.print(S1);
      Serial.print(F("\r\nS2: "));
      Serial.print(S2);
      Serial.print(F("\r\nS3: "));
      Serial.print(S3); 
      Serial.print(F("\r\nS4: "));
      Serial.println(S4);

    
    }

    if(PS3.getButtonPress(CIRCLE)){
         S4 = -100;
         S3 = 100;
         S2 = -100;
         S1 = 100;
    }
      

    }
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1000000;
    prevT = currT;
    PID_control(S1,S2,S3,S4,deltaT);
    //Serial.println(deltaT);
}

void PID_control(int S1,int S2,int S3,int S4,float deltaT)
{ Serial.println();
  rpmcalc1(deltaT);
  Serial.println(v1Filt[0]);
  int v1=(v1Filt[0]*255)/RPM_MAX;
 int  V1=map(v1, -255, 255, -127, 127);
  float e1 = S1-V1;
 // Serial.println();
  float eintegral1 = sum(error1) + e1*deltaT;
  float u1 = kp*e1 + ki*eintegral1 - kd*e1/(deltaT);
 // Serial.println(u1);
  rpmcalc2(deltaT);
  Serial.println(v1Filt[1]);
  int v2=(v1Filt[1]*255)/RPM_MAX;
  int V2=map(v2, -255, 255, -127, 127);
  float e2 = S2-V2;
  float eintegral2 = sum(error2) + e2*deltaT;
  float u2 = kp*e2 + ki*eintegral2 - kd*e2/(deltaT);
 // Serial.println(u2);
  rpmcalc3(deltaT);
  Serial.println(v1Filt[2]);
  int v3=(v1Filt[2]*255)/RPM_MAX;
  int V3=map(v3, -255, 255, -127, 127);
  float e3 = S3-V3;
  float eintegral3 = sum(error3) + e3*deltaT;
  float u3 = kp*e3 + ki*eintegral3- kd*e3/(deltaT);
 // Serial.println(u3);
  rpmcalc4(deltaT);
  Serial.println(v1Filt[3]);
  int v4=(v1Filt[3]*255)/RPM_MAX;
  int V4=map(v4, -255, 255, -127, 127);
  float e4 = S4-V4;
  float eintegral4 = sum(error4) + e4*deltaT;
  float u4 = kp*e4 + ki*eintegral4 - kd*e4/(deltaT);
  //Serial.println(u4);
      
  
  
  // read datasheet and decide input impluse to feed

      ST1.motor(1, u2);  
      ST1.motor(2, u3);    //value of u_ should be integer -127 to 127
      ST2.motor(1, u4); 
      ST2.motor(2, u1); 

  if(i>9)
  i=0;  

  error1[i]=e1;
  error2[i]=e2;
  error3[i]=e3;
  error4[i]=e4;
  i++;
}
float sum(float error[10])
{ float esum=0;
  for(int i=0;i<10;i++)
  esum=esum+error[i];
  return esum;

}

void rpmcalc1(float deltaT)
{
  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i[0];
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  //Serial.println(pos_i[0]);
  float velocity1 = (pos - posPrev[0])/deltaT;
  posPrev[0] = pos;
  

  // Convert count/s to RPM
  float v1 = velocity1/144.0*60.0;
  //Serial.println(v1);
  // Low-pass filter (25 Hz cutoff)
  v1Filt[0] = 0.854*v1Filt[0] + 0.0728*v1 + 0.0728*v1Prev[0];
  v1Prev[0] = v1;
 
}

void rpmcalc2(float deltaT)
{
  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i[1];
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  //Serial.println(pos_i[1]);
  float velocity1 = (pos - posPrev[1])/deltaT;
  posPrev[1] = pos;
  

  // Convert count/s to RPM
  float v1 = velocity1/144.0*60.0;
  //Serial.println(v1);
  // Low-pass filter (25 Hz cutoff)
  v1Filt[1] = 0.854*v1Filt[1] + 0.0728*v1 + 0.0728*v1Prev[1];
  v1Prev[1] = v1;
 
}

void rpmcalc3(float deltaT)
{
  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i[2];
  interrupts(); // turn interrupts back on
  //Serial.println(pos_i[2]);
  // Compute velocity with method 1
  
  float velocity1 = (pos - posPrev[2])/deltaT;
  posPrev[2] = pos;
  

  // Convert count/s to RPM
  float v1 = velocity1/144.0*60.0;
  //Serial.println(v1);
  // Low-pass filter (25 Hz cutoff)
  v1Filt[2] = 0.854*v1Filt[2] + 0.0728*v1 + 0.0728*v1Prev[2];
  v1Prev[2] = v1;
  
}

void rpmcalc4(float deltaT)
{
  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i[3];
  interrupts(); // turn interrupts back on
  //Serial.println(pos_i[3]);
  // Compute velocity with method 1
  
  float velocity1 = (pos - posPrev[3])/deltaT;
  posPrev[3] = pos;
  

  // Convert count/s to RPM
  float v1 = velocity1/144.0*60.0;
  //Serial.println(v1);
  // Low-pass filter (25 Hz cutoff)
  v1Filt[3] = 0.854*v1Filt[3] + 0.0728*v1 + 0.0728*v1Prev[3];
  v1Prev[3] = v1;
 
}

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENC1B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
 pos_i[0] = pos_i[0] + increment;
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENC2B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[1] = pos_i[1] + increment;
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENC3B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[2] = pos_i[2] + increment;
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENC4B);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[3] = pos_i[3] + increment;
}
