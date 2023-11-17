#include <PS3BT.h>
#include <usbhub.h>
#include<Sabertooth.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial SWSerial(NOT_A_PIN, 10);
Sabertooth ST1(128, SWSerial);
Sabertooth ST2(129, SWSerial);
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

// Sint S1,S2,S3,S4;



USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;
String pass;
void setup()
 {
  //  SabertoothTXPinSerial.begin(115200);
  SWSerial.begin(9600);
  ST1.autobaud();
  ST2.autobaud();
  // ST1.setBaudRate(115200);
  // ST2.setBaudRate(115200);
  // SWSerial.end();


  // SWSerial.begin(115200);


  
  //Serial.begin(115200);
Serial.begin(9600);
#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  Wire.begin();                // join i2c bus with address #8
   // register event
             // start serial for output
}

void loop() 
{
 Usb.Task();
  
  int S1 = 0;
  int S2 = 0;
  int S3 = 0;
  int S4 = 0;
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) { 
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
      // Serial.print(s1);
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
      Serial.print(S4);
    
    }

    if(PS3.getButtonPress(CIRCLE))
    {
         S4 = -100;
         S3 = 100;
         S2 = -100;
         S1 = 100;
        
    }

     if(PS3.getButtonPress(TRIANGLE))
     {
         S4 = 100;
         S3 = -100;
         S2 = 100;
         S1 = -100;
         
     }
  }
  pass=String(S1)+" "+String(S2)+" "+String(S3)+" "+String(S4);

  // x[0]=(char)S1;
  // x[1]=(char)S2;
  // x[2]=(char)S3;
  // x[3]=(char)S4;
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(pass.c_str()); 

  // for(int i=0;i<pass.length();i++)
  // Serial.print(pass[i]);
  // Serial.println();
  //Wire.write(x[i]); 
  // Serial.print(x[1]);
  // Serial.print(x[2]);
  // Serial.print(x[3]);            // sends one byte
  Wire.endTransmission();

}

