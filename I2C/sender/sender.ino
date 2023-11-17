#include <PS3BT.h>                                         // Including all the required librabries
#include <usbhub.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

EasyTransferI2C ET;

int16_t S1,S2,S3,S4;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
 int16_t i1;;
 int16_t i2;
 int16_t i3;
 int16_t i4;
};
//give a name to the group of data
SEND_DATA_STRUCTURE mydata;
//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

USB Usb;                                                   //USBHub Hub1(&Usb); // Some dongles have a hub inside


BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so        /* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ET.begin(details(mydata), &Wire);
  Serial.begin(9600);
  #if !defined(_MIPSEL_)
  while (!Serial);                               // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt    
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}
 
void loop() {
  //delay(100);
  Usb.Task();

  int16_t S1 = 0;                              // Initializing all the 4 motors
  int16_t S2 = 0;
  int16_t S3 = 0;
  int16_t S4 = 0;

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {

      int x = 2*((PS3.getAnalogHat(LeftHatX)-130));                   // Speed in x Direction
      int y = 2*(129 - (PS3.getAnalogHat(LeftHatY)));                 // Speed in y Direction
       
      if(x<-255)                                                      // Setting up the controller range
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

      
      int s1 = -(-0.7071*x + 0.7071*y)*0.7352;                      //Calculating the speeds of all 4 motors
      int s2 = (-0.7071*x - 0.7071*y)*0.7352;
      int s3 = (0.7071*x - 0.7071*y)*0.7352;
      int s4 = (0.7071*x + 0.7071*y)*0.7352;     

     
      
      S1=map(s1, -255, 255, -510, 510);                             //Mapping the Range of PWM in the Range of RPM
      S2=map(s2, -255, 255, -510, 510);
      S3=map(s3, -255, 255, -510, 510);
      S4=map(s4, -255, 255, -510, 510);
    
      mydata.i1=S1;
      mydata.i2=S2;
      mydata.i3=S3;
      mydata.i4=S4;
      
      //send the data
     Serial.println(S1);
     Serial.println(S2);
     Serial.println(S3);
     Serial.println(S4);
     Serial.print("sending");
     ET.sendData(I2C_SLAVE_ADDRESS);
                                           // Assigning Motor Drivers to all the 4 Motors
      // ST.motor(2, pid(S1, M1_m2));                                     // Calling pid function and giving it to the motor driver
      
      // ST.motor(1, pid(S2, M1_m1)); 
      
      // ST1.motor(2, pid(S3, M2_m2));
      
      // ST1.motor(1, pid(S4, M2_m1));
    
 
  } 
// else
//   { Serial.print("sending");
//       mydata.i1=0;
//       mydata.i2=0;
//       mydata.i3=0;
//       mydata.i4=0;
//      ET.sendData(I2C_SLAVE_ADDRESS);
//   }
}
}

