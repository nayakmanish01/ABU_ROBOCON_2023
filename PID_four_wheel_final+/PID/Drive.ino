/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3BT.h>                                         // Including all the required librabries
#include <usbhub.h>
#include <Encoder.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>


#ifdef dobogusinclude                                      // Satisfy the IDE, which needs to see the include statment in the ino too.
#include <spi4teensy3.h>
#endif

SoftwareSerial SWSerial(NOT_A_PIN, 30 );                   // RX on no pin (unused), TX on pin 30
Sabertooth ST(128, SWSerial);                              // The Sabertooth is on address 128
Sabertooth ST1(129, SWSerial);                             // The Sabertooth is on address 129


int S1,S2,S3,S4;                                           //Declaring motor speeds for the 4 motors

#define ENC_COUNT_REV 138                                  // Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_IN_RIGHT_A 19 //right of sabertooth                                 //SW 1 128
#define ENC_IN_RIGHT_B 24 
#define ENC_IN_LEFT_A 18  //left of sabertooth
#define ENC_IN_LEFT_B 22
#define ENC1_IN_RIGHT_A 20  //right of sabertooth                               // SW 2 129
#define ENC1_IN_RIGHT_B 26
#define ENC1_IN_LEFT_A 21    //left of sabertooth
#define ENC1_IN_LEFT_B 28
#define qsize 10                                           // Defining the size of Queue

boolean Direction_right = true;                            //Defining boolean for direction in the encoder
boolean Direction_left = true;
boolean Direction_right1 = true;
boolean Direction_left1 = true;


volatile long right_wheel_pulse_count = 0;                 // Keep track of the number of right wheel pulses
volatile long left_wheel_pulse_count = 0;
volatile long right1_wheel_pulse_count = 0;
volatile long left1_wheel_pulse_count = 0;

 
int interval = 1000;                                       // One-second interval for measurements
  

long previousMillis = 0;                                    // Counters for milliseconds during interval
long currentMillis = 0;
 
float M1_m2 = 0;    //SW 1                                    // Variable for RPM measuerment
float M1_m1 = 0; 
float M2_m2 = 0;    //SW 2
float M2_m1 = 0;


// float kp=1, kd=0, ki=0;                                     // Values of Kp, Kd and Ki

float error=0,preverror=0;                                  // Globally defining variables for the error functions
float errorFunc;
int i;
float prevSerror = 0;
float newerror = 0;
float queue[qsize]={0,0,0,0,0,0,0,0,0,0};                  // Storing previous 10 error in queue for integral part
int n;

USB Usb;                                                   //USBHub Hub1(&Usb); // Some dongles have a hub inside


BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so        /* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;


void setup() {
  
  ST.motor(2, 0);
  delay(2000);
  
                                                      
  Serial.begin(115200);                               // Setting the baud rate
  SWSerial.begin(9600);                               // The default baud rate is 9600 and we are setting it to 115200
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);  
  SWSerial.end();
  SWSerial.begin(115200);
  Serial.begin(115200); 
  
#if !defined(_MIPSEL_)
  while (!Serial);                               // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt    
  }
  
    pinMode(ENC_IN_RIGHT_A,INPUT_PULLUP);                                             //Motor 1
    pinMode(ENC_IN_RIGHT_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);  //Using interrupt for Encoder 1 
     
    pinMode(ENC_IN_LEFT_A,INPUT_PULLUP);                                              //Motor 2
    pinMode(ENC_IN_LEFT_B,INPUT); 
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);    //Using interrupt for Encoder 2
    
    pinMode(ENC1_IN_RIGHT_A,INPUT_PULLUP);                                            //Motor 3
    pinMode(ENC1_IN_RIGHT_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_IN_RIGHT_A), right1_wheel_pulse, RISING); //Using interrupt for Encoder 3
 
    pinMode(ENC1_IN_LEFT_A,INPUT_PULLUP);                                              //Motor 4
    pinMode(ENC1_IN_LEFT_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_IN_LEFT_A), left1_wheel_pulse, RISING);   //Using interrupt for Encoder 4 
  
}

float new_rot_speed;                       //float M1_m2 = 0;


void loop() {
   if(i<qsize)                             // For i less than Queue
  {
   
  Usb.Task();

  int S1 = 0;                              // Initializing all the 4 motors
  int S2 = 0;
  int S3 = 0;
  int S4 = 0;

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
      

      
      
      Serial.println("S1");                                     // Assigning Motor Drivers to all the 4 Motors
      ST.motor(2, pid(S1, M1_m2));                                     // Calling pid function and giving it to the motor driver
       Serial.println("S2");
      ST.motor(1, pid(S2, M1_m1)); 
       Serial.println("S3");
      ST1.motor(2, pid(S3, M2_m2));
       Serial.println("S4");
      ST1.motor(1, pid(S4, M2_m1));

    currentMillis = millis();                       //Timing function

    if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;                 //Updating previous Millis
 
    M1_m2 = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);                   // Calculate revolutions per minute 
    right_wheel_pulse_count = 0;

    
    M1_m1 = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);                     // Calculate revolutions per minute
    left_wheel_pulse_count = 0;

    
    M2_m2 = (float)(right1_wheel_pulse_count * 60 / ENC_COUNT_REV);                 // Calculate revolutions per minute
    right1_wheel_pulse_count = 0;


    M2_m1 = (float)(left1_wheel_pulse_count * 60 / ENC_COUNT_REV);                    // Calculate revolutions per minute
    left1_wheel_pulse_count = 0;

  
        }   
      }      
    }
  }
}



 
