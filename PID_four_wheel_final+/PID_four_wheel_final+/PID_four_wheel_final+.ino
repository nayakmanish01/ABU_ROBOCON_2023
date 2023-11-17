/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3BT.h>
#include <usbhub.h>
#include <Encoder.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);  //2nd motor driver

//Declaring motor speeds for the 4 motors
int S1, S2, S3, S4;

// Motor encoder output pulses per 360 degree revolution (measured manually)

#define ENC_COUNT_REV 324
#define ENC_IN_RIGHT_A 19  //SW  128 right of sabertooth
#define ENC_IN_RIGHT_B 24
#define ENC_IN_LEFT_A 18  //SW  128 left of sabertooth
#define ENC_IN_LEFT_B 22
#define qsize 10

#define ENC1_IN_RIGHT_A 20  //SW  129 right sabertooth
#define ENC1_IN_RIGHT_B 26
#define ENC1_IN_LEFT_A 21  //SW  129 left sabertooth
#define ENC1_IN_LEFT_B 28


boolean Direction_right = true;
boolean Direction_left = true;
boolean Direction_right1 = true;
boolean Direction_left1 = true;

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
volatile long right1_wheel_pulse_count = 0;
volatile long left1_wheel_pulse_count = 0;
// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;
float rpm_right1 = 0;
float rpm_left1 = 0;

// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;
float ang_velocity_right1 = 0;
float ang_velocity_right1_deg = 0;
float ang_velocity_left1 = 0;
float ang_velocity_left1_deg = 0;
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;

//Values of Kp, Kd and Ki
float kp = 1, kd = 0, ki = 0;

float olderror, error, preverror = 0;
float errorFunc;
float actual;
int i;
float prevSerror = 0;
float newerror = 0;
float queue[qsize] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int n;

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb);  // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd);  // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;


void setup() {


  delay(2000);

  //Setting the baud rate

  SWSerial.begin(9600);
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);
  SWSerial.end();
  SWSerial.begin(115200);
  Serial.begin(115200);
  // ST.motor(1, 127);
#if !defined(MIPSEL)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }

  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);  //Motor 1
  pinMode(ENC_IN_RIGHT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  //ST.motot(2,0);
  //delay(2000);

  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);  //Motor 2
  pinMode(ENC_IN_LEFT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
  //ST.motot(1,0);
  //delay(2000);

  pinMode(ENC1_IN_RIGHT_A, INPUT_PULLUP);  //Motor 3
  pinMode(ENC1_IN_RIGHT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_IN_RIGHT_A), right1_wheel_pulse, RISING);
  //ST1.motot(2,0);delay(2000);
  //delay(2000);

  pinMode(ENC1_IN_LEFT_A, INPUT_PULLUP);  //Motor 4
  pinMode(ENC1_IN_LEFT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_IN_LEFT_A), left1_wheel_pulse, RISING);
  //ST1.motot(2,0);
  delay(2000);
}

float new_rot_speed;
//float rpm_right = 0;

void loop() {
  if (i < qsize) {

    Usb.Task();

    int S1 = 0;
    int S2 = 0;
    int S3 = 0;
    int S4 = 0;

    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {

        //Speeds in x and y Directions
        int x = 2 * ((PS3.getAnalogHat(LeftHatX) - 130));
        int y = 2 * (129 - (PS3.getAnalogHat(LeftHatY)));

        if (x < -255) {
          x = -255;
        }
        if (x > 255) {
          x = 255;
        }
        if (y < -255) {
          y = -255;
        }
        if (y > 255) {
          y = 255;
        }
        if (x < 50 and x > -50) {
          x = 0;
        }
        if (y < 50 and y > -50) {
          y = 0;
        }

        //Calculating the speeds of all 4 motors
        int s1 = (-0.7071 * x + 0.7071 * y) * 0.7352;
        int s2 = (-0.7071 * x - 0.7071 * y) * 0.7352;
        int s3 = (0.7071 * x - 0.7071 * y) * 0.7352;
        int s4 = (0.7071 * x + 0.7071 * y) * 0.7352;

        //Mapping the Range of PWM in the Range of RPM
        // S1=map(s1, -255, 255, -510, 510);
        // S2=map(s2, -255, 255, -510, 510);
        // S3=map(s3, -255, 255, -510, 510);
        // S4=map(s4, -255, 255, -510, 510);


        // Serial.print(F("\r\nS1: "));
        // Serial.print(S1);
        // Serial.print(F("\r\nS2: "));
        // Serial.print(S2);
        // Serial.print(F("\r\nS3: "));
        // Serial.print(S3);
        // Serial.print(F("\r\nS4: "));
        // Serial.print(S4);
        // ST.motor(1,127);
        //Assigning Motor Drivers to all the 4 Motors
        //  ST.motor(2, pid(S1));
        //   ST.motor(1, pid(S2));
        //   ST1.motor(2, pid(S3));
        //   ST1.motor(1, pid(S4));
      }
    }
  }
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians;
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;

    // Serial.print(" new_rot_speed ");
    // Serial.println(new_rot_speed);
    // Serial.print(" Speed: ");
    // Serial.print(rpm_right);


    rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_left = rpm_left * rpm_to_radians;
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;


    rpm_right1 = (float)(right1_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right1 = rpm_right1 * rpm_to_radians;
    ang_velocity_right1_deg = ang_velocity_right1 * rad_to_deg;


    rpm_left1 = (float)(left1_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_left1 = rpm_left1 * rpm_to_radians;
    ang_velocity_left1_deg = ang_velocity_left1 * rad_to_deg;

    // Serial.print(" Pulses: ");
    // Serial.println(left_wheel_pulse_count);
    // Serial.println(right_wheel_pulse_count);
    // Serial.println(left1_wheel_pulse_count);
    // Serial.println(right1_wheel_pulse_count);
    // Serial.println(rpm_left1);

    left_wheel_pulse_count = 0;
    right_wheel_pulse_count = 0;
    left1_wheel_pulse_count = 0;
    right1_wheel_pulse_count = 0;
     ST.motor(1, pid());
  //   ST.motor(1,127);
  // Serial.println(rpm_left1);
  }


  // ST.motor(1,127);
  // Serial.println(rpm_left1);
}

void right_wheel_pulse() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {
    right_wheel_pulse_count++;
  } else {
    right_wheel_pulse_count--;
  }
}


void left_wheel_pulse() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = false;  // Reverse
  } else {
    Direction_left = true;  // Forward
  }

  if (Direction_left) {
    left_wheel_pulse_count++;
  } else {
    left_wheel_pulse_count--;
  }
}

void right1_wheel_pulse() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right1 = false;  // Reverse
  } else {
    Direction_right1 = true;  // Forward
  }

  if (Direction_right1) {
    right1_wheel_pulse_count++;
  } else {
    right1_wheel_pulse_count--;
  }
}

void left1_wheel_pulse() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_LEFT_B);

  if (val == LOW) {
    Direction_left1 = false;  // Reverse
  } else {
    Direction_left1 = true;  // Forward
  }

  if (Direction_left1) {
    left1_wheel_pulse_count++;
  } else {
    left1_wheel_pulse_count--;
  }
}

//Function for PID
int pid(int S) {

  new_rot_speed = rpm_left1;
  error = 250 - new_rot_speed;
  queue[i] = error;
  i++;
  //Serial.println(error);

  if (i >= 10) {
    i = 0;
  }

  int error_sum = 0;
  for (n = 0; n < qsize; n++) {
    error_sum = error_sum + queue[n];
  }

  errorFunc = 1.0* error+ 0.0*(preverror - error)+ 0.0*error_sum;     //PID
  actual = new_rot_speed;
  newerror = (127 * errorFunc) / 425;
  preverror = error;
  if (newerror > 127)
    newerror = 127;
  if (newerror < -127)
    newerror = -127;

  //  Serial.println(errorFunc);
  // Serial.println(450);
  
    Serial.print(rpm_left1);
    Serial.print(" ");
    Serial.println(newerror);
    // Serial.print(',');
  
  return newerror;
}