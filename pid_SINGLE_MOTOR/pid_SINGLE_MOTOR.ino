#include <Encoder.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>


SoftwareSerial SWSerial(NOT_A_PIN, 30);
Sabertooth ST(129, SWSerial);   //1st motor driver
Sabertooth ST1(128, SWSerial);  //2nd motor driver
#define ENC_COUNT_REV 324
#define ENC1_IN_LEFT_A 21  //SW  129 left sabertooth
#define ENC1_IN_LEFT_B 28
#define qsize 10
float queue[qsize] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int n;
volatile long left1_wheel_pulse_count = 0;
long previousMillis = 0;
long currentMillis = 0;
boolean Direction_right = true;
boolean Direction_left = true;
boolean Direction_right1 = true;
boolean Direction_left1 = true;
float interval=100, error, preverror = 0;
float errorFunc;
float actual;
int i;
float prevSerror = 0;
float newerror = 0;
float rpm_left1 = 0;
float new_rot_speed;
void setup() {
  // put your setup code here, to run once:
  SWSerial.begin(9600);
  ST.setBaudRate(115200);
  ST1.setBaudRate(115200);
  SWSerial.end();
  SWSerial.begin(115200);
  Serial.begin(115200);
   pinMode(ENC1_IN_LEFT_A, INPUT_PULLUP);  //Motor 4
  pinMode(ENC1_IN_LEFT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_IN_LEFT_A), left1_wheel_pulse, RISING);
  
}

void loop() {
  if (i < qsize) {

currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    rpm_left1 = (float)((left1_wheel_pulse_count * 60*1000) / (ENC_COUNT_REV*interval));
  
    left1_wheel_pulse_count = 0;
    ST.motor(1, pid());
//     ST.motor(1, 127);
// Serial.println(rpm_left1);    
  }
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

int pid() {
  // int S=(float)((250*127)/450);
  new_rot_speed = rpm_left1;
  error = 250- new_rot_speed;
  queue[i] = error;
  i++;
  //Serial.println(error);

  if (i >= 10) {
    i = 0;
  }

  int error_sum = 0;
 
  //Serial.println(error);

  
for (n = 0; n < qsize; n++) {
    error_sum = error_sum + queue[n];
  }
  errorFunc = 2.6* error+ .4*(preverror - error) + 0.15*error_sum;     //PID

  newerror = (127* errorFunc) / 445 ;
  preverror = error;
  if (newerror > 127)
    newerror = 127;
  if (newerror < -127)
    newerror = -127;

  //  Serial.println(errorFunc);
  // Serial.println(450);
 
    Serial.print(rpm_left1);
    Serial.print(" ");
    // Serial.print(errorFunc);
    // Serial.print(" ");
    Serial.println(newerror);
    // Serial.print(',');
 
  
  return newerror;
}
