#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t val;
  int16_t val1;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE sensorData;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9
//Declaring motor speeds for the 4 motors
int S1,S2,S3,S4;

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define qsize 10
#define ENC_COUNT 324
#define ENC_IN_RIGHT_A 19 //SW 1 128 right of sabertooth
#define ENC_IN_RIGHT_B 24
#define ENC_IN_LEFT_A 18 //SW 1 128 left of sabertooth
#define ENC_IN_LEFT_B 22
#define ENC1_IN_RIGHT_A 20//SW 2 129 right sabertooth
#define ENC1_IN_RIGHT_B 26
#define ENC1_IN_LEFT_A 21//SW 2 129 left sabertooth
#define ENC1_IN_LEFT_B 28
#define dir1 29
#define dir2 30
#define dir3 31
#define dir4 32
#define pwm1 33
#define pwm2 34
#define pwm3 35
#define pwm4 36
#define sgn(x) ((x) < 0 ? 0 : ((x) > 0 ? 1 : 0))


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
int interval = 1;
  
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

int i;
float error[4]={0},preverror[4]={0};
float errorFunc[4] = {0};
float prevSerror[4] = {0};
float newerror[4] = {0};
float queue1[qsize]={0,0,0,0,0,0,0,0,0,0};
float queue2[qsize]={0,0,0,0,0,0,0,0,0,0};
float queue3[qsize]={0,0,0,0,0,0,0,0,0,0};
float queue4[qsize]={0,0,0,0,0,0,0,0,0,0};
//int n;





int PIDmotor1(float rpm_Right, float rpm_required){
  error[0] = rpm_required - rpm_Right;
  queue1[i]=error[0]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue1[n];     
  } 
   
  errorFunc[0] = 0.9*error[0] + 0.1*(preverror[0] - error[0])+ 0.01*error_sum;     //PID
  newerror[0] = (127*errorFunc[0])/424;
  preverror[0] = error[0];
  //Serial.println(newerror);
  return newerror[0];

}

int PIDmotor2(float rpm_Left, float rpm_required){
  error[1] = rpm_required - rpm_Left;
  queue2[i]=error[1]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue2[n];     
  } 
   
  errorFunc[1] = 0.9*error[1] + 0.1*(preverror[1] - error[1])+ 0.01*error_sum;     //PID
  newerror[1] = (127*errorFunc[1])/424;
  preverror[1] = error[1];
  //Serial.println(newerror);
  return newerror[1];

}

int PIDmotor3(float rpm_Right1, float rpm_required){
  error[2] = rpm_required - rpm_Right1;
  queue3[i] =error[2]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue3[n];     
  } 
   
  errorFunc[2] = 0.9*error[2] + 0.1*(preverror[2] - error[2])+ 0.01*error_sum;     //PID
  newerror[2] = (127*errorFunc[2])/424;
  preverror[2] = error[2];
  //Serial.println(newerror);
  return newerror[2];

}

int PIDmotor4(float rpm_Left1, float rpm_required){
  error[3] = rpm_required - rpm_Left1;
  queue4[i]=error[3]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue4[n];     
  } 
   
  errorFunc[3] = 0.9*error[3] + 0.1*(preverror[3] - error[3])+ 0.01*error_sum;     //PID
  newerror[3] = (127*errorFunc[3])/424;
  preverror[3] = error[3];
  //Serial.println(newerror);
  return newerror[3];

}

void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}


void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
    left_wheel_pulse_count++;
  }
  else {
    left_wheel_pulse_count--;
  }
}

void right1_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right1 = false; // Reverse
  }
  else {
    Direction_right1 = true; // Forward
  }
   
  if (Direction_right1) {
    right1_wheel_pulse_count++;
  }
  else {
    right1_wheel_pulse_count--;
  }
}

void left1_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left1 = false; // Reverse
  }
  else {
    Direction_left1 = true; // Forward
  }
   
  if (Direction_left1) {
    left1_wheel_pulse_count++;
  }
  else {
    left1_wheel_pulse_count--;
  }
}

// void receiveData(int byteCount) {
//   // Read the struct from the master device
//   if (byteCount == sizeof(sensorData)) {
//     Wire.readBytes((uint8_t *)&sensorData, sizeof(sensorData));
//     Serial.print("x ");
//     Serial.print(sensorData.val);
//     Serial.print(" y ");
//     Serial.print(sensorData.val1);
//   }
// }

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc. 
  ET.begin(details(sensorData), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive); 
  Serial.begin(115200); 
  
  //Defination of Pins for motor drivers

 
    pinMode(dir1,OUTPUT);
    pinMode(dir2,OUTPUT);
    pinMode(dir3,OUTPUT);
    pinMode(dir4,OUTPUT);
    pinMode(pwm1,OUTPUT);
    pinMode(pwm2,OUTPUT);
    pinMode(pwm3,OUTPUT);
    pinMode(pwm4,OUTPUT);

    
    pinMode(ENC_IN_RIGHT_A,INPUT_PULLUP);//Motor 1
    pinMode(ENC_IN_RIGHT_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
    pinMode(ENC_IN_LEFT_A,INPUT_PULLUP);//Motor 2
    pinMode(ENC_IN_LEFT_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
    pinMode(ENC1_IN_RIGHT_A,INPUT_PULLUP);//Motor 3
    pinMode(ENC1_IN_RIGHT_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC1_IN_RIGHT_A), right1_wheel_pulse, RISING);
    pinMode(ENC1_IN_LEFT_A,INPUT_PULLUP);//Motor 4
    pinMode(ENC1_IN_LEFT_B,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC1_IN_LEFT_A), left1_wheel_pulse, RISING); 
}

void loop() {
  int S1 = 0;
  int S2 = 0;
  int S3 = 0;
  int S4 = 0;

  int PWM1, PWM2, PWM3, PWM4 = 0;
  int DIR1,DIR2,DIR3,DIR4;
  //  if(i<qsize)
  //  {
    Serial.println(ET.receiveDadsdasrta());
   if(ET.receiveData()){ 
    // if (sensorData.val > 137 || sensorData.val < 117 || sensorData.val1 > 137 || sensorData.val1 < 117 ){

      //Speeds in x and y Directions
      int x = 2*((sensorData.val-130));
      int y = 2*(129 - (sensorData.val1));
       
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

      //Calculating the speeds of all 4 motors
      int s1 = (-0.7071*x + 0.7071*y)*0.7352; // max = 265.1285592, min = -265.1285592
      int s2 = (-0.7071*x - 0.7071*y)*0.7352;
      int s3 = (0.7071*x - 0.7071*y)*0.7352;
      int s4 = (0.7071*x + 0.7071*y)*0.7352;     

      //Mapping the Range of PWM in the Range of RPM
       S1=map(s1, -265, 265, -424, 424);
       S2=map(s2, -265, 265, -424, 424);
       S3=map(s3, -265, 265, -424, 424);
       S4=map(s4, -265, 265, -424, 424);
      
      
      // Serial.print(F("\r\nS1: "));
      // Serial.print(S1);
      // Serial.print(F("\r\nS2: "));
      // Serial.print(S2);
      // Serial.print(F("\r\nS3: "));
      // Serial.print(S3);
      // Serial.print(F("\r\nS4: "));
      // Serial.print(S4);
      
      //Assigning Motor Drivers to all the 4 Motors
      //  ST.motor(2, pid(S1));
      //  ST.motor(1, pid(S2)); 
      //  ST1.motor(2, pid(S3));
      //  ST1.motor(1, pid(S4));
      currentMillis = millis();

    if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT);
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
    
    rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT);
    ang_velocity_left = rpm_left * rpm_to_radians;   
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;
      
   
    rpm_right1 = (float)(right1_wheel_pulse_count * 60 / ENC_COUNT);
    ang_velocity_right1 = rpm_right1 * rpm_to_radians;   
    ang_velocity_right1_deg = ang_velocity_right1 * rad_to_deg;
 
   
    rpm_left1 = (float)(left1_wheel_pulse_count * 60 / ENC_COUNT);
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
    //ST.motor(1,pid());
    PWM1 = 2*abs(PIDmotor1(rpm_right,S1));
    DIR1=sgn(PIDmotor1(rpm_right,S1));
    PWM2 = 2*abs(PIDmotor2(rpm_left,S2));
    DIR2=sgn(PIDmotor2(rpm_left,S2));
    PWM3 = 2*abs(PIDmotor3(rpm_right1,S3));
    DIR3=sgn(PIDmotor3(rpm_right1,S3));
    PWM4 = 2*abs(PIDmotor4(rpm_left1,S4));
    DIR4=sgn(PIDmotor4(rpm_left1,S4));
    // }
        
      }
     Serial.print("Blink:");
    Serial.println(sensorData.val); 
    Serial.print("Pause:");
    Serial.println(sensorData.val1);
  }

    // if((PWM1<30 && PWM1>-30))
    // {
    // PWM1 = 0;
    // }
    // if((PWM2<30 && PWM2>-30))
    // {
    // PWM2 = 0;
    // }
    // if((PWM3<30 && PWM3>-30))
    // {
    // PWM3 = 0;
    // }
    // if((PWM4<30 && PWM4>-30))
    // {
    // PWM4 = 0;
    // }
    

  // Serial.print(S1);
  // Serial.print(" ");
  // Serial.print(PWM1);
  // Serial.print(" ");
  // Serial.print(S2);
  // Serial.print(" ");
  // Serial.print(PWM2);
  // Serial.print(" ");
  // Serial.print(S3);
  // Serial.print(" ");
  // Serial.print(PWM3);
  // Serial.print(" ");
  // Serial.print(S4);
  // Serial.print(" ");
  // Serial.println(PWM4);

  // ST1.motor(2,PWM1);
  // ST1.motor(1,PWM2);
  // ST.motor(2,PWM3);
  // ST.motor(1,PWM4);

  digitalWrite(pwm1,PWM1);
  digitalWrite(pwm2,PWM2);
  digitalWrite(pwm3,PWM3);
  digitalWrite(pwm4,PWM4);
  digitalWrite(dir1,DIR1);
  digitalWrite(dir2,DIR2);
  digitalWrite(dir3,DIR3);
  digitalWrite(dir4,DIR4);
   
       
    
  // }
  
}
void receive(int numBytes) {}
