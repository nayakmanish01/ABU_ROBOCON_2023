
#include <EasyTransfer.h>
#include <DueTimer.h>
#include <DuePWM.h>

#define SerialX SerialUSB


#define N 3
#define TimerEncoder Timer1
#define UpperLowerSerial Serial3
#define PWM_FREQ1 3000
#define PWM_FREQ2 3000
#define EncoderTime 0.1

#define maxMotRPMA 560
#define maxMotRPM 590
#define maxWheelRPM 560

#define maxPWM 255
#define baserpm 0
#define RPMToVelocity(x) (2*pi*RadiusOmniWheel*x)/60
#define maxRobotVelocity 1.2*RMPToVelocity(maxWheelRPM)


#define pi 3.141592653589
#define DegreeToRadian(x) x*0.0174532  
#define RadianToDegree(x) x*57.295779

#define RadiusOmniDrive 37.5  //centimetre
#define RadiusOmniWheel 7.5 //centimetre
#define VelocityToRPM(x) x*60/(2*pi*RadiusOmniWheel) //vel in cm/s

#define Angle1 90
#define Angle2 90+120  //3.66519  //210      //90+120
#define Angle3 90+240  //5.75958  //330      //90+240

#define blueLEDon digitalWrite(A10,HIGH)
#define blueLEDoff digitalWrite(A10,LOW)

DuePWM pwm(PWM_FREQ1, PWM_FREQ2);

EasyTransfer ET;

struct DATASTRUCT {
  float omega;
  float velocity;
  float theta;
};
DATASTRUCT mydata;

class Wheel {
  public:
    float translationSpeed;
    float angularSpeed;
    int angle;
    float maxRPM;
    float Speed;
    float rpm;
    int prevRPM;
};

Wheel Wheel1 = {0.0, 0.0, Angle1, maxWheelRPM, 0, 0, 0 }; 
Wheel Wheel2 = {0.0, 0.0, Angle2, maxWheelRPM, 0, 0, 0 }; 
Wheel Wheel3 = {0.0, 0.0, Angle3, maxWheelRPM, 0, 0, 0 }; 

Wheel *pWheel[N] = {&Wheel1, &Wheel2, &Wheel3};

class Encoder
{
  public:
    int channel1;
    int channel2;
    int ppr;
    volatile long long int Count;
    volatile long long int prevCount;
    int rpm;
    float gearRatio;
    void initEncoder() {
      pinMode(channel1, INPUT);
      pinMode(channel2, INPUT);
    }
};

Encoder encoder1 = {50, 48, 1000, 0, 0, 0,18.0/56.0} ,   *pEncoder1 = &encoder1;
Encoder encoder2 = {52, 53, 1000, 0, 0, 0,39.0/35.0} ,   *pEncoder2 = &encoder2;
Encoder encoder3 = {51, 47, 1000, 0, 0, 0,39.0/35.0} ,   *pEncoder3 = &encoder3;

Encoder *pEncoder[N] = {&encoder1, &encoder2, &encoder3};

void returnCount1();
void returnCount2();
void returnCount3();


class Motor {
  public:
    int direction1;
    int direction2;
    int pwmPin;

    void initMotor() {
      pinMode(direction1, OUTPUT);
      pinMode(pwmPin, OUTPUT);
      pinMode(direction2, OUTPUT);
      digitalWrite(direction1, LOW);
      digitalWrite(direction2, LOW);
    }
};


Motor motor1 = {25,23, 8};
Motor motor2 = {27,29, 7};   
Motor motor3 = {35,33, 6};

Motor *pMotor[N] = {&motor1, &motor2, &motor3};
/*********************************************************************************************************************************************/
/******************************************************       PID          ***************************************************************************************/

class PID {
  public:
    float Kp;
    float Kd;
    float Ki;
    float maxControl;
    float minControl;
    
    float required;
    float prevRequired;
    float error;
    float prevError;
    float derivativeError;
    float integralError;
    float prev_integralError;
    void initPID(float kp, float kd, float ki, float req, float minV, float maxV);
    float pidControl(float actual);

};

PID PIDMotor1;
PID *pPIDMotor1 = &PIDMotor1;

PID PIDMotor2;
PID *pPIDMotor2 = &PIDMotor2;

PID PIDMotor3;
PID *pPIDMotor3 = &PIDMotor3;

PID *pPIDMotor[N] = {&PIDMotor1, &PIDMotor2, &PIDMotor3};


//////////////////////////////////////////////////Global variables****************************************************
float output[N];
int softBrake = 1, directionChange[3] = {0,0,0};

void setup() {
  
  pinMode(A10,OUTPUT);
  blueLEDon;
  UpperLowerSerial.begin(115200);
  ET.begin(details(mydata), &UpperLowerSerial);
  
  for (int i = 0; i < N; ++i)
    pMotor[i]->initMotor();

  pwm.setFreq1(PWM_FREQ1);
  pwm.setFreq2(PWM_FREQ2);
  pwm.pinFreq2(pMotor[0]->pwmPin);
  
  for (int i = 1; i < N; ++i)
    pwm.pinFreq1(pMotor[i]->pwmPin);

  for (int i = 0; i < N; ++i)
    pEncoder[i]->initEncoder();

  attachInterrupt(pEncoder1->channel1, returnCount1, RISING);
  attachInterrupt(pEncoder2->channel1, returnCount2, RISING);
  attachInterrupt(pEncoder3->channel1, returnCount3, RISING);

  TimerEncoder.attachInterrupt(timerHandler);
  TimerEncoder.start(1000000 * EncoderTime);

  pPIDMotor1->initPID(1.8, 0.0, 0, 0, -baserpm, maxMotRPM - baserpm);//4.2 1.2
  pPIDMotor2->initPID(1.8, 0.0, 0, 0, -baserpm, maxMotRPM - baserpm);//1.1 
  pPIDMotor3->initPID(1.8, 0.0, 0, 0, -baserpm, maxMotRPM - baserpm);//1.8

    pPIDMotor[1]->required = -100;
}

float linearV =0, angularV =0, angleTheta =0 ;
int ledCntr=0;
void loop() { 
  getUpperData();
//  SerialX.println(String(output[0])+" "+String(output[1])+" "+String(output[2]));
}


void timerHandler()
{
  for(int i = 0; i < N; ++i)
    pEncoder[i]->rpm = ((pEncoder[i]->Count - pEncoder[i]->prevCount) * 60.0) / (EncoderTime * pEncoder[i]->gearRatio * pEncoder[i]->ppr);

  
  for(int i = 0; i < N; ++i)
    pEncoder[i]->prevCount = pEncoder[i]->Count;

   for(int i = 0; i < N; ++i)
  {
    if(pPIDMotor[i]->required * pPIDMotor[i]->prevRequired > 0 && directionChange[i] == 0)
    {
      output[i] = baserpm + pPIDMotor[i]->pidControl(pEncoder[i]->rpm);
    }
    else
    {
      softBrake = 1;
      output[i] = 0;
      if(fabs(pEncoder[i]->rpm) > 40)
        directionChange[i] = 1;
      else
        directionChange[i] = 0;
    }
    pPIDMotor[i]->prevRequired = pPIDMotor[i]->required;
  }
  
  for(int i = 0; i < N; ++i)
  {
    if(i == 0)
      driveMotorReq(output[i], pPIDMotor[i], pMotor[i], maxMotRPMA);
    else
      driveMotorReq(output[i], pPIDMotor[i], pMotor[i], maxMotRPM);
  }
      
}
