#include <LiquidCrystal.h>
#include <EasyTransfer.h>
#include <TimerOne.h>
#include <Servo.h>
int velpass=0;
enum {BLUE = -1, RED = 1};
int arenaFlag = RED;

const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

EasyTransfer ETUpperLower, ETMpu, ETPlot, ETAgentPR, ETAgentTR;

struct DATASTRUCT {
  float omega;
  float velocity;
  float theta;
};
DATASTRUCT mydata;

struct MPU {
  float robotYaw;
};
MPU mympu;

struct PLOT {
  float x;
  float y;
};
PLOT myplot;

struct MULTIAGENT {
  float x;
  float y;
  float yaw;
};
MULTIAGENT agentPR,agentTR;
 
#define UpperLowerSerial Serial2
#define MPUSerial  Serial2
#define BluetoothSerial Serial3
#define BluetoothSerial2 Serial1
#define CollabSerial Serial3
#define SerialLSA Serial

#define pi 3.141592653589

#define DegreeToRadian(x) x*0.0174532  
#define RadianToDegree(x) x*57.295779

#define RadiusXYWheel   2.9 //centimetre
#define maxWheelRPM 560
#define RadiusOmniDrive 37.5  //centimetre
#define RadiusOmniWheel 7.5 //centimetre
#define RadiusXYCircle 4.5 //centimetre

#define VelocityToRPM(x) x*60/(2*pi*RadiusOmniWheel) //vel in cm/s
#define RPMToVelocity(x) (2*pi*RadiusOmniWheel*x)/60


#define redLEDoff digitalWrite(A4,LOW);
#define redLEDon digitalWrite(A4,HIGH);
#define yellowLEDon digitalWrite(A6, HIGH)
#define yellowLEDoff digitalWrite(A6, LOW)
#define greenLEDon digitalWrite(A5, LOW)
#define greenLEDoff digitalWrite(A5, HIGH)
#define buzzOn digitalWrite(A1,HIGH)
#define buzzOff digitalWrite(A1,LOW)
#define proximityPin A15

class Encoder
{
  public:
    int channel1;
    int channel2;
    int ppr;
    volatile long long int Count;
    volatile long long int prevCount;
    int rpm;

    Encoder(int ch1, int ch2, int ppR){
      channel1 = ch1;
      channel2 = ch2;
      ppr = ppR;
      Count = 0;
      prevCount = 0;
      rpm = 0;
      pinMode(channel1, INPUT_PULLUP);
      pinMode(channel2, INPUT_PULLUP);
    }
};

Encoder xencoder(2, 4, 1000);  
Encoder *pEncoderX = &xencoder;

Encoder yencoder(3, 5, 1000); 
Encoder *pEncoderY = &yencoder;

void returnCountX();
void returnCountY();

/*********************************************************************************************************************************************/
/*********************************************************** Autonomous Bot **********************************************************************************/
class Auto_Bot {
  public:
    volatile float X_pos;
    volatile float Y_pos;
    volatile float X_pos1;
    volatile float Y_pos1;
    volatile float del_x;
    volatile float del_y;
    float Angle;
    float vel;
    float Omega;
    volatile float prev_X;
    volatile float prev_Y;
    Auto_Bot() {
      X_pos = 0;
      Y_pos = 0;
      X_pos1 = 0;
      Y_pos1 = 0;
      del_x = 0;
      del_y = 0;
      Angle = 0;
      prev_X = 0;
      prev_Y = 0;
      vel = 0;
    }
};

Auto_Bot fourWheelDrive;
Auto_Bot *pBot = &fourWheelDrive;

class PID {
  public:
    float Kp;
    float Kd;
    float Ki;
    float Kp_old;
    float Kd_old;
    float Ki_old;
    float maxControl;
    float minControl;

    float required;
    float prevRequired;
    float error;
    float prevError;
    float derivativeError;
    float integralError;
    void initPID(float kp, float kd, float ki, float req, float minV, float maxV);
    float pidControl(float actual);
   
};

PID pidDistance;
PID *ppidDistance = &pidDistance;

PID pidOmega;
PID *ppidOmega = &pidOmega;

PID pidCircle;
PID *ppidCircle = &pidCircle;

PID pidRoller;
PID *ppidRoller = &pidRoller;

PID pidCollaborateX;
PID *ppidCollaborateX = &pidCollaborateX;

PID pidCollaborateY;
PID *ppidCollaborateY = &pidCollaborateY;

PID pidLSA080;
PID *ppidLSA080=&pidLSA080;

PID pidLSA081;
PID *ppidLSA081=&pidLSA081;

//PID pidCollaborateDist;
//PID *pidCollaborateDist= & pidCollaborateDist;



enum {forward, backward, stopp};

class Motor {
  public:
    int direction1;
    int direction2;
    int pwmPin;

    Motor(int dir1, int dir2, int pwmP) {
      direction1 = dir1;
      direction2 = dir2;
      pwmPin = pwmP;
      
      pinMode(direction1, OUTPUT);
      pinMode(pwmPin, OUTPUT);
      pinMode(direction2, OUTPUT);
      digitalWrite(direction1, LOW);
      digitalWrite(direction2, LOW);
    }

    void actuateMotor(int pwmm, uint8_t directionn)   //forward, backward, stopp
    {
      if(directionn != stopp)
      {
        digitalWrite(direction1, !directionn);
        digitalWrite(direction2, directionn);
      }
      else
      {
        digitalWrite(direction1, HIGH);
        digitalWrite(direction2, HIGH);
      }

      analogWrite(pwmPin,pwmm);  
    }
};
//////////////////////////////////////////////////  PS2 ///////////////////////////////////////////////////////////////

EasyTransfer ET_ps3;
  
struct PS3_data {
  float ps3data;
  float LD;
  float LT;
  float RD;
  float RT;
};
PS3_data data;     


enum value {noData, LEFTx, UPx, RIGHTx, DOWNx, SELECTx, PSx, STARTx, SQUAREx, TRIANGLEx, CIRCLEx, CROSSx, L1x, L2x, L3x, R1x, R2x, R3x};
value PS3Button = noData;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////LSA08////////////////////////////////////////////////////

const int rotateRPM = 120;
unsigned int rpmmax = 400;
bool lsa = true, junctionFlag = 0, junctionFlag2 = 0;
enum {f,b,r,l,s};
float reading=255;

class LSA08{
 public: 
  char Address;
  int Theta;
  int JunctionPin;
  int JunctionCount;
  int OePin;
  uint8_t junction_detect[4];
  
  LSA08(char a, int b, int c, int d, int eee){
    this->Address = a;
    this->Theta = b;
    this->JunctionPin = c;
    this->JunctionCount = d;
    this->OePin = eee;
  };
  
  void initLSA(int,int);
  void sendCommand(char,char,char);
  void ChangeBaud(char, char);
  void clearJunction(char);
  int getJunction(char);
  int GetByteOfLSA(int);
  void Calibrate(int);
};

LSA08 LSAX(0x01,90,23,0,25);    // f=1//2->4
LSA08 LSAY(0x04,0,39,0,41);
LSA08 *LSAArray[2]={&LSAX,&LSAY};



////////////////////////////////////////////////////////////////////////////////////////////////////////

class Obstacle {
  public:
    float h;
    float k;
    float r;
    Obstacle(float H, float K, float R){
      h = H;
      k = K;
      r = R;
    }
};
Obstacle pole1(157.5,407.5,100);
Obstacle pole2(290.5,256.0,100);
Obstacle pole3(425.0,407.5,100);
Obstacle pole4(556.5,256.0,100);
Obstacle pole5(691.0,407.5,100);
Obstacle *pole[5] = {&pole1, &pole2, &pole3, &pole4, &pole5};

enum {STOP,CONTINUE};
enum {clockwise = 1, antiClockwise = -1};
enum {accelerate, decelerate, constant, accdec};


enum State {checkCollision, avoidCollision, nearObstacle, aroundObstacle, motionToGoal};
State planState = checkCollision;

enum FSM {arenaSelect, waitForRefree, receiveBall, Collaborate, placeBall, resetState, variableState, goHome, kickBall,alignForKick, manualDrive};
enum zone {startZone = -1 ,kickingZone = 0 , receiveZone = 1};
FSM fsmState = arenaSelect;

int speedMode = constant;
int initYaw = 0, timerMPU = 0;

bool collabFlag = 0, startPlanning = 0;
bool resetFlag=1, ps2Flag = 0;
bool integralFlag = 0;

float vBot =0.0;
int resetAngle = 0;

float shiftX = 1000-(41.52+6), shiftY = 42.9 ; // SHIFTX SHIFTY 41.82 31.9
float resetSx = 0, resetSy = 0;

float minSpeed = 0, maxSpeed = 0;
float BotX = shiftX, BotY = shiftY, prevBotX = 0, prevBotY = 0;
float botTheta = 0,  manualSpeed = 350;
float robotYaw = 0.0, omegacontrol = 0.0, constError = 0, prevRobotYaw = 0;
float delYaw = 0.0 , prevYaw = 0.0;
float initialRobotYaw = 0;

float trySpotX[5] ={159.0,292.0,425.0,558.0,691.0};
float trySpotY[5] ={565.5,565.5,565.5,565.5,565.5};
float kickingX = 900, kickingY = 300,kickingSetupX = 900,kickingSetupY = 280, kickingAngle = 17.5;
float receiveX = 930.5, receiveY = shiftY; //845.5,38
float goalX  = 0, goalY = 0, startX = 0, startY = 0;
 
int detObjects[5] = {-1,-1,-1,-1,-1};

long long int buzzTime = 0;

void setup() {

//  initLSA();

  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  pinMode(A6,OUTPUT);
  pinMode(A7,OUTPUT);
  pinMode(proximityPin,INPUT);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  
  Serial.begin(9600);
  Serial1.begin(9600);
  UpperLowerSerial.begin(115200);
  ETUpperLower.begin(details(mydata), &UpperLowerSerial);
  

  MPUSerial.begin(115200);
  ETMpu.begin(details(mympu), &MPUSerial);

  BluetoothSerial.begin(115200);
  ET_ps3.begin(details(data), &BluetoothSerial);


  BluetoothSerial2.begin(38400);
//  ETPlot.begin(details(myplot), &BluetoothSerial2);
//  
//  CollabSerial.begin(115200);
//  ETAgentPR.begin(details(agentPR), &CollabSerial);
//  ETAgentTR.begin(details(agentTR), &CollabSerial);

  mydata = {0,0,0};
  mympu = {0};
  agentPR = {0,0,0};
  agentTR = {0,0,0};

  ETUpperLower.sendData();

  
  
  attachInterrupt(digitalPinToInterrupt(pEncoderX->channel1), returnCountX, RISING);
  attachInterrupt(digitalPinToInterrupt(pEncoderY->channel1), returnCountY, RISING);
  Timer1.initialize(50000);
  Timer1.attachInterrupt(calculatebotTheta); // blinkLED to run every 0.15 seconds
  interrupts();
//  ETAgentTR.sendData();

  
  // p d i req min max
  ppidOmega->initPID(0.20,2.25,0.001, 0, -3, 3);   //0.2 1.0   //0.4,2.5  
  ppidDistance->initPID(7.50,50,0,0,-400,400);
  ppidCircle->initPID(15,0,0,0,-560,560);//15 5
  ppidRoller->initPID(250,200,0,0,-200,200);
  ppidCollaborateX->initPID(0,0,0,0,-375,375);
  ppidCollaborateY->initPID(0,0,0,0,-375,375);
  ppidLSA080->initPID(6, 1.75, 0, 0, -150, 150);
  ppidLSA081->initPID(3, 3.3, 0, -15, -150, 150);
  
  Serial.println("Setup");  
  
  timerMPU = millis();
  while(millis() - timerMPU < 500)
  {
    while(ETMpu.receiveData()>0)
      robotYaw = mympu.robotYaw;
    initialRobotYaw = robotYaw;
  }
  Serial1.println("hi mc");
}

void loop()
{
    finiteStateMachine();
//      calculateSpeed(-omegacontrol,DegreeToRadian(180),velpass);
}
