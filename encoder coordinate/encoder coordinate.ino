#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_9Axis_MotionApps41.h>
#include <helper_3dmath.h>

#include <Encoder.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include<math.h>

MPU6050 mpu6050(Wire);

long timer = 0;
int Count_x,Count_y;
long position_x  = -999;
long position_y = -999;
float RadiusXYWheel=0.285;// in cm
long prev_X=0,prev_Y=0,prevRobotX=0,prevRobotY=0,RobotX=0,RobotY=0,X_pos1=0,Y_pos1=0,del_x,del_y;
Encoder encoder_x(2, 7);
Encoder encoder_y(3, 8);

float the(float del_y,float del_x,float botYaw)//doubt
{
  /*botYaw = int(botYaw*10);
  botYaw = float(botYaw)/10.0;*/
   
  return (atan2(del_y,del_x) + ((botYaw*M_PI)/180));
}

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  long new_x, new_y;
  float total_turnx,old_turnx,total_turny,old_turny;
  
  new_x = encoder_x.read();
  total_turnx=new_x/4000;
  Count_x=total_turnx-old_turnx;
  old_turnx=total_turnx;
  new_y = encoder_y.read();
  total_turny=new_y/4000;  
  Count_y=total_turny-old_turny;
  old_turny=total_turny;
  if (new_x != position_x || new_y != position_y) {
    Serial.println("========================ENCODER DATA===============================");
    Serial.print("x = ");
    Serial.print(Count_x);
    Serial.print("y = ");
    Serial.print(Count_y);
    Serial.println("=======================================================\n");
    position_x = new_x;
    position_y = new_y;
    
  }
  
 mpu6050.update();
   if(millis() - timer > 500){
    
       Serial.println("========================IMU DATA===============================\n");
    /*Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());*/
   
    float Yaw=mpu6050.getAngleZ();
    Serial.print(Yaw);
     
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");
    timer = millis();

    prev_X = X_pos1;    
    prev_Y = Y_pos1; 
      
    X_pos1 = (Count_x * 2 * M_PI * RadiusXYWheel /4000); //cm   
    del_x = X_pos1 - prev_X ;   
  
     
    Y_pos1 = (Count_y * 2 * M_PI * RadiusXYWheel /4000); //cm
    del_y = Y_pos1 -prev_Y ;
    
    float r = sqrt(pow(del_x,2)+pow(del_y,2));
    float Theta = the(del_y,del_x,Yaw);// doubt
       
    prevRobotX = RobotX;
    prevRobotY = RobotY;
      
    RobotX = prevRobotX + r*(cos(Theta));
    RobotY = prevRobotY + r*(sin(Theta));
    Serial.print("Coordinates of X:");
    Serial.print(RobotX);
    Serial.print("Coordinates of Y:");
    Serial.print(RobotY);
  }

 
}

////////////////


