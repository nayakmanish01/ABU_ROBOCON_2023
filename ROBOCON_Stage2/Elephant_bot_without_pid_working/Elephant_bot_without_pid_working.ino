struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t x;
  int16_t y;
};

RECEIVE_DATA_STRUCTURE data;


int S1,S2,S3,S4;

#define dir1 13
#define dir2 11
#define dir3 8
#define dir4 5
#define pwm1 12
#define pwm2 10
#define pwm3 9
#define pwm4 6
#define sgn(x) ((x) < 0 ? 0 : ((x) > 0 ? 1 : 0))


boolean Direction_right = true;
boolean Direction_left = true;
boolean Direction_right1 = true;
boolean Direction_left1 = true;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial3.begin(9600); // Initialize serial communication
  Serial3.setTimeout(10); 
 
  //Defination of Pins for motor drivers


    pinMode(dir1,OUTPUT);
    pinMode(dir2,OUTPUT);
    pinMode(dir3,OUTPUT);
    pinMode(dir4,OUTPUT);
    pinMode(pwm1,OUTPUT);
    pinMode(pwm2,OUTPUT);
    pinMode(pwm3,OUTPUT);
    pinMode(pwm4,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  

  int PWM1, PWM2, PWM3, PWM4 = 0;
  int DIR1,DIR2,DIR3,DIR4;
  if (Serial3.available() >= sizeof(RECEIVE_DATA_STRUCTURE)) {
  // Check if a full PS3Data struct has been received
  // Create a PS3Data struct to hold the received data
    Serial3.readBytes((uint8_t*)&data, sizeof(data));
    Serial.print("x ");Serial.println(data.x);
    Serial.print("y");
    Serial.println(data.y);
    }


if ((data.x > 137) || (data.x < 117 )|| (data.y > 137) || (data.y < 117) ) {
     
      //Speeds in x and y Directions
      int x = 2*((data.x-130));
      int y = 2*(129 - (data.y));
      //  int x = 2*((255-130));
      // int y = 2*(129 - 127);
      // Serial.println(x);
      // Serial.println(y);
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

    PWM1 = abs(s1);
    DIR1=sgn(s1);
    PWM2 = abs(s2);
    DIR2=sgn(s2);   
    PWM3 = abs(s3);
    DIR3=sgn(s3);    
    PWM4 = abs(s4);
    DIR4=sgn(s4);

}

else 
{
  PWM1 = 0;
    DIR1=0;
    PWM2 = 0;
    DIR2=0;   
    PWM3 = 0;
    DIR3=0;    
    PWM4 = 0;
    DIR4=0;

}
  // Serial.print("PWM1 "); Serial.print(PWM1); Serial.print(" "); Serial.println(DIR1);
  // Serial.print("PWM2 "); Serial.print(PWM2); Serial.print(" "); Serial.println(DIR2);
  // Serial.print("PWM3 "); Serial.print(PWM3); Serial.print(" "); Serial.println(DIR3);
  // Serial.print("PWM4 "); Serial.print(PWM4); Serial.print(" "); Serial.println(DIR4);
  
  analogWrite(pwm1,PWM1);
  analogWrite(pwm2,PWM2);
  analogWrite(pwm3,PWM3);
  analogWrite(pwm4,PWM4);
  digitalWrite(dir1,DIR1);
  digitalWrite(dir2,DIR2);
  digitalWrite(dir3,DIR3);
  digitalWrite(dir4,DIR4);
// digitalWrite(pwm1,150);
//   digitalWrite(pwm2,150);
//   digitalWrite(pwm3,150);
//   digitalWrite(pwm4,150);
//   digitalWrite(dir1,DIR1);
//   digitalWrite(dir2,DIR2);
//   digitalWrite(dir3,DIR3);
//   digitalWrite(dir4,DIR4);
  // digitalWrite(pwm1,0);
  // digitalWrite(pwm2,0);
  // digitalWrite(pwm3,0);
  // digitalWrite(pwm4,0);
  // digitalWrite(dir1,DIR1);
  // digitalWrite(dir2,DIR2);
  // digitalWrite(dir3,DIR3);
  // digitalWrite(dir4,DIR4);
  

}
