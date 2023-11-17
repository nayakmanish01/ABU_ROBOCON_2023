void set_rpm(float rpm)
{
   pPIDMotor[0]->required = rpm;
   pPIDMotor[1]->required = -rpm;
   pPIDMotor[2]->required = -rpm;
   pPIDMotor[3]->required = rpm;
}

void motorTest(float pWM)
{
  for(int i=0;i<N;i++)
  {
    digitalWrite(pMotor[i]->direction1,HIGH);
    digitalWrite(pMotor[i]->direction2,LOW);
    pwm.pinDuty(pMotor[i]->pwmPin,pWM);
//    analogWrite(pMotor[i]->pwmPin,pWM);
  }
}

void plotRPM()
{
//  SerialX.print(abs(pEncoder[0]->rpm));
//  SerialX.print(" ");
//  SerialX.print(abs(pEncoder[1]->rpm));
//  SerialX.print(" ");
//  SerialX.println(abs(pEncoder[2]->rpm));
//  SerialX.print(" ");
  SerialX.println(abs(pEncoder[0]->rpm));
}

void printCount()
{
  SerialX.println(String((int)pEncoder[0]->Count)+"  "+String((int)pEncoder[1]->Count)+"  "+String((int)pEncoder[2]->Count));//+"  "+String((int)pEncoder[3]->Count)+"  ");
}

void setSlowly(int rpm, int d_rpm)
{
   pPIDMotor[0]->required += d_rpm;
   pPIDMotor[1]->required += -d_rpm;
   pPIDMotor[2]->required += -d_rpm;
   pPIDMotor[3]->required += d_rpm;
   
   if(abs(pPIDMotor[0]->required)>=rpm)
     {
       pPIDMotor[0]->required = rpm;
       pPIDMotor[1]->required = -rpm;
       pPIDMotor[2]->required = -rpm;
       pPIDMotor[3]->required = rpm;  
     }
    delay(100);   
}
