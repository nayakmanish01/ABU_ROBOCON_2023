void driveMotorReq(float op,PID *pPIDMotor, Motor *pMotor, float maxval)
{
  if(pPIDMotor->required > 0)
  {
    digitalWrite(pMotor->direction1,HIGH);
    digitalWrite(pMotor->direction2,LOW);
  }
  else if(pPIDMotor->required < 0)
  {
    digitalWrite(pMotor->direction2,HIGH);
    digitalWrite(pMotor->direction1,LOW);
  }
  else if(pPIDMotor->required == 0 || op == 0)
  {
//    if(!softBrake)
//    {
//      digitalWrite(pMotor->direction2,HIGH);
//      digitalWrite(pMotor->direction1,HIGH);
//    }
//    else
//    {  
//      digitalWrite(pMotor->direction2,LOW);
//      digitalWrite(pMotor->direction1,LOW);
//    }
  }

  
  if(op > maxval)
    op = maxval;

  op = (op*maxPWM)/maxval;
  
  pwm.pinDuty(pMotor->pwmPin,(int)op);
}
