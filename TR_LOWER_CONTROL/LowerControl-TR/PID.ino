void PID::initPID(float kp,float kd,float ki,float req,float minV,float maxV)
{
  Kp=kp;
  Kd=kd;
  Ki=ki;
  required=req;
  maxControl=maxV;
  minControl=minV;
  error=0;
  prevError=1;
  derivativeError=0;
  integralError=0;
  prev_integralError=0;
  prevRequired=req;
}

float PID::pidControl(float actual)
{
  error = fabs(required) - fabs(actual);
  derivativeError = error - prevError;
  prevError = error;
  
  float Output = Kp*error + Kd*derivativeError + Ki*integralError;
  
  if(Output > maxControl)
    Output = maxControl;
  else if(Output < minControl)
    Output = minControl;
  else
    integralError = integralError + error;

  return Output;
}
