void PID::initPID(float kp,float kd,float ki,float req,float minV,float maxV)
  {
    Kp=kp;
    Kd=kd;
    Ki=ki;
    Kp_old=kp;
    Kd_old=kd;
    Ki_old=ki;
    required=req;
    maxControl=maxV;
    minControl=minV;
    error=0;
    prevError=0;
    derivativeError=0;
    integralError=0;
    prevRequired=req;
  }

float PID::pidControl(float actual)
{
    error = required - actual;
    derivativeError = error - prevError;
    prevError = error;
    float Output = Kp*error + Kd*derivativeError + Ki*integralError;
   
    
    if(Output > maxControl)
      {
        Output = maxControl;
      }
    else if(Output < minControl)
      {
        Output = minControl;
      }
    else if(fabs(error) < 5)
      integralError = integralError + error;
      
    return Output;
}
