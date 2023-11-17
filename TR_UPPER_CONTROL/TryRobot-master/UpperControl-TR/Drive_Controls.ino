int signOmega = 0;

void calculateSpeed(float omega , float angle , float Vtranslational)     //Omega rad/s      Vtranslational  m/s
{
  angle+=DegreeToRadian(robotYaw);
  if(angle < 0)
    angle += 2*pi; 

  mydata.omega = omega;
  mydata.theta = angle;
  
  if(Vtranslational > maxWheelRPM)
    Vtranslational = maxWheelRPM;
    
  mydata.velocity = Vtranslational;
    
  ETUpperLower.sendData();  
}

float OmegaControlMPU(float yaw,float tolerence)
{
  float omegacontrol=0;
  float error = ppidOmega->required - yaw;

  if(fabs(error) < tolerence)
    {
      if( mydata.velocity == 0)
        omegacontrol = 0;
      else
        omegacontrol = ppidOmega->Ki*ppidOmega->integralError;
      
      integralFlag = 1;
      redLEDon;
    }
  else 
    {
      if(integralFlag)
      {
        ppidOmega->integralError = 0;
        integralFlag = 0;
      }
        
      omegacontrol = ppidOmega->pidControl(yaw);
      if( mydata.velocity == 0)
        omegacontrol -= ppidOmega->Ki*ppidOmega->integralError;
      redLEDoff;
    }
  return omegacontrol;
}
//float OmegaControlMPU(float yaw,float tolerence)
//{
//  float omegacontrol=0;
//  float error = ppidOmega->required - yaw;
//  
//  if(fabs(error) > tolerence && fabs(error)<4)
//    {
//      omegacontrol = ppidOmega->pidControl(yaw) - ppidOmega->Kp*ppidOmega->error - ppidOmega->Kd*ppidOmega->derivativeError;
//      redLEDoff;
//    }
//    else if(fabs(error) < tolerence)
//    {
//      omegacontrol = 0;
//      ppidOmega->integralError = 0;
//      redLEDon;
//    }
//    else
//    {   
//      omegacontrol = ppidOmega->pidControl(yaw);
//      redLEDoff;
//    }
//  return omegacontrol;
//}
