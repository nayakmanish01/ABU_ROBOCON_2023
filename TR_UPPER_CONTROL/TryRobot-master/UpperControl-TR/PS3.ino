//__________________________________________________Global Variables____________________________________________

double LeftAnalogTheta = 0, RightAnalogTheta = 0;
double LeftAnalogDistance = 0, RightAnalogDistance = 0;
int buttonFlag = 0,tryspotnum = -1;
//_______________________________________________________________________________________________________________

//____________________________________________________Functions__________________________________________________

void operateManually()
{
  uint16_t orientationAngle = 0;
  float omegaSpeed = 1.8;     
  
      
      if(LeftAnalogDistance<=0.01 && RightAnalogDistance<=0.01)
        brakeWheels();
      else
        calculateSpeed(-omegacontrol,DegreeToRadian((LeftAnalogTheta- orientationAngle)),LeftAnalogDistance* manualSpeed); 
}  

void receivePS3data(){

    while(ET_ps3.receiveData()>0)
    {
      PS3Button = int(data.ps3data);
      LeftAnalogDistance = data.LD;
      LeftAnalogTheta = data.LT;
      RightAnalogDistance = data.RD;
      RightAnalogTheta = data.RT; 
//      Serial.println(PS3Button);
    }  
}

void PS3ExecutePressed()
{
  if(PS3Button == CIRCLEx)
  {
    FlagOmegaControl = 0;
  }
  if(PS3Button == TRIANGLEx)
  {
    startPlanning = 1;
    FlagOmegaControl = 1;
  }
  if(PS3Button == CROSSx)
  {
    startPlanning = 0;
    ppidOmega->required = robotYaw;
  }
  if(!startPlanning)
  {
    if(PS3Button == UPx)
    {
      initTask(trySpotX[1],trySpotY[1]);
      tryspotnum = 1;
    }
    if(PS3Button == DOWNx)
    {
      initTask(trySpotX[3],trySpotY[3]);
      tryspotnum = 3;
    }
    if(PS3Button == LEFTx)
    {
      initTask(trySpotX[0],trySpotY[0]);
      tryspotnum = 0;
    }
    if(PS3Button == RIGHTx)
    {
      initTask(trySpotX[2],trySpotY[2]);
      tryspotnum = 2;
    }
    if(PS3Button == SQUAREx)
    {
      initTask(arenaFlag*BotX,shiftY);
      tryspotnum = 2;
    }
  }
}
///////////////////////////////////////////////////////////////////                            PS3 ENDED                            ///////////////////////////////////////////
