
//__________________________________________________Global Variables____________________________________________
int stopCounter = 0, counterYaw = 0, counter = 0;
//____________________________________________________Functions___________________________________________________
float dist(float x1,float y1,float x2,float y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float Angle(float x1,float y1,float x2,float y2)
{
  float angle=atan2(y2-y1,x2-x1);
  if(angle<0)
    return 2*pi + angle;
  else
    return angle;
}

void printLCD(String data1, String data2)
{
  lcd.setCursor(0,0); // Sets the cursor to col 0 and row 0
  lcd.print(data1);
  lcd.setCursor(0,1); // Sets the cursor to col 1 and row 0
  lcd.print(data2);
}

void brakeWheels(){
  calculateSpeed(-omegacontrol,0,0);
}

int yawReset()
{
   traceLine(resetSx,resetSy,resetSx,resetSy+55,100,constant,0,0,STOP,0); // BotX BotY
   counterYaw++;//counterYaw1 ++;
  
   if(counterYaw >= 100)//(counterYaw1>=100)
   { 
    if(fabs(prevBotX-BotX) <= 0.1 && fabs(prevBotY-BotY) <= 0.1) // < 0.1
      counterYaw++;
    else 
      counterYaw = 100;
    
    
    if(counterYaw >= 200)
    {
      initialRobotYaw += robotYaw;
      //counterYaw1 = 0;
      counterYaw = 0;
      
      BotY = 579.5; //567.5
      return -1;
    }
   } 
   return 1;
}

bool botStop()
{
  if(fabs(BotX - prevBotX) < 0.01 && fabs(BotY - prevBotY) < 0.01)
  {  
    stopCounter++;
    if(stopCounter > 100)
    {
     stopCounter = 0;
     return 0;
    }
  }
  else
    stopCounter = 0;
  return 1;
}
bool cornerReset(float resetX, float resetY, int zone)
{
  switch(resetAngle)
  {  
    case 0 :
      traceLine(resetSx, resetSy, resetSx + 1000, resetSy, 120, constant,0,0,STOP,0);
      if(!botStop())
      {
        
        resetSx = BotX;
        resetSy = BotY;
//        initialRobotYaw = mympu.robotYaw - ppidOmega->required;
        counter = 0;
        if(zone == 0)
        {
        resetAngle = 90;
        ppidOmega->required = 107.5;
        }
        else
        resetAngle = 270;
      }
     break;

    case 180 :
      traceLine(resetSx, resetSy, resetSx - 1000, resetSy, 100, constant,0,0,STOP,0);
      if(!botStop())
      {
        resetSx = BotX;
        resetSy = BotY;
        if(zone == 0)
        resetAngle = 90;
        else
        resetAngle = 270;
      }
      break;

    case 90 :
        traceLine(resetSx-15, resetSy, resetSx-15, resetSy + 1000, 150, constant,0,0,STOP,0);
        counter++;
        if(stopCounter<70)
        stopCounter = 70;
        Serial1.println(String(counter) + "    " + String(stopCounter) +"  " +String(robotYaw));
        if(counter>150)
        {
       if(!botStop())
      {
        resetSx = BotX;
        resetSy = BotY;
//        resetAngle = 45;
        counter = 0;
        return 0;
      }
        }
      break;

    case 270 :
      traceLine(resetSx, resetSy, resetSx, resetSy - 1000, 100, constant,0,0,STOP,0);
      if(!botStop())
      {  
        resetSx = BotX;
        resetSy = BotY;
        if(arenaFlag*zone == 1)
          resetAngle = 315;
        else
          resetAngle = 225;
      }
      break;
      
    case 45 :
    traceLine(resetSx, resetSy, resetSx + 1000, resetSy + 1000, 100, constant,0,0,STOP,0);
    if(!botStop())
    {
      BotX = resetX;
      BotY = resetY;
//        initialRobotYaw+=robotYaw;
      return 0;
    }
      break;
      
    case 225 :
      traceLine(resetSx, resetSy, resetSx - 1000, resetSy - 1000, 100, constant,0,0,STOP,0);
      if(!botStop())
      {
        BotX = resetX;
        BotY = resetY;
//        initialRobotYaw+=robotYaw;
        return 0;
      }
      break;

    case 315 :
      traceLine(resetSx, resetSy, resetSx + 1000, resetSy - 1000, 100, constant,0,0,STOP,0);
      if(!botStop())
      {
        BotX = resetX;
        BotY = resetY;
//        initialRobotYaw+=robotYaw;
        return 0;
      }
      break;
  }
  return 1;
}

void initTask(float x2,float y2)
{
  startX = BotX;
  startY = BotY;

  goalX = arenaFlag*x2;
  goalY = y2;

  planState = checkCollision;
}

//_______________________________INTERRUPTS______________________________________________
void returnCountX()
{
  if(digitalRead(pEncoderX->channel2))
    pEncoderX->Count++;
  else
    pEncoderX->Count--; 
}

void returnCountY()
{
  if(digitalRead(pEncoderY->channel2))
    pEncoderY->Count--;
  else
    pEncoderY->Count++;
}
