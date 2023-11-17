//__________________________________________________Global Variables____________________________________________
float prevReading=220;
//____________________________________________________Functions___________________________________________________
int lineControl(int numLSA)
{
  float vel;
  
  readingLSA[numLSA]=GetLSAReading(numLSA);
  
  if(abs(readingLSA[numLSA])<=35)
   prevReading=readingLSA[numLSA];
  
  if(numLSA==0)
   vel = ppidLSA080->pidControl(prevReading);
  else
   vel = ppidLSA081->pidControl(prevReading);
   

  int sign = abs(vel)/vel;
  float angleMove = (1-numLSA)*((1-sign)/2)*pi+numLSA*(pi+sign*pi/2);
//  printLCD("hehe","");
  
  calculateSpeed(-omegacontrol,angleMove,abs(vel));

  if(prevBotX - BotX == 0 && prevBotY - BotY == 0 && abs(readingLSA[numLSA]) < 5)
  {
    calculateSpeed(-omegacontrol,0,0);
    return 0;
  }

  return -1;
}

int lineFollow(int numLSA, float vel)
{
  readingLSA[numLSA] = GetLSAReading(numLSA);
  
  if(abs(readingLSA[numLSA]) <= 35)
    prevReading = readingLSA[numLSA];
  
  float perpendicularVel = ppidLSA081->pidControl(prevReading);
  float angleMove = atan2(-perpendicularVel,vel*(-arenaFlag));
 
  if(angleMove < 0)
    angleMove = 2*pi + angleMove;
  
  calculateSpeed(-omegacontrol,angleMove,sqrt((vel)*(vel)+(perpendicularVel)*(perpendicularVel)));

  if(digitalRead(LSAArray[1]->JunctionPin))
    return -1;
 return 1;
}
