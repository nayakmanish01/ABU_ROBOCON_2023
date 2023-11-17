//__________________________________________________Global Variables____________________________________________
int cnt = 0, dir = 0, obCounter = 0, lastObj = 0;
int startI = 0, endI = 4, ranDom;
float xI = 0, yI = 0, xJ = 0, yJ = 0;
bool lineFlag = 1, collisionFlag = 1;
int fenceMode = 0;
//__________________________________________________Functions___________________________________________________
bool planPath(float x1, float y1, float x2, float y2, float vel)
{
  switch(planState)
  {
//    speedMode = constant;
    case checkCollision:
       
      generateWindow(x1,y1,x2,y2); // searching nearest obstacles

      lineFlag = 1;   // reseting flags, counters and array
      collisionFlag = 1;
      
      fenceMode = 0;
      
      obCounter = 0;
      lastObj = 0;
      
      dir = 0;
      cnt = 0;

      for(int i = 0; i < 5; i++) 
        detObjects[i] = -1;
        
      xI = x2;
      yI = y2;
      xJ = x1;
      yJ = y1;

      if(endI - startI != 0)
        ranDom = abs(endI - startI)/(endI - startI);
      else
        ranDom = 1;
      
      for(int i= startI; i*ranDom <= endI*ranDom ; i = i + ranDom) 
      { 
        if(collisionCheckGrowingCircle(x1,y1,x2,y2,arenaFlag*pole[i]->h,pole[i]->k,pole[i]->r))
        {  
            detObjects[obCounter] = i;
            obCounter++; 
            Serial.println(i);
        }
      }
       
      //checking for fence....
      if(obCounter == 0)
        {
          if(fabs(x2) > fabs(pole[4]->h) && fabs(y2) >= fabs(pole[4]->k))
          {
            detObjects[obCounter] = 4;
            obCounter++;
          }
          else if(fabs(x2) < fabs(pole[0]->h) && fabs(y2) >= fabs(pole[0]->k))
          {
            detObjects[obCounter] = 0;
            obCounter++;
          }
        }

      if(obCounter == 0)
      {
        planState = motionToGoal;
        speedMode = constant; // accdec
      }
      else
      {
        planState = avoidCollision;
        speedMode = constant;  // acceleration
      }
        
      lastObj = obCounter - 1;
       
      obCounter = -1;
      
     
      return 1;
    
    case avoidCollision:
      
      obCounter++;
      cnt = detObjects[obCounter];

      if(cnt == 4)
        fenceMode = arenaFlag*1; //left fence(long)
      else if(cnt == 0)
        fenceMode = arenaFlag*-1;  //right fence
      else
        fenceMode = 0;
      
      
      avoidGrowingCircle(xJ,yJ,x2,y2,arenaFlag*pole[cnt]->h,pole[cnt]->k,pole[cnt]->r);
      Serial.println(String(xI)+" "+String(yI));
      planState = nearObstacle;
      
      return 2;
      
    case nearObstacle:
      lineFlag = traceLine(xJ,yJ,xI,yI,vel,constant,150,250,CONTINUE,15); // speedMode = constant  //tol = 15
      if(lineFlag)
        planState = nearObstacle;
      else
      {
        planState = aroundObstacle;
        lineFlag = 1;
        collisionFlag = 1;
      }
        return 3;

     case aroundObstacle:
      
      if(collisionFlag)
      {
        collisionFlag = collisionCheckGrowingCircle(BotX,BotY,x2,y2,arenaFlag*pole[cnt]->h,pole[cnt]->k,pole[cnt]->r-10);
        bool arcFlag = traceCircle(arenaFlag*pole[cnt]->h,pole[cnt]->k,pole[cnt]->r,dir,250);
        xJ = BotX;
        yJ = BotY;
      }
      else
      {
          if(lastObj != obCounter)
            planState = avoidCollision;
          else
          {
            planState = motionToGoal;
            speedMode = constant; //decelerate
            
            xI = x2;
            yI = y2; 
          }   
      }        
      return 4;

      case motionToGoal:
        lineFlag = traceLine(xJ,yJ,xI,yI,vel,decelerate,0,150,STOP,10); // speedMode = constant  // tol = 15
//        Serial.println(obCounter);
        greenLEDon;
        return lineFlag;
  }
}
