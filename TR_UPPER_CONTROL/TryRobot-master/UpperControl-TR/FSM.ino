int fsmFlag = 1, buzzCount=0, fsmCount = 0;
void finiteStateMachine()
{
    if(PS3Button == R1x)
    {
       fsmState = manualDrive;
       lcd.clear();  
    }
    if(PS3Button == SELECTx )
    {
      fsmState = variableState;
       printLCD(String(fsmFlag) + "    " + String(BotY)+"   ", String(robotYaw));
    }
  switch(fsmState)
  {
    case arenaSelect:

      if(PS3Button == R2x)
        {
            arenaFlag=RED;

            BotX = arenaFlag*shiftX;
            BotY = shiftY;
            fsmState = waitForRefree;
            printLCD("RED ARENA"," ");
           } 
        if(PS3Button == L2x)
        {
              arenaFlag=BLUE;
            
              BotX = arenaFlag*shiftX;
              BotY = shiftY;
              fsmState = waitForRefree;
              printLCD("BLUE ARENA"," ");
        }
    
    break;
    
    case waitForRefree:
//         printLCD("Waiting for signal","");
    
           if(PS3Button == TRIANGLEx)
           { 
             lcd.clear();
             initTask(receiveX,receiveY);
//             fsmState = receiveBall;
             fsmState = variableState;
             resetAngle = (1 - arenaFlag)*90;
             printLCD("Go to receive spot"," ");
           }
    break;

    case receiveBall:
//          printLCD(String(counterxy)+" "+String(counter),String(RadianToDegree(resetAngle)));
          if(buzzCount<500)
          {
            buzzOn;
            buzzCount++;
          }
          else
            buzzOff;
          
          if(fsmFlag)
          {
            fsmFlag = planPath(startX,startY,goalX,goalY,100);
            resetSx = BotX;
            resetSy = BotY;
          }
          else
          {
           if(resetFlag)
            resetFlag = cornerReset((1000 - shiftX)*arenaFlag,shiftY,receiveZone);
           else
           {
              calculateSpeed(0,0,0);
              fsmState = Collaborate;
           }
          }
    break;

    case Collaborate:
//          collaborate(150,0);
//          if(!collaborate)
           selectTrySpot();           
    break;

    case placeBall:
          fsmFlag = planPath(startX,startY,goalX,goalY,200);
//          printLCD(String(goalX)+" "+String(goalY),"Place Ball");
          if(!fsmFlag)
          {
            fsmState = resetState;
            fsmFlag=1;
            resetSx = BotX;
            resetSy = BotY;
          }
           
    break;

    case resetState:
            


//       printLCD(" "," Reset");

       if(fsmFlag  == 1)
        fsmFlag = yawReset();  
       
       if (fsmFlag == -1)
       {
        if(digitalRead(proximityPin))
        {
         fsmFlag = 3;
         printLCD(String(fsmFlag),"PROXIMITY ONN");
        }
        else
        {
         fsmFlag = 4;
          printLCD("","PROXIMITY OFF");
        }       
       } 
       else if(fsmFlag == 3)//LEFT
       {
//        calculateSpeed(-omegacontrol,0,100);
        traceLine(BotX,BotY,BotX-(10),BotY + 1,150,constant,0,0,STOP,2);
        
        if(!digitalRead(proximityPin))
          fsmFlag=2;
       }
       else if(fsmFlag == 4)//RIGHT
       {
//        calculateSpeed(-omegacontrol,pi,100);
         traceLine(BotX,BotY,BotX + (10),BotY + 1,150,constant,0,0,STOP,2);
       
       if(digitalRead(proximityPin))
        fsmFlag=2;
       }
       else if(fsmFlag == 2)
       {
          BotX = goalX;
          calculateSpeed(0,0,0);
          fsmState = variableState; 
//          printLCD("","           Gotchaaa");
       }
         
    break;     

    case variableState:
              printLCD("","variable          ");
              calculateSpeed(-omegacontrol,0,0);
             
//               if(PS3Button == TRIANGLEx)
//               {
//                   initTask(receiveX,receiveY);
//                   fsmState = receiveBall;
//                   resetAngle = (1 - arenaFlag)*90;
//                   resetFlag = 1;
//                   fsmFlag = 1; 
//               }
               if(PS3Button == CIRCLEx)
               {
                   initTask(kickingSetupX,kickingSetupY);
                   fsmState = kickBall;
                   resetAngle= 0;
                   ppidOmega->required = 90;
                   fsmFlag = 1;
                   planState = checkCollision;
               }
               if(PS3Button == SQUAREx)
               {
                  initTask(shiftX+25,shiftY+25 );
                  fsmState = goHome;
                  resetAngle = (1 + arenaFlag)*90;
                  resetFlag = 1;
                  fsmFlag = 1;
               }
           
     break;
    case goHome:
          if(fsmFlag)
          {
            fsmFlag = planPath(startX,startY,goalX,goalY,100);
            resetSx = BotX;
            resetSy = BotY;
          }
          else
          { 
           if(resetFlag)
           {
            resetAngle = (1 + arenaFlag)*90;
            resetFlag = cornerReset(arenaFlag*shiftX,shiftY,startZone)  ;
           }
           else
           { 
             if(ps2Flag)
             {
              if(PS3Button == CIRCLEx)
              {
                  lcd.clear();
                  fsmState = kickBall;
                  initTask(kickingSetupX,kickingSetupY);
                  planState = checkCollision;              
              }
             }
            } 
          }
          
    break;

    case kickBall:
//            printLCD("KICKING      ","");
        if(fsmFlag == 1||fsmFlag==2)
        {
          if(!planPath(startX,startY,goalX,goalY,200))
          {
            initTask(kickingX,kickingY);
            fsmFlag++;
            resetAngle = (1-arenaFlag)*90;
            resetSx = BotX;
            resetSy = BotY;
            resetFlag = 1;
          }
        }
        if(fsmFlag==3)
        {  
          
           if(resetFlag)
            resetFlag = cornerReset((kickingX)*arenaFlag,kickingY,kickingZone);
           else
           {
              calculateSpeed(0,0,0);
              fsmState = alignForKick;
//              initialRobotYaw =  mympu.robotYaw - 90;
              ppidOmega->required = 107.5;
              printLCD("Select Strategy        ","");
              planState = checkCollision;
           }
        }
            
    break;
    case alignForKick:
          if(fsmFlag==2)
          {
            calculateSpeed(0,DegreeToRadian(30),100);
            counter++;
            if(counter>150)
            {
              fsmFlag= 1;
               printLCD("    "," Done Aligning    ");
               fsmState = variableState;
               counter =0;
            }
            
          }
          else
          {
          calculateSpeed(0,DegreeToRadian(80),100);
          if(!botStop())
          {
            counter=0;
            fsmFlag = 2;
           
          }
          }
      
    break;

    case manualDrive:
//          printLCD("MANUAL         ","");
          operateManually();
    break;
  }
}

void selectTrySpot()
{
//  if(ps2Flag)
  {
    if(PS3Button == LEFTx)
    {
       lcd.clear();
       initTask(trySpotX[0],trySpotY[0]);
       fsmState = placeBall;
    }
    if(PS3Button == UPx)
    {
       lcd.clear();
       initTask(trySpotX[1],trySpotY[1]); 
       fsmState = placeBall;     
    }
    if(PS3Button == RIGHTx)
    {
       lcd.clear();
       initTask(trySpotX[2],trySpotY[2]);
       fsmState = placeBall;
    }
    if(PS3Button == DOWNx)
    {
       lcd.clear();
       initTask(trySpotX[3],trySpotY[3]);
       fsmState = placeBall;
    }
    if(PS3Button == L1x)
    {
       lcd.clear();
       initTask(trySpotX[4],trySpotY[4]-15);
       fsmState = placeBall;
    }
  }
}
