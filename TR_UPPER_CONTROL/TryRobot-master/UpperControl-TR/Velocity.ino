//__________________Global Variables______________________
float dX = 0, dY = 0;
float minV = 0.0, velOutput = 0.0;

//____________________Functions___________________________
void calculatebotTheta()
{
    dX = (BotX - prevBotX);
    dY = (BotY - prevBotY);
   
    vBot = sqrt(dX*dX + dY*dY)/50;     
} 

float stoppingDist(float remDist,float distCover, float currVel, float accVel, float passVel, float stopDist, int SpeedMode, int Gmode)
{
  switch (SpeedMode)
      {
        case accdec :
      
              if(remDist < stopDist)
              {
                  if(vBot*1000 > passVel+50)
                  {
                    velOutput = 0; FlagOmegaControl = 0;
                  }
              
                  else if(Gmode == STOP)
                  {
                    minV = 40;
                    velOutput = (200 - minV)*remDist/60+ minV; 
                    FlagOmegaControl= 1;
                  }
                  else
                {
                    velOutput = passVel;    
                    FlagOmegaControl= 1;   
                }
            }
            else if(distCover < 80)
            {
              velOutput = (currVel-accVel)*distCover/80 + accVel;
              FlagOmegaControl= 1;
            }
            else
            {
              velOutput = currVel;
              FlagOmegaControl= 1;
            }

        break ;
        case accelerate :
              
            if(distCover < 80)
            {
              velOutput = (currVel-accVel)*distCover/80 + accVel;
              FlagOmegaControl= 1;
            }
            else
            {
              velOutput = currVel;
              FlagOmegaControl= 1;
            }
        break;
        case decelerate :
            if(remDist < stopDist)
            {
                if(vBot*1000 > passVel+70)
                {
                  velOutput = 0; FlagOmegaControl = 0;
                }
            
                else if(Gmode == STOP)
                {
                  minV = 40;
                  velOutput = (passVel - minV)*remDist/60+ minV; 
                  FlagOmegaControl= 1;
                }
                else
              {
                  velOutput = passVel;    
                  FlagOmegaControl= 1;   
              }
            }
            else
            {
              velOutput = currVel;
              FlagOmegaControl= 1;
            }
        break;
        
        case constant : 
             velOutput = currVel;
             FlagOmegaControl= 1;
        break;
      }
            
  return velOutput;
}
