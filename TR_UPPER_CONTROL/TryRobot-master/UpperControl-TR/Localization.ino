//____________________Functions___________________________
void handleXY(void)
{    
    
    pBot->prev_X = pBot->X_pos1;    
    pBot->prev_Y = pBot->Y_pos1; 
      
    pBot->X_pos1 = (pEncoderX->Count * 2 * pi * RadiusXYWheel / pEncoderX->ppr); //cm   
    pBot->del_x = pBot->X_pos1 - pBot->prev_X ;   
  
     
    pBot->Y_pos1 = (pEncoderY->Count * 2 * pi * RadiusXYWheel / pEncoderY->ppr); //cm
    pBot->del_y = pBot->Y_pos1 - pBot->prev_Y ;
    
    float r = dist(0,0,pBot->del_x,pBot->del_y);
    float Theta = the(pBot->del_y,pBot->del_x,robotYaw+135);
       
    prevBotX = BotX;
    prevBotY = BotY;
      
    BotX = BotX + r*(cos(Theta));
    BotY = BotY + r*(sin(Theta));

}

float the(float del_y,float del_x,float botYaw)
{
  botYaw = int(botYaw*10);
  botYaw = float(botYaw)/10.0;
  return (atan2(del_y,del_x) - DegreeToRadian(botYaw));
}
