bool collisionCheckGrowingCircle(float x1, float y1, float x2, float y2, float h, float k, float r)
{
  bool spCase = 0;
  float angleSG = atan2(y2-y1,x2-x1);
  float angleSC = atan2(k-y1,h-x1);

  float m,c,alpha,beta,gamma;
  float deL = -1;
  
  if(fabs(angleSG) == pi/2)
  {
    if(x1>h-r && x1<h+r)
      spCase = 1;
    c = 0;
  }
  else
  {  
    if(angleSG < 0)
      angleSG = angleSG + 2*pi;
    if(angleSC < 0)
      angleSC = angleSC + 2*pi;
  
    m = (y2-y1)/(x2-x1); 
    c = y1 - x1*m;
  
    alpha = 1 + m*m;
    beta = 2*(h - m*c + m*k);
    gamma = r*r - h*h - k*k - c*c + 2*c*k;

    deL = beta*beta + 4*alpha*gamma;
  }
  
  
  if(deL>=0 || spCase)
    return 1;
  return 0;
}
void avoidGrowingCircle(float x1, float y1, float x2, float y2, float h, float k, float r)
{
    float angleTan, mt, ct, theta, cn;
    
    float angleSG = atan2(y2-y1,x2-x1);
    float angleSC = atan2(k-y1,h-x1);
   
    theta = asin(r/dist(x1,y1,h,k));

    if(angleSG < 0)
       angleSG = angleSG + 2.0*pi;
    if(angleSC < 0)
       angleSC = angleSC + 2.0*pi;
  
    int sign = fabs(fabs(angleSG-angleSC)-pi)/(pi-fabs(angleSG-angleSC));

    switch(fenceMode)
    {
      case 0:
        if(angleSG > angleSC)
        {
          angleTan = angleSC + sign*theta;
          dir = clockwise;
        }
        else
        {
          angleTan = angleSC - theta;
          dir = antiClockwise;
        }
        break;

      case -1:
        if(y2 > y1)
        {
          angleTan = angleSC - theta;
          dir = antiClockwise;
        }
        else
        {
          angleTan = angleSC + theta;
          dir = clockwise;
        }
      break;

      case 1:
        if(y1 > y2)
        {
          angleTan = angleSC - theta;
          dir = antiClockwise;
        }
        else
        {
          angleTan = angleSC + theta;
          dir = clockwise;
        }
      break;
    }

    mt = tan(angleTan);
    ct = y1 - mt*x1;
    cn = k + h/mt;

    xI = ((cn - ct)*mt)/(1 + mt*mt);
    yI = xI*mt + ct;
//    return 0;
}
bool generateWindow(float x1, float y1, float x2, float y2)
{
  int index[2] = {-1,-1};

  float xLine;
  
  int sign = 1;
  
  if(x1 != x2)
    sign = fabs(x1 - x2)/(x1 - x2);
    
    for(int i = 0; i < 5; i++)
    {
      xLine = arenaFlag*pole[i]->h;

     if((xLine >= x1 + sign*pole[i]->r && xLine <= x2 - sign*pole[i]->r) || (xLine <= x1 + sign*pole[i]->r && xLine >= x2 - sign*pole[i]->r))
      {
        if(index[0] == -1)
        {
          index[0] = i;
          index[1] = i;
        }
        else
          index[1] = i;
      }
    }
  
  if(index[0] == -1)
  {
    startI = 0;
    endI = 0;
    return 0;
  }
 
  if(dist(BotX,BotY,arenaFlag*pole[index[0]]->h,pole[index[0]]->k) < dist(BotX,BotY,arenaFlag*pole[index[1]]->h,pole[index[1]]->k))
  {
    startI = index[0];
    endI = index[1];
  }
  else
  {
    startI = index[1];
    endI = index[0];
  }
//  Serial.println(String(startI)+" "+String(endI));
  return 1;
}
