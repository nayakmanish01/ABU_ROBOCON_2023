bool collaborate(float constX, float constY)
{
  ppidCollaborateX->required = agentPR.x - (abs(agentPR.x)/agentPR.x)*constX;
  ppidCollaborateY->required = agentPR.y - (abs(agentPR.y)/agentPR.y)*constY;
  ppidOmega->required = agentPR.yaw;

  float vX = ppidCollaborateX->pidControl(BotX);
  float vY = ppidCollaborateY->pidControl(BotY);

  float vBot = sqrt(vX*vX + vY*vY);
  float angleMove = atan2(vY,vX);

  calculateSpeed(-omegacontrol,angleMove,vBot);

  if( abs(ppidCollaborateY->required) < 3 && vBot < 10)
   return 0;

  return 1;
}



