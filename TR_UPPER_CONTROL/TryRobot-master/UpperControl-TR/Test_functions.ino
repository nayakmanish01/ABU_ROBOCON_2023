void printXY_Yaw()
{   
Serial.print("Yaw(degree) "+String(robotYaw)+ " ");
Serial.print((int)pBot->X_pos);
Serial.print("   ");
Serial.print((int)pBot->Y_pos);
Serial.print("   ");
Serial.println(millis()/1000);

}
void printCount()
{
  Serial.print("      X (count):   "+String((int)pEncoderX->Count));
  Serial.println("    Y (count):   "+String((int)pEncoderY->Count));
}

void printRtheta()
{
  Serial.print("r (cm): "+String(sqrt(pBot->X_pos*pBot->X_pos + pBot->Y_pos*pBot->Y_pos)));
  Serial.println("theta(degree) " + String(DegreeToRadian(atan2(pBot->Y_pos,pBot->X_pos))));
}
