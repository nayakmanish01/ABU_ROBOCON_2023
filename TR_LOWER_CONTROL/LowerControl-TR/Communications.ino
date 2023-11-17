uint8_t zeroCntr = 0;

void getUpperData(){
   while(ET.receiveData()>0)
   {  
    angularV = mydata.omega;
     linearV = mydata.velocity;
     angleTheta = mydata.theta;

     if(ledCntr < 50)
      blueLEDoff;
    else if(ledCntr >= 50 && ledCntr < 100 )
      blueLEDon;
    else
      ledCntr = 0;
    ledCntr++;
   }  
  
//    linearV = 200;
//    angleTheta = pi/2;
    calculateIK(angularV, angleTheta, linearV); 
}
