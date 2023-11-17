int PIDmotor1(float rpm_Right, float rpm_required){
  error[0] = rpm_required - rpm_Right;
  queue1[i]=error[0]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue1[n];     
  } 
   
  errorFunc[0] = 0.9*error[0] + 0.1*(preverror[0] - error[0])+ 0.01*error_sum;     //PID
  newerror[0] = (127*errorFunc[0])/424;
  preverror[0] = error[0];
  //Serial.println(newerror);
  return newerror[0];

}

int PIDmotor2(float rpm_Left, float rpm_required){
  error[1] = rpm_required - rpm_Left;
  queue2[i]=error[1]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue2[n];     
  } 
   
  errorFunc[1] = 0.9*error[1] + 0.1*(preverror[1] - error[1])+ 0.01*error_sum;     //PID
  newerror[1] = (127*errorFunc[1])/424;
  preverror[1] = error[1];
  //Serial.println(newerror);
  return newerror[1];

}

int PIDmotor3(float rpm_Right1, float rpm_required){
  error[2] = rpm_required - rpm_Right1;
  queue3[i] =error[2]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue3[n];     
  } 
   
  errorFunc[2] = 0.9*error[2] + 0.1*(preverror[2] - error[2])+ 0.01*error_sum;     //PID
  newerror[2] = (127*errorFunc[2])/424;
  preverror[2] = error[2];
  //Serial.println(newerror);
  return newerror[2];

}

int PIDmotor4(float rpm_Left1, float rpm_required){
  error[3] = rpm_required - rpm_Left1;
  queue4[i]=error[3]; 
  i++;
  if (i>=10){
       i=0;    
  } 
  int error_sum=0;    
  for(int n=0;n<qsize;n++)
  {
    error_sum = error_sum + queue4[n];     
  } 
   
  errorFunc[3] = 0.9*error[3] + 0.1*(preverror[3] - error[3])+ 0.01*error_sum;     //PID
  newerror[3] = (127*errorFunc[3])/424;
  preverror[3] = error[3];
  //Serial.println(newerror);
  return newerror[3];

}
