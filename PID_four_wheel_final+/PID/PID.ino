int pid(int s, float m){

  new_rot_speed = m;            // Current speed
  error = s - new_rot_speed;            // error 
  queue[i]=error;                       // Queue is updated
  i++;   
  //Serial.println(error);

           
  if (i>=10){                           // Previous elements being deleted after 10 elements are added  
       i=0;                             
   }        
   
            
   int error_sum=0;    
   for(n=0;n<qsize;n++)
   {
        error_sum = error_sum + queue[n];                      // Calculating the sum of current and all previous errors
   } 

   
   error = s - new_rot_speed;                                              // Calculating error
   errorFunc = 0.9*error + 0.1*(preverror - error)+ 0.01*error_sum;        // PID
   newerror = (127*errorFunc)/200;                                         // Final speed/error to be given to the motors 
   preverror = error;                                                      // Updating the previous error 
   Serial.println(newerror);                                               // Printing the final data to be given
   return newerror;                                                        // Returning the final data
   
}
