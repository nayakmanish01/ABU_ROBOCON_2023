void left1_wheel_pulse() {
  
  int val = digitalRead(ENC1_IN_LEFT_B);                      // Read the value for the encoder for the left wheel ST 128_left
 
  if(val == LOW) {
    Direction_left1 = false;          // Reverse
  }
  else {
    Direction_left1 = true;           // Forward
  }
   
  if (Direction_left1) {              // Updating the new encoder data for direction
    left1_wheel_pulse_count++;
  }
  else {
    left1_wheel_pulse_count--;
  }
}


void right_wheel_pulse() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);                      // Read the value for the encoder for the right wheel ST 128_right
  
  if(val == LOW) {
    Direction_right = false;        // Reverse
  }
  else {
    Direction_right = true;         // Forward
  }
   
  if (Direction_right) {            // Updating the new encoder data for direction
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}


void left_wheel_pulse() {
   
  int val = digitalRead(ENC_IN_LEFT_B);                        // Read the value for the encoder for the left wheel ST 129_left
 
  if(val == LOW) {
    Direction_left = false;               // Reverse
  }
  else {
    Direction_left = true;                // Forward
  }
   
  if (Direction_left) {                   // Updating the new encoder data for direction
    left_wheel_pulse_count++;
  }
  else {
    left_wheel_pulse_count--;
  }
}

void right1_wheel_pulse() {
   
  int val = digitalRead(ENC1_IN_RIGHT_B);                      // Read the value for the encoder for the right wheel ST 129_right
 
  if(val == LOW) {
    Direction_right1 = false;            // Reverse
  }
  else {
    Direction_right1 = true;             // Forward
  }
   
  if (Direction_right1) {                // Updating the new encoder data for direction
    right1_wheel_pulse_count++;
  }
  else {
    right1_wheel_pulse_count--;
  }
}
