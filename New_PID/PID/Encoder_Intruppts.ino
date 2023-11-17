void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}


void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
    left_wheel_pulse_count++;
  }
  else {
    left_wheel_pulse_count--;
  }
}

void right1_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right1 = false; // Reverse
  }
  else {
    Direction_right1 = true; // Forward
  }
   
  if (Direction_right1) {
    right1_wheel_pulse_count++;
  }
  else {
    right1_wheel_pulse_count--;
  }
}

void left1_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC1_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left1 = false; // Reverse
  }
  else {
    Direction_left1 = true; // Forward
  }
   
  if (Direction_left1) {
    left1_wheel_pulse_count++;
  }
  else {
    left1_wheel_pulse_count--;
  }
}
