#include <Wire.h>
int x[4];
void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  while (1 < Wire.available()) {
  // loop through all but the last
    String c = Wire.read(); // receive byte as a character
    Serial.print(c); 
  //   Serial.println(); 
  //  for(int i=0;i<howMany;i++){
  //  int d = Wire.read();
  //  x[i]=d;  
  //  Serial.println(d); 
  // }        // print the integer
}        // print the character
}
   