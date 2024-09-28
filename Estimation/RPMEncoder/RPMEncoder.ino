#include <Encoder.h>  // include the Encoder library

// constants won't change. They're used here to 
// set pin numbers and create the Encoder object:
const int encoderPinA = 2;   // encoder pin A
const int encoderPinB = 3;   // encoder pin B
Encoder encoder(encoderPinA, encoderPinB);  // create the Encoder object

// variables will change:
unsigned long time;          // time used to calculate RPM
int encoderPos;              // current position of the encoder
int lastEncoderPos = 0;      // previous position of the encoder
int rpm;                     // calculated RPM

void setup() {
  // start the serial port:
  Serial.begin(9600);
}

void loop() {
  // read the encoder position:
  encoderPos = encoder.read();

  // check if the encoder position has changed:
  if (encoderPos != lastEncoderPos) {
    // if it has, reset the time:
    time = millis();
  }
  // save the current position as the last position for the next loop:
  lastEncoderPos = encoderPos;

  // check if 1 second has passed:
  if (millis() - ti
  me >= 1000) {
    // if it has, calculate the RPM:
    rpm = (encoderPos - lastEncoderPos) / 2;  // divide by 2 since most encoders have 2 ticks per revolution
    rpm = rpm / (millis() - time) * 1000;  // multiply by 1000 to convert from seconds to milliseconds
    // reset the time and encoder position:
    time = millis();
    lastEncoderPos = encoderPos;
  }

  // print the RPM to the serial port:
  Serial.println(rpm);
}
