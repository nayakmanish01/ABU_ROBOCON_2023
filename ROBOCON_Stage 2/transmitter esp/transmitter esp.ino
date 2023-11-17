 #include <Ps3Controller.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
// EasyTransferI2C ET; 

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t x;
  int16_t y;
};

//give a name to the group of data
SEND_DATA_STRUCTURE data;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

void setup(){
  Serial.begin(9600) ;
  Wire.begin();
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  // ET.begin(details(mydata), &Wire);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Ps3.begin("00:1a:7d:da:71:10");
    Serial.println("Ready.");
  
}

void loop(){

  if(Ps3.isConnected()){
     //this is how you access the variables. [name of the group].[variable name]
     
 int X= Ps3.data.analog.stick.lx;
  int Y = -Ps3.data.analog.stick.ly;
//  
  data.x=map(X,-127,127,0,255);
  
  data.y=map(Y,-127,127,0,255);
  Serial.print("x ");
 Serial.println(data.x);
 Serial.print("y ");
Serial.println(data.y);
  //  Serial.println(X);
  // Serial.println(Y);

  //send the data
  Serial2.write((uint8_t*)&data, sizeof(data)); // Send the PS3Data struct as a byte array
  
  delay(100);
  }
  
}