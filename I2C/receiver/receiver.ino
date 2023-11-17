#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET; 
int16_t S1,S2,S3,S4;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
 int16_t i1;
 int16_t i2;
 int16_t i3;
 int16_t i4;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;
//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

void setup() {
   Serial.begin(9600);  // start serial for output
   Wire.begin(I2C_SLAVE_ADDRESS);
   //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc. 
  ET.begin(details(mydata), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive);
}

void loop() {
   //delay(500);
}

void receive(int numBytes) {
   if(ET.receiveData()){
    //this is how you access the variables. [name of the group].[variable name]
   S1=mydata.i1;
   S2=mydata.i2;
   S3=mydata.i3;
   S4=mydata.i4;

   Serial.print("S1:");
   Serial.println(S1);
   Serial.print("S2:");
   Serial.println(S2);
   Serial.print("S3:");
   Serial.println(S3);
   Serial.print("S4:");
   Serial.println(S4);

  }
}