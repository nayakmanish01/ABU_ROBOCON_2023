int readingLSA[2] = {220,220};

void LSA08 :: initLSA(int baud,int OutputEnable){
  pinMode(OutputEnable,OUTPUT);
  digitalWrite(OutputEnable,HIGH);
  for(int i=0;i<2;i++){
    LSAArray[i]->junction_detect[i]=0;  
  }
}


void LSA08 :: sendCommand(char command, char data, char address){
  char checksum = address + command + data;  
  SerialLSA.write(address);
  SerialLSA.write(command);
  SerialLSA.write(data);
  SerialLSA.write(checksum);
}

void LSA08 :: ChangeBaud(char baud, char add){
  char command='R';
  char data;

  if(baud==9600) data=0;
  else if(baud==19200) data=1;
       else if(baud==38400) data=2;
            else if(baud==57600) data=3;
                 else if(baud==115200) data=4;
                      else if(baud==230400) data=5;
   this->sendCommand(command,data,add);
}

void LSA08 :: clearJunction(char add) {
  char address = add;
  char command = 'X';
  
  char data = 0x00;
  this->sendCommand(command,data,address);
}

int LSA08 :: getJunction(char add){
  char address = add;
  char command = 'X';
  char data = 0x01;
  this->sendCommand(command,data,address);

  while(SerialLSA.available() <= 0);
  return (int(SerialLSA.read()));
} 
int LSA08 :: GetByteOfLSA(int OutputEnable){                                            //Initially each and every serialLSAEnX(X=1,2,3) should be HIGH
  int a = 0;
  digitalWrite(OutputEnable,LOW);
  while(SerialLSA.available()<=0);
  a = SerialLSA.read();
  digitalWrite(OutputEnable,HIGH);
  lsa = false;
  return a;   
}
      
float GetLSAReading(uint8_t sensor)
{
  reading = LSAArray[sensor]->GetByteOfLSA(LSAArray[sensor]->OePin);
  reading =  reading-35;
   Serial.println(reading);
  return reading;  
}

void initLSA(void){

  SerialLSA.begin(9600);

  pinMode(LSAArray[0]->OePin,OUTPUT);
  pinMode(LSAArray[1]->OePin,OUTPUT);

  digitalWrite(LSAArray[0]->OePin,HIGH);
  digitalWrite(LSAArray[1]->OePin,HIGH);


  pinMode(LSAArray[0]->JunctionPin,INPUT);
  pinMode(LSAArray[1]->JunctionPin,INPUT);


  digitalWrite(LSAArray[0]->JunctionPin,LOW);
  digitalWrite(LSAArray[1]->JunctionPin,LOW);
 
  
  LSAArray[0]->clearJunction(LSAArray[0]->Address);
  LSAArray[1]->clearJunction(LSAArray[1]->Address);  
}

void LSA08 :: Calibrate(int Sensor)
{
  LSAArray[Sensor]->sendCommand(0x43, 0x05, LSAArray[Sensor]->Address);
}







