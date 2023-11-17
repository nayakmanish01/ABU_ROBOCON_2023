#include <Wire.h>

struct SensorData {
  int8_t val;
  int8_t val1;
};

SensorData sensorData;

void setup() {
  Wire.begin(8);        // join I2C bus as slave with address 8
  Wire.onReceive(receiveData);
  Serial.begin(9600);
}

void loop() {
  // Do nothing in the loop function
}

void receiveData(int byteCount) {
  // Read the struct from the master device
  if (byteCount == sizeof(sensorData)) {
    Wire.readBytes((uint8_t *)&sensorData, sizeof(sensorData));
    Serial.print("Temperature: ");
    Serial.print(sensorData.val);
    Serial.print(" Humidity: ");
    Serial.print(sensorData.val1);
  }
}
