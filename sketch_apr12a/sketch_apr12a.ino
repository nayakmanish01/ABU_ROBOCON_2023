#include <Wire.h>

struct SensorData {
  int8_t val;
  int8_t val1;
};

SensorData sensorData;

void setup() {
  // Wire.setClock(400000);
  Wire.begin(8);        // join I2C bus as slave with address 8
  Wire.onReceive(receiveData);
  Serial.begin(115200);
}

void loop() {
  // Do nothing in the loop function
}

void receiveData(int byteCount) {
  // Read the struct from the master device
  if (byteCount == sizeof(sensorData)) {
    Wire.readBytes((uint8_t *)&sensorData, sizeof(sensorData));
    Serial.print("Temperature: ");
    Serial.println(sensorData.val);
    Serial.print(" Humidity: ");
    Serial.println(sensorData.val1);
  }
}
