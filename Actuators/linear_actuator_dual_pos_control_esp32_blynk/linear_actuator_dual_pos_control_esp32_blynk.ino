/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL3R0RCYrsk"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "qbG5v1LPOFmTnu3ToXjpVCEEiA28fVbn"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "redmi";
char pass[] = "redmi1234";

BlynkTimer timer;

// define the pins for motor control
const int IN1A = 27;
const int IN2A = 26;
const int IN1B = 19;
const int IN2B = 21;
//const int ENA = 13;

// define the maximum and minimum position values for the actuators
const int MAX_POS = 100;
const int MIN_POS = 0;

// define the target position and current position variables for the two actuators
int currentPosA = 0;
int init_flag = 0;

//const int frequency = 500;
//const int pwm_channel = 0;
//const int resolution = 8;

void runMotor(int targetPos)
{
  while(init_flag != 1) {
//    ledcWrite(pwm_channel,50);
    // read the current positions of the actuators and map them to the range of MIN_POS to MAX_POS
    currentPosA = map(analogRead(34), 0, 4095, MIN_POS, MAX_POS);

    // print the current positions to the serial monitor
    Serial.print("Current Position A: ");
    Serial.print(currentPosA);
    Serial.println();

    // calculate the errors between the target position and the current positions
    int errorA = targetPos - currentPosA;

    // determine the motor directions based on the errors
    if (errorA > 2) {
      digitalWrite(IN1A, HIGH);
      digitalWrite(IN2A, LOW);
    } else if (errorA < 0) {
      digitalWrite(IN1A, LOW);
      digitalWrite(IN2A, HIGH);
    } else {
      digitalWrite(IN1A, LOW);
      digitalWrite(IN2A, LOW);
      Serial.println("Target position A reached");
      init_flag = 1;
    }

    if (errorA > 2) {
      digitalWrite(IN1B, HIGH);
      digitalWrite(IN2B, LOW);
    } else if (errorA < -2) {
      digitalWrite(IN1B, LOW);
      digitalWrite(IN2B, HIGH);
    } else {
      digitalWrite(IN1B, LOW);
      digitalWrite(IN2B, LOW);
      Serial.println("Target position B reached");
    }
    delay(10);
  }
  init_flag = 0;
}

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);

  // Call the function with the appropriate value
  if (value == 0)
  {
    runMotor(18);
  }
  else if (value == 1)
  {
    runMotor(65);
  }
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}

void setup()
{
  // set the motor control pins as outputs
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);
//  pinMode(ENA, OUTPUT);
  
//  // set the initial position of the actuators to the current position variables
//  currentPosA = analogRead(34);
//  ledcSetup(pwm_channel, frequency, resolution);
//  ledcAttachPin(ENA, pwm_channel);
  
  // Debug console
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // You can also specify server:
  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);
+
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
  
}
