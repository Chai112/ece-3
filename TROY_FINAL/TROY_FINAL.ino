#include <ECE3.h>

uint16_t sensorValues[8];

// pins constants
const int LEFT_NSLP_PIN = 31;
const int LEFT_DIR_PIN  = 29;
const int LEFT_PWM_PIN  = 40;

const int RIGHT_NSLP_PIN = 11;
const int RIGHT_DIR_PIN  = 30;
const int RIGHT_PWM_PIN  = 39;

// PID constants
const int Kp = 2;
const int Kd = 1;
const int Ki = 0;
const int BASE_SPEED = 100;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  delay(1000);

  Serial.println("on");
}

void loop()
{
  
}
