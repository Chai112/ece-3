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
const int KP = 0;
const int KD = 0;
const int KI = 0;
const int BASE_SPEED = 100;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  pinMode(LEFT_NSLP_PIN,OUTPUT);
  pinMode(LEFT_DIR_PIN,OUTPUT);
  pinMode(LEFT_PWM_PIN,OUTPUT);
  pinMode(RIGHT_NSLP_PIN,OUTPUT);
  pinMode(RIGHT_DIR_PIN,OUTPUT);
  pinMode(RIGHT_PWM_PIN,OUTPUT);

  digitalWrite(LEFT_NSLP_PIN,HIGH);
  digitalWrite(RIGHT_NSLP_PIN,HIGH);
  
  delay(1000);
}

void loop()
{
  ECE3_read_IR(sensorValues);
  int pos = ( (sensorValues[0] * -8) +
              (sensorValues[1] * -4) +
              (sensorValues[2] * -2) +
              (sensorValues[3] * -1) +
              (sensorValues[4] * 1) +
              (sensorValues[5] * 2) +
              (sensorValues[6] * 4) +
              (sensorValues[7] * 8) ) * 0.0035;
  Serial.println(pos);
}
