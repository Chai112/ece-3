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
const float KP = 20;
const float KD = 40;
const float KI = 0;
const float BASE_SPEED = 100;

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
  int pos = ( (sensorValues[0] - 500) * (1000 / 1210) * -8 +
              (sensorValues[1] - 500) * (1000 / 1234) * -8 +
              (sensorValues[2] - 500) * (1000 / 1021) * -8 +
              (sensorValues[3] - 500) * (1000 / 757) * -8 +
              (sensorValues[4] - 500) * (1000 / 711) * -8 +
              (sensorValues[5] - 500) * (1000 / 806) * -8 +
              (sensorValues[6] - 500) * (1000 / 759) * -8 +
              (sensorValues[7] - 500) * (1000 / 951) * -8 ) * 0.0035
  Serial.println(pos);
}
