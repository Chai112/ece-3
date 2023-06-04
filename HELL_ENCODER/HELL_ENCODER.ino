#include <ECE3.h>

uint16_t sensorValues[8];

// CONSTANTS
const int LEFT_NSLP_PIN = 31;
const int LEFT_DIR_PIN  = 29;
const int LEFT_PWM_PIN  = 40;

const int RIGHT_NSLP_PIN = 11;
const int RIGHT_DIR_PIN  = 30;
const int RIGHT_PWM_PIN  = 39;

const int BASE_SPEED = 100;
const int END = 15000;

const int Kp = 2;
const int Kd = 2;
const int Ki = 0;

// VARIABLES

int Ep = 0;
int Ed = 0;
int Ei = 0;

int minimum;
int maximum;
int pos = 0;
int prevEp = 0;

void setup() {  

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

  resetEncoderCount_left();
  resetEncoderCount_right();

}

void loop() {
 int getL = getEncoderCount_left();
 int getR = getEncoderCount_right();

  int distanceTraveled = ((getEncoderCount_left() + getEncoderCount_right()) / 2);
}
