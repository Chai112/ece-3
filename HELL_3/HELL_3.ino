#include <ECE3.h>

uint16_t sensorValues[8];

// CONSTANTS
const int LEFT_NSLP_PIN = 31;
const int LEFT_DIR_PIN  = 29;
const int LEFT_PWM_PIN  = 40;

const int RIGHT_NSLP_PIN = 11;
const int RIGHT_DIR_PIN  = 30;
const int RIGHT_PWM_PIN  = 39;

const int END = 5000;

const int STAGE_1;
const int SPEED_1;
const int OFFSET_1;
const int KP_1;
const int KD_1;

const int STAGE_2;
const int SPEED_2;
const int OFFSET_2;
const int KP_2;
const int KD_2;

const int STAGE_3;
const int SPEED_3;
const int OFFSET_3;
const int KP_3;
const int KD_3;

const int STAGE_4;
const int SPEED_4;
const int OFFSET_4;
const int KP_4;
const int KD_4;

const int STAGE_5;
const int SPEED_5;
const int OFFSET_5;
const int KP_5;
const int KD_5;

const int STAGE_6;
const int SPEED_6;
const int OFFSET_6;
const int KP_6;
const int KD_6;

const int STAGE_7;
const int SPEED_7;
const int OFFSET_7;
const int KP_7;
const int KD_7;

const int STAGE_8;
const int SPEED_8;
const int OFFSET_8;
const int KP_8;
const int KD_8;

// VARIABLES

int baseSpeed;
int offset;
int distanceTraveled;

int Kp;
int Kd;

int Ep;
int Ed;

int minimum;
int maximum;
int pos;
int prevEp = 0;

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
  digitalWrite(LEFT_DIR_PIN,LOW);
  digitalWrite(RIGHT_DIR_PIN,LOW);
  
  resetEncoderCount_left();
  resetEncoderCount_right();
  delay(1000);

  baseSpeed = SPEED_1;
  offset = OFFSET_1;
  Kp = KP_1;
  Kd = KD_1;
  
  ChangeBaseSpeeds(0, baseSpeed, 0, baseSpeed);
  do 
  {
    ECE3_read_IR(sensorValues);
    minimum = min(sensorValues[0], min(sensorValues[1], min(sensorValues[2], min(sensorValues[3], min(sensorValues[4], min(sensorValues[5], min(sensorValues[6], sensorValues[7])))))));
    for (int i = 0; i < 8; i++) {
      sensorValues[i] = sensorValues[i] - minimum;
    }
    maximum = max(sensorValues[0], max(sensorValues[1], max(sensorValues[2], max(sensorValues[3], max(sensorValues[4], max(sensorValues[5], max(sensorValues[6], sensorValues[7])))))));
    for (int i = 0; i < 8; i++) {
      sensorValues[i] = sensorValues[i] * 1000 / maximum;
    }
    pos = ( (sensorValues[0] * -4) +
            (sensorValues[1] * -3) +
            (sensorValues[2] * -2) +
            (sensorValues[3] * -1) +
            (sensorValues[4] *  1) +
            (sensorValues[5] *  2) +
            (sensorValues[6] *  3) +
            (sensorValues[7] *  4) ) * 0.01;
    Ep = abs(pos);
    Ed = Ep - prevEp;
    prevEp = Ep;
    if (Ep < offset) {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, BASE_SPEED - Kp * Ep - Kd * Ed);
        analogWrite(RIGHT_PWM_PIN, BASE_SPEED);
      }
      else {
        analogWrite(LEFT_PWM_PIN, BASE_SPEED);
        analogWrite(RIGHT_PWM_PIN, BASE_SPEED - Kp * Ep - Kd * Ed);
      }
    }
    else {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, BASE_SPEED);
      }
      else {
        analogWrite(LEFT_PWM_PIN, BASE_SPEED);
        analogWrite(RIGHT_PWM_PIN, 0);
      }
    }
  } while (distanceTraveled < STAGE_1);

  
}

void loop() {
  
}

void ChangeBaseSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
  /*
  * This function changes the car speed gradually (in about 30 ms) from initial
  * speed to final speed. This non-instantaneous speed change reduces the load
  * on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft)/numSteps; // left in(de)crement
  int deltaRight = (diffRight)/numSteps; // right in(de)crement
  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(LEFT_PWM_PIN,pwmLeftVal);
    analogWrite(RIGHT_PWM_PIN,pwmRightVal);
    delay(30);
  } // end for int k
  analogWrite(LEFT_PWM_PIN,finalLeftSpd);
  analogWrite(RIGHT_PWM_PIN,finalRightSpd);
} // end void ChangeWheelSpeeds

void turnAround() {
  resetEncoderCount_left();
  digitalWrite(LEFT_DIR_PIN,HIGH);
  analogWrite(LEFT_PWM_PIN, 150);
  analogWrite(RIGHT_PWM_PIN, 150);
//  left
  do {
    // no-op
  } while (getEncoderCount_left() < 360);
  digitalWrite(LEFT_DIR_PIN,LOW);
}
