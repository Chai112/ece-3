#include <ECE3.h>

uint16_t sensorValues[8];

// CONSTANTS
const int LEFT_NSLP_PIN = 31;
const int LEFT_DIR_PIN  = 29;
const int LEFT_PWM_PIN  = 40;

const int RIGHT_NSLP_PIN = 11;
const int RIGHT_DIR_PIN  = 30;
const int RIGHT_PWM_PIN  = 39;

const int STAGE_1 = 0;
const int STAGE_1_SPD = 100;
const int STAGE_1_KP = 2;
const int STAGE_1_KD = 5;
const int STAGE_1_KI = 0;

const int STAGE_2 = 5.5*360;
const int STAGE_2_SPD = 150;
const int STAGE_2_KP = 3;
const int STAGE_2_KD = 10;
const int STAGE_2_KI = 0;

const int STAGE_3 = 9.5*360;
const int STAGE_3_SPD = 50;
const int STAGE_3_KP = 2;
const int STAGE_3_KD = 2;
const int STAGE_3_KI = 0;

const int STAGE_4 = 11.5*360;
const int STAGE_4_SPD = 150;
const int STAGE_4_KP = 3;
const int STAGE_4_KD = 10;
const int STAGE_4_KI = 0;

const int STAGE_5 = 0;          // TURNAROUND
const int STAGE_5_SPD = 100;
const int STAGE_5_KP = 2;
const int STAGE_5_KD = 5;
const int STAGE_5_KI = 0;

const int STAGE_6 = 14.5*360;
const int STAGE_6_SPD = 50;
const int STAGE_6_KP = 2;
const int STAGE_6_KD = 2;
const int STAGE_6_KI = 0;

const int STAGE_7 = 17*360;
const int STAGE_7_SPD = 150;
const int STAGE_7_KP = 3;
const int STAGE_7_KD = 10;
const int STAGE_7_KI = 0;

const int STAGE_8 = 21*360;
const int STAGE_8_SPD = 100;
const int STAGE_8_KP = 2;
const int STAGE_8_KD = 5;
const int STAGE_8_KI = 0;

const int OFFSET = 40;
const int END = 5000;

// VARIABLES

int spd;

int Kp = 0;
int Kd = 0;
int Ki = 0;

int Ep = 0;
int Ed = 0;
int Ei = 0;

int minimum;
int maximum;
int pos = 0;
int prevEp = 0;

int stage = 0;

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

  digitalWrite(LEFT_DIR_PIN,LOW);
  digitalWrite(RIGHT_DIR_PIN,LOW);
  
  delay(1000);
  
  ChangeBaseSpeeds(0, STAGE_1_SPD, 0, STAGE_1_SPD);
  resetEncoderCount_left();
  resetEncoderCount_right();
  
  do {

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

    Ei = 0;
    
    int encoderCount = (getEncoderCount_left() + getEncoderCount_right()) / 2;
    if (encoderCount > STAGE_4) {
      spd = STAGE_4_SPD;
      Kp = STAGE_4_KP;
      Kd = STAGE_4_KD;
      Ki = STAGE_4_KI;
    } else if (encoderCount > STAGE_3) {
      spd = STAGE_3_SPD;
      Kp = STAGE_3_KP;
      Kd = STAGE_3_KD;
      Ki = STAGE_3_KI;
      if (stage == 2) {
        ChangeBaseSpeeds(STAGE_2_SPD, STAGE_3_SPD, STAGE_2_SPD, STAGE_3_SPD);
        stage = 3;
      }
    } else if (encoderCount > STAGE_2) {
      spd = STAGE_2_SPD;
      Kp = STAGE_2_KP;
      Kd = STAGE_2_KD;
      Ki = STAGE_2_KI;
      stage = 2;
    } else if (encoderCount > STAGE_1) {
      spd = STAGE_1_SPD;
      Kp = STAGE_1_KP;
      Kd = STAGE_1_KD;
      Ki = STAGE_1_KI;
    }

    if (Ep < OFFSET) {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, spd - Kp * Ep - Kd * Ed);
        analogWrite(RIGHT_PWM_PIN, spd);
      }
      else {
        analogWrite(LEFT_PWM_PIN, spd);
        analogWrite(RIGHT_PWM_PIN, spd - Kp * Ep - Kd * Ed);
      }
    }
    else {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, spd);
      }
      else {
        analogWrite(LEFT_PWM_PIN, spd);
        analogWrite(RIGHT_PWM_PIN, 0);
      }
    }
  
  } while (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < END);

  ChangeBaseSpeeds(STAGE_4_SPD, 0, STAGE_4_SPD, 0);
  turnAround();
  ChangeBaseSpeeds(0, STAGE_5_SPD, 0, STAGE_5_SPD);
  do {
    // no-op
  } while (getEncoderCount_left() < 500);

  resetEncoderCount_left();
  resetEncoderCount_right();
  do {

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

    Ei = 0;
    
    int encoderCount = (getEncoderCount_left() + getEncoderCount_right()) / 2;
    if (encoderCount > STAGE_8) {
      spd = STAGE_8_SPD;
      Kp = STAGE_8_KP;
      Kd = STAGE_8_KD;
      Ki = STAGE_8_KI;
    } else if (encoderCount > STAGE_7) {
      spd = STAGE_7_SPD;
      Kp = STAGE_7_KP;
      Kd = STAGE_7_KD;
      Ki = STAGE_7_KI;
    } else if (encoderCount > STAGE_6) {
      spd = STAGE_6_SPD;
      Kp = STAGE_6_KP;
      Kd = STAGE_6_KD;
      Ki = STAGE_6_KI;
    } else if (encoderCount > STAGE_5) {
      spd = STAGE_5_SPD;
      Kp = STAGE_5_KP;
      Kd = STAGE_5_KD;
      Ki = STAGE_5_KI;
    }

    if (Ep < OFFSET) {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, spd - Kp * Ep - Kd * Ed);
        analogWrite(RIGHT_PWM_PIN, spd);
      }
      else {
        analogWrite(LEFT_PWM_PIN, spd);
        analogWrite(RIGHT_PWM_PIN, spd - Kp * Ep - Kd * Ed);
      }
    }
    else {
      if (pos > 0) {
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, spd);
      }
      else {
        analogWrite(LEFT_PWM_PIN, spd);
        analogWrite(RIGHT_PWM_PIN, 0);
      }
    }
  
  } while (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < END);

}

void turnAround() {
  //ChangeBaseSpeeds(BASE_SPEED, 0, BASE_SPEED, 0);
  resetEncoderCount_left();
  digitalWrite(LEFT_DIR_PIN,HIGH);
  analogWrite(LEFT_PWM_PIN, 150);
  analogWrite(RIGHT_PWM_PIN, 150);
//  left
  do {
    // no-op
  } while (getEncoderCount_left() < 350);

  
  digitalWrite(LEFT_DIR_PIN,LOW);
  //ChangeBaseSpeeds(0, BASE_SPEED, 0, BASE_SPEED);
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
