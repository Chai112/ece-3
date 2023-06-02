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

int Ep = 0;
int Ed = 0;
int Ei = 0;

int offset = 0;

void ChangeBaseSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
  /*
  * This function changes the car speed gradually (in about 30 ms) from initial
  * speed to final speed. This non-instantaneous speed change reduces the load
  * on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd - initialLeftSpd;
  int diffRight = finalRightSpd - initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft) / stepIncrement;
  int numStepsRight = abs(diffRight) / stepIncrement;
  int numSteps = max(numStepsLeft, numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft) / numSteps; // left in(de)crement
  int deltaRight = (diffRight) / numSteps; // right in(de)crement
  for(int k = 0 ; k < numSteps ; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(LEFT_PWM_PIN, pwmLeftVal);
    analogWrite(RIGHT_PWM_PIN, pwmRightVal);
    delay(30);
  } // end for int k
  analogWrite(LEFT_PWM_PIN, finalLeftSpd);
  analogWrite(RIGHT_PWM_PIN, finalRightSpd);
} // end void ChangeWheelSpeeds

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

  ChangeBaseSpeeds(0, BASE_SPEED, 0, BASE_SPEED);

  do {
    
    ECE3_read_IR(sensorValues);
    Ep = ( (sensorValues[0] * -8) +
           (sensorValues[1] * -4) +
           (sensorValues[2] * -2) +
           (sensorValues[3] * -1) +
           (sensorValues[4] * 1) +
           (sensorValues[5] * 2) +
           (sensorValues[6] * 4) +
           (sensorValues[7] * 8) ) * 0.0035;

    Ed = abs(Ep) - offset ;
    offset = abs(Ep);

    Ei = 0;

    if (Ep > 10) {
      analogWrite(LEFT_PWM_PIN, BASE_SPEED - Kp * Ep - Kd * Ed);
      analogWrite(RIGHT_PWM_PIN, BASE_SPEED);
    }
    else if (Ep < 10) {
      analogWrite(LEFT_PWM_PIN, BASE_SPEED);
      analogWrite(RIGHT_PWM_PIN, BASE_SPEED - Kp * Ep - Kd * Ed);
    }
    else {
      analogWrite(LEFT_PWM_PIN, BASE_SPEED);
      analogWrite(RIGHT_PWM_PIN, BASE_SPEED);
    }
  
  } while (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 9000);

  ChangeBaseSpeeds(BASE_SPEED, 0, BASE_SPEED, 0);
}
