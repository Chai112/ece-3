#include <ECE3.h>

const float SUM_OF_SENSORS_MIN = 0.3;

// pins constants
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// PID constants
const float kP = 20;
const float kI = 0;
const float kD = 20;
const float SPEED = 100;
const float TURN_COEFF = 50;

// PID variables
const int PREV_LINES = 3;
float previousLinePositions[PREV_LINES];
float integralLinePosition = 0;

uint16_t sensorValues[8];
uint16_t offsetValues[8];
char isInitialized = 0;
float leftSpd = 0;
float rightSpd = 0;

// initialize/calibrate the white values
void calibrateWhite() {
  for (int i = 0; i < 20; i++) {
    ECE3_read_IR(sensorValues);
    delay(50);
  }
  for (int i = 0; i < 8; i++) {
    offsetValues[i] = sensorValues[i];
  }
}


// note: returns 0 if no line detected
// left (+1)     0       right (-1)
float getLinePosition(uint16_t inputSensorValues[]) {
  float sumOfSensors = 0;
  float sumOfSensorsWeighted = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    float offsettedSensorValue = inputSensorValues[i] - offsetValues[i];
    float normalizedSensorValue = offsettedSensorValue / (float)(2500 - offsetValues[i]);
    sumOfSensors += normalizedSensorValue;
    sumOfSensorsWeighted += normalizedSensorValue * (float)(i + 1);
    
    //Serial.print(normalizedSensorValue);
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  if (sumOfSensors > SUM_OF_SENSORS_MIN) {
    
    // sumOfSensorsWeighted/sumOfSensors is centred at 4.5 and goes from 1.5 - 7.5
    float linePosition = ((sumOfSensorsWeighted/sumOfSensors) - 4.5) / 3;
    return linePosition;
  } else {
    return -99;
  }
}

// initialize pin values
void initPins() {

  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

}

void setMotorSpeedLeft(int speed) {
  analogWrite(left_pwm_pin, speed);
}

void setMotorSpeedRight(int speed) {
  analogWrite(right_pwm_pin, speed);
}

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  isInitialized = 0;
  Serial.println("initializing");
  // MAKE SURE VEHICLE IS ON A WHITE SURFACE WHEN INIT
  initPins(); // in
  setMotorSpeedLeft(0);
  setMotorSpeedRight(0);
  calibrateWhite(); // initialize the white values
  delay(2000);
  
  Serial.println("init completed");
  isInitialized = 1;
}

float wheelSpdLeft;
float wheelSpdRight;

float clamp(float d, float min, float max) {
  const float t = d < min ? min : d;
  return t > max ? max : t;
}

void loop()
{
  if (!isInitialized) return;
  
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  float linePosition = getLinePosition(sensorValues);
  if (linePosition == -99) linePosition = previousLinePositions[PREV_LINES - 1];
  //Serial.println(linePosition);

  float derivative = 0;
  for (int i = 0; i < PREV_LINES; i++) {
    derivative += previousLinePositions[i] / PREV_LINES;
  }
  float integral = integralLinePosition;
  float proportional = linePosition;

  // implement PID controller
  float output = kP * proportional + kI * integral + kD * derivative;

  float targetWheelSpdLeft = SPEED - output - TURN_COEFF*abs(linePosition);
  float targetWheelSpdRight = SPEED + output - TURN_COEFF*abs(linePosition);

  wheelSpdLeft += clamp(targetWheelSpdLeft - wheelSpdLeft, -20, 20);
  wheelSpdRight += clamp(targetWheelSpdRight - wheelSpdRight, -20, 20);
  setMotorSpeedLeft(wheelSpdLeft);
  setMotorSpeedRight(wheelSpdRight);

  // update PID values
  for (int i = 0; i < PREV_LINES - 1; i++) {
    previousLinePositions[i] = previousLinePositions[i + 1];
  }
  previousLinePositions[PREV_LINES - 1] = linePosition;
  integralLinePosition += linePosition;
  delay(30);
}
