#include <ECE3.h>

uint16_t sensorValues[8];
uint16_t offsetValues[8];
char isInitialized = 0;
float leftSpd = 0;
float rightSpd = 0;

const float SUM_OF_SENSORS_MIN = 0.3;

// pins
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

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
    return -999;
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

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  isInitialized = 0;
  Serial.println("initializing");
  // MAKE SURE VEHICLE IS ON A WHITE SURFACE WHEN INIT
  //calibrateWhite(); initialize the white values
  initPins();
  
  Serial.println("init completed");
  isInitialized = 1;
}

void setMotorSpeedLeft(int speed) {
  analogWrite(left_pwm_pin, speed);
}

void setMotorSpeedRight(int speed) {
  analogWrite(right_pwm_pin, speed);
}

void loop()
{
  if (!isInitialized) return;
  
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  setMotorSpeedLeft(20);
  setMotorSpeedRight(20);

  float linePosition = getLinePosition(sensorValues);
  Serial.println(linePosition);
  delay(50);
}
