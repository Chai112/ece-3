#include <ECE3.h>

uint16_t sensorValues[8];

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 30;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  int sum = 0;
  int maxVal = 0;
  int maxValIdx = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    if (sensorValues[i] > maxVal) {
      maxVal = sensorValues[i];
      maxValIdx = i;
    }
  }
  Serial.print(maxValIdx);
  Serial.print('\t');

  int leftWheelSpd = 0;
  int rightWheelSpd = 0;
  switch(maxValIdx) {
    case 0:
      leftWheelSpd = 255;
      rightWheelSpd = 128;
      break;
    case 1:
      leftWheelSpd = 255;
      rightWheelSpd = 170;
      break;
    case 2:
      leftWheelSpd = 255;
      rightWheelSpd = 214;
      break;
    case 3:
      leftWheelSpd = 255;
      rightWheelSpd = 255;
      break;
    case 4:
      leftWheelSpd = 255;
      rightWheelSpd = 255;
      break;
    case 5:
      leftWheelSpd = 214;
      rightWheelSpd = 255;
      break;
    case 6:
      leftWheelSpd = 170;
      rightWheelSpd = 255;
      break;
    case 7:
      leftWheelSpd = 128;
      rightWheelSpd = 255;
      break;
  }

  Serial.print(leftWheelSpd);
  Serial.print("\t");
  Serial.print(rightWheelSpd);
  Serial.print("\t");

  analogWrite(left_pwm_pin, leftWheelSpd);
  analogWrite(right_pwm_pin, rightWheelSpd);
  
  Serial.println();

  delay(50);
}
