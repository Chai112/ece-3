#include <ECE3.h>

uint16_t sensorValues[8];

int minimum;
int maximum;
int pos;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  delay(1000);
}

void loop()
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
  
  pos = ( (sensorValues[0] * -8) +
          (sensorValues[1] * -4) +
          (sensorValues[2] * -2) +
          (sensorValues[3] * -1) +
          (sensorValues[4] *  1) +
          (sensorValues[5] *  2) +
          (sensorValues[6] *  4) +
          (sensorValues[7] *  8) ) * 0.01;
  
  Serial.println(pos);
}
