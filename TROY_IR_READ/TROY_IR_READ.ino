#include <ECE3.h>

uint16_t sensorValues[8];

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  delay(1000);

  Serial.println("on");
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
